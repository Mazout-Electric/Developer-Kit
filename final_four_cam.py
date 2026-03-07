import cv2
import numpy as np
import pickle
import sys
import time
import os
import json
import base64
import threading
import queue
import websocket
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

TARGET_FPS = 5
RESOLUTION = (1280, 480)

EYE_RES = (640, 480)  
DEPTH_RES = (320, 240) 

CAMERAS = [
    {"id": 7,  "calib": "cam1_calibration.pkl", "name": "Front"},
    {"id": 9,  "calib": "cam2_calibration.pkl", "name": "Rear"},
    {"id": 11, "calib": "cam3_calibration.pkl", "name": "Right"},
    {"id": 13, "calib": "cam4_calibration.pkl", "name": "Left"}
]

WS_URL = "wss://api.mazoutelectric.com/device-stream"
DEVICE_ID = "MP2_001"
DEVICE_KEY = "668fcc4639eec9010e21c7a87e93e5243c02e4d119ff03408c475338ad47acfa"
payload_queue = queue.Queue(maxsize=1)

from yolov8_pp import NeuralNetwork
coco_labels = ["animal", "auto", "barrier", "bike", "bus", "cars", "coconut stand","dog","gate", "people", "pothole", "sign", "speed-breaker", "truck"]

print("[INFO] Initializing NPU...")
detector = NeuralNetwork(
    model_file="yolov8n_pertensor_1.nb",
    input_mean=128.0, 
    input_std=1.0, 
    conf_threshold=0.45, 
    iou_threshold=0.45, 
    normalize=True
)
npu_lock = threading.Lock() 

global_payloads = {}
payload_lock = threading.Lock()

def on_error(ws, error):
    print("\n[WS ERROR]:", error)

def on_close(ws, close_status_code, close_msg):
    print("\n[WS DISCONNECTED]:", close_status_code, close_msg)

def sender(ws):
    while True:
        payload_str = payload_queue.get() 
        if ws.sock is None or not ws.sock.connected:
            print("\n[WS ERROR] Socket not connected")
            break
        try:
            ws.send(payload_str)
        except Exception:
            print("\n[WS ERROR] Exception Occurred while sending data, Try Again")
            break

def on_open(ws):
    print("\n[INFO] WebSocket Connected! Started Send Thread.")
    sender_thread = threading.Thread(target=sender, args=(ws,), daemon=True)
    sender_thread.start()

def run_websocket():
    ws = websocket.WebSocketApp(
        f"{WS_URL}?deviceId={DEVICE_ID}&key={DEVICE_KEY}",
        on_open=on_open,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever()

class HWJPEGEncoder:
    def __init__(self, width, height):
        pipeline_str = (
            f"appsrc name=src ! video/x-raw,format=BGR,width={width},height={height},framerate={TARGET_FPS}/1 ! "
            f"videoconvert ! v4l2jpegenc ! appsink name=sink max-buffers=1 drop=true"
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.src = self.pipeline.get_by_name("src")
        self.sink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING)

    def encode(self, frame):
        data = frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.src.emit("push-buffer", buf)
        
        sample = self.sink.emit("pull-sample")
        if sample:
            out_buf = sample.get_buffer()
            success, map_info = out_buf.map(Gst.MapFlags.READ)
            if success:
                jpg_bytes = map_info.data
                out_buf.unmap(map_info)
                return jpg_bytes
        return None

class CameraNode(threading.Thread):
    def __init__(self, cam_dict):
        threading.Thread.__init__(self)
        self.cam_dict = cam_dict
        self.name = cam_dict["name"]
        self.running = True
        
        with open(cam_dict["calib"], "rb") as f:
            calib = pickle.load(f)
        
        mtxL, distL = calib["cameraMatrix1"], calib["distCoeffs1"]
        mtxR, distR = calib["cameraMatrix2"], calib["distCoeffs2"]
        R, T = calib["R"], calib["T"]
        self.baseline_cm = abs(T[0][0]) if T.ndim > 1 else abs(T[0])
        
        R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, EYE_RES, R, T, alpha=0.5)
        self.mapLx, self.mapLy = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, EYE_RES, cv2.CV_16SC2)
        self.mapRx, self.mapRy = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, EYE_RES, cv2.CV_16SC2)
        
        self.f_depth_px = P1[0, 0] * (DEPTH_RES[0] / EYE_RES[0])
        
        self.stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)
        self.stereo.setUniquenessRatio(15)
        self.stereo.setSpeckleWindowSize(100)
        self.stereo.setSpeckleRange(32)
        self.stereo.setMinDisparity(0)
        
        cap_pipe = (
            f"v4l2src device=/dev/video{cam_dict['id']} io-mode=2 ! "
            f"video/x-raw, format=YUY2, width={RESOLUTION[0]}, height={RESOLUTION[1]}, framerate={TARGET_FPS}/1 ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink name=sink emit-signals=false max-buffers=1 drop=true"
        )
        self.pipeline = Gst.parse_launch(cap_pipe)
        self.appsink = self.pipeline.get_by_name("sink")
        
        self.encoder = HWJPEGEncoder(EYE_RES[0], EYE_RES[1])
        
        #avi_filename = f"output{cam_dict['id']}.avi"
        #fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #self.out_video = cv2.VideoWriter(avi_filename, fourcc, TARGET_FPS, EYE_RES)
        #print(f"[INFO] Initialized Local Writer: {avi_filename}")

    def run(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        print(f"[INFO] {self.name} Node Online.")
        
        while self.running:
            sample = self.appsink.emit("try-pull-sample", Gst.SECOND)
            if not sample:
                time.sleep(0.01)
                continue
                
            buf = sample.get_buffer()
            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success: continue
            frame = np.frombuffer(map_info.data, np.uint8).reshape((RESOLUTION[1], RESOLUTION[0], 3)).copy()
            buf.unmap(map_info)

            raw_l, raw_r = frame[:, :640], frame[:, 640:]
            rectL = cv2.remap(raw_l, self.mapLx, self.mapLy, cv2.INTER_LINEAR)
            rectR = cv2.remap(raw_r, self.mapRx, self.mapRy, cv2.INTER_LINEAR)

            gL = cv2.resize(cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY), DEPTH_RES)
            gR = cv2.resize(cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY), DEPTH_RES)
            disp = self.stereo.compute(gL, gR).astype(np.float32) / 16.0
            disp[disp <= 0] = 0.1
            depth_map = (self.f_depth_px * self.baseline_cm) / disp

            img_npu = cv2.cvtColor(cv2.resize(rectL, (256, 256)), cv2.COLOR_BGR2RGB)
            
            with npu_lock:
                detector.launch_inference(img_npu)
                detections = list(detector.get_results()) 

            dataList = []
            for x1, y1, x2, y2, conf, cls_id in detections:
                sx = int(((x1 + x2) / 2) * DEPTH_RES[0])
                sy = int(((y1 + y2) / 2) * DEPTH_RES[1])
                
                x_start, x_end = max(0, sx-3), min(DEPTH_RES[0], sx+3)
                y_start, y_end = max(0, sy-3), min(DEPTH_RES[1], sy+3)
                roi_depth = depth_map[y_start:y_end, x_start:x_end]
                
                valid_depths = roi_depth[roi_depth > 0]
                dist_val = np.median(valid_depths) if len(valid_depths) > 0 else 0

                if dist_val > 1000 or dist_val <= 0:
                    display_dist = "NA"
                    payload_dist = "NA"
                else:
                    display_dist = f"{int(dist_val)}cm"
                    payload_dist = int(dist_val)

                ix1, iy1 = int(x1 * EYE_RES[0]), int(y1 * EYE_RES[1])
                ix2, iy2 = int(x2 * EYE_RES[0]), int(y2 * EYE_RES[1])
                
                ix1, iy1 = max(0, ix1), max(0, iy1)
                ix2, iy2 = min(EYE_RES[0], ix2), min(EYE_RES[1], iy2)
                
                obj_name = coco_labels[int(cls_id)] if int(cls_id) < len(coco_labels) else "Object"
                dataList.append({obj_name: payload_dist})
                
                cv2.rectangle(rectL, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
                cv2.putText(rectL, f"{obj_name} {display_dist}", (ix1, iy1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            #self.out_video.write(rectL)

            jpg_bytes = self.encoder.encode(rectL)
            
            if jpg_bytes:
                with payload_lock:
                    global_payloads[self.name] = {
                        "cam_name": self.name,
                        "image": base64.b64encode(jpg_bytes).decode('utf-8'),
                        "dataList": dataList
                    }

    def stop(self):
        self.running = False
        self.pipeline.set_state(Gst.State.NULL)
        self.encoder.pipeline.set_state(Gst.State.NULL)
        #if self.out_video:
            #self.out_video.release()

threading.Thread(target=run_websocket, daemon=True).start()

nodes = []
for cam in CAMERAS:
    node = CameraNode(cam)
    node.start()
    nodes.append(node)
    time.sleep(1.0) 

print("\n[INFO] All Cameras Active. Entering payload assembly loop...\n")

try:
    while True:
        start_time = time.time()
        
        with payload_lock:
            payload = list(global_payloads.values())
        payload_json = json.dumps(payload)
        
        if not payload_queue.full():
            payload_queue.put(payload_json)
        
        
        time.sleep(0.1) 

except KeyboardInterrupt:
    print("\n[INFO] Shutting down...")
    for node in nodes:
        node.stop()
        node.join()
    print("[INFO] System offline")
