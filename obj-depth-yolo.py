import cv2
import numpy as np
import pickle
import sys
import time
import os
import websocket
import json
import base64
import threading
import queue

CALIB_FILE = "stereo_calibration_data.pkl"
#OUTPUT_FILE = 'zooty_log.avi'
payload_queue = queue.Queue(maxsize=1)
WS_URL = "wss://api.mazoutelectric.com/device-stream"
DEVICE_ID = "MP2_001"
DEVICE_KEY="668fcc4639eec9010e21c7a87e93e5243c02e4d119ff03408c475338ad47acfa"

if not os.path.exists(CALIB_FILE):
    print(f"[ERROR] Calibration file {CALIB_FILE} not found!")
    sys.exit(1)

with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = np.array(calib["cameraMatrix1"]), np.array(calib["distCoeffs1"])
mtxR, distR = np.array(calib["cameraMatrix2"]), np.array(calib["distCoeffs2"])
R, T = np.array(calib["R"]), np.array(calib["T"])
baseline_cm = abs(T[0][0]) if T.ndim > 1 else abs(T[0])

out_w, out_h = 640, 360
depth_w, depth_h = 320, 180

R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, (1280, 720), R, T, alpha=0.5, newImageSize=(out_w, out_h))
mapLx, mapLy = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, (out_w, out_h), cv2.CV_16SC2)
mapRx, mapRy = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, (out_w, out_h), cv2.CV_16SC2)
f_depth_px = P1[0, 0] * (depth_w / out_w)

from yolov8_pp import NeuralNetwork

coco_labels = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]
detector = NeuralNetwork(
    model_file="yolov8n_pertensor.nb",
    input_mean=128.0, 
    input_std=1.0, 
    conf_threshold=0.3, # Raised to 0.45 to eliminate background noise
    iou_threshold=0.45, 
    normalize=True
)

stereo = cv2.StereoBM_create(numDisparities=32, blockSize=15)

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Disconnected:", close_status_code, close_msg)

def sender(ws):
    while True:
        payload = payload_queue.get()
        img=payload['image']
        if ws.sock is None or not ws.sock.connected:
            print("Socket not connected")
            break
        try:
            _, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            data = json.dumps({"image": base64.b64encode(enc).decode('utf-8'),"listData":payload['dataList']})
            #print(len(data))
            ws.send(data)
        except Exception:
            print("Exception Occured while sending data, Try Again")
            break

def on_open(ws):
    print("Started Send Thread")
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

threading.Thread(target=run_websocket, daemon=True).start()

cap = cv2.VideoCapture(7, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#out_video = cv2.VideoWriter(OUTPUT_FILE, cv2.VideoWriter_fourcc(*'XVID'), 10.0, (out_w, out_h))

#print("[INFO] Zooty Vision Online (High-Confidence Mode)...")

try:
    while True:
        start_time = time.time()
        ret, frame = cap.read()
        if not ret: continue

        raw_l, raw_r = frame[:, :1280], frame[:, 1280:]
        rectL = cv2.remap(raw_l, mapLx, mapLy, cv2.INTER_LINEAR)
        rectR = cv2.remap(raw_r, mapRx, mapRy, cv2.INTER_LINEAR)

        img_npu = cv2.resize(rectL, (256, 256))
        img_npu = cv2.cvtColor(img_npu, cv2.COLOR_BGR2RGB)
        detector.launch_inference(img_npu)
        detections = detector.get_results()

        gL = cv2.resize(cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY), (depth_w, depth_h))
        gR = cv2.resize(cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY), (depth_w, depth_h))
        disp = stereo.compute(gL, gR).astype(np.float32) / 16.0
        disp[disp <= 0] = 0.1
        depth_map = (f_depth_px * baseline_cm) / disp

        dataList=[]
        for x1, y1, x2, y2, conf, cls_id in detections:
            sx = int(((x1 + x2) / 2) * depth_w)
            sy = int(((y1 + y2) / 2) * depth_h)
            
            x_start, x_end = max(0, sx-3), min(depth_w, sx+3)
            y_start, y_end = max(0, sy-3), min(depth_h, sy+3)
            roi_depth = depth_map[y_start:y_end, x_start:x_end]
            
            valid_depths = roi_depth[(roi_depth > 0) & (roi_depth < 1000)]
            if len(valid_depths) > 0:
                dist_val = np.median(valid_depths)
            else:
                dist_val = 0

            ix1, iy1 = int(x1 * out_w), int(y1 * out_h)
            ix2, iy2 = int(x2 * out_w), int(y2 * out_h)
            
            ix1, iy1 = max(0, ix1), max(0, iy1)
            ix2, iy2 = min(out_w, ix2), min(out_h, iy2)
            
            obj_name = coco_labels[int(cls_id)] if int(cls_id) < len(coco_labels) else "Object"
            dataList.append({obj_name:int(dist_val)})
            color = (0, 255, 0) # Green
            cv2.rectangle(rectL, (ix1, iy1), (ix2, iy2), color, 2)
            cv2.putText(rectL, f"{obj_name} {int(dist_val)}cm", (ix1, iy1-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        #out_video.write(rectL)
        if not payload_queue.full():
            payload={
                "image":rectL,
                "dataList":dataList
                }
            payload_queue.put(payload)
        
        curr_fps = 1.0 / (time.time() - start_time)
except KeyboardInterrupt:
    print("\n[INFO] Saving buffers and exiting...")
finally:
    cap.release()
    #out_video.release()
