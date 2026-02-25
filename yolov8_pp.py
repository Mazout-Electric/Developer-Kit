#!/usr/bin/python3
from stai_mpu import stai_mpu_network
import numpy as np

class NeuralNetwork:
    def __init__(self, model_file, input_mean, input_std, conf_threshold, iou_threshold, normalize):
        self._model_file = model_file
        #print(f"[INFO] Initializing NPU Model: {self._model_file}")
        self._conf_threshold = conf_threshold
        self._iou_threshold = iou_threshold

        self.stai_mpu_model = stai_mpu_network(model_path=self._model_file)
        
        self.input_tensor_infos = self.stai_mpu_model.get_input_infos()
        self.output_tensor_infos = self.stai_mpu_model.get_output_infos()

    def iou(self, box1, box2):
        """ Intersection over Union calculation for NMS """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0

    def launch_inference(self, img):
        """
        Match the ST Edge AI YAML Config: scale: 1/255, offset: 0
        """
        input_data = np.expand_dims(img, axis=0).astype(np.float32) / 255.0
        
        in_scale = self.input_tensor_infos[0].get_scale()
        in_zp = self.input_tensor_infos[0].get_zero_point()
        
        input_data = np.clip(np.round(input_data / in_scale) + in_zp, -128, 127).astype(np.int8)
        
        self.stai_mpu_model.set_input(0, input_data)
        self.stai_mpu_model.run()
    def get_results(self):
        """
        Dequantize NPU output and perform Non-Maximum Suppression (NMS).
        """
        raw_output = self.stai_mpu_model.get_output(index=0)
        outputs = np.squeeze(raw_output) # Expected shape: [84, 1344]
        
        scale = self.output_tensor_infos[0].get_scale()
        zp = self.output_tensor_infos[0].get_zero_point()

        output_data = (outputs.astype(np.float32) - zp) * scale
        
        output_data = np.transpose(output_data) 
        
        max_conf = np.max(output_data[:, 4:])
        max_box = np.max(output_data[:, :4])

        boxes = []
        for row in output_data:
            classes_scores = row[4:]
            conf = np.clip(np.max(classes_scores), 0.0, 1.0)
            
            if conf > 0.5: 
                class_id = np.argmax(classes_scores)
                xc, yc, w, h = row[:4]
                
                x1, y1 = (xc - w/2) / 256.0, (yc - h/2) / 256.0
                x2, y2 = (xc + w/2) / 256.0, (yc + h/2) / 256.0
                
                boxes.append([x1, y1, x2, y2, conf, class_id])
        boxes.sort(key=lambda x: x[4], reverse=True)
        final_dets = []
        while len(boxes) > 0:
            chosen = boxes.pop(0)
            final_dets.append(chosen)
            boxes = [b for b in boxes if self.iou(b[:4], chosen[:4]) < self._iou_threshold]
            
        return final_dets
