import torch
import numpy as np
from detector import Detector

class ImageDumpsterDetector:
    def __init__(self, model_def="/home/haitao/Project/tongji_sanitation_car/Yolo/config/yolov3-custom.cfg",
                load_path="/home/haitao/Project/tongji_sanitation_car/Yolo/parameters/yolov3_ckpt_base.pth"):
        self.yolo = Detector(torch.device("cpu"), model_def, load_path,
                             reg_threshold=0.9, cls_threshold=0.2, nms_threshold=0.2, image_size=416)
    def __call__(self, input_img_cur):
        filter = [1, 2]
        # detection result
        # print(type(input_img_cur))
        detection = self.yolo(input_img_cur)
        bbox_list = []
        if detection is None: return bbox_list
        for x1, y1, x2, y2, conf, cls_conf, cls_pred in detection:
            #if int(cls_pred) in filter:
            bbox = []
            bbox = np.array([x1, y1, x2, y2], dtype=np.int32)
            bbox_list.append(bbox)
        return bbox_list