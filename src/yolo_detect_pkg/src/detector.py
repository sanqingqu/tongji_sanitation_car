import torch
import torch.nn.functional as f

from models import Darknet
from utils import non_max_suppression, rescale_boxes


class Detector(object):
    def __init__(self, device, model_def, load_path, reg_threshold, cls_threshold, nms_threshold, image_size):
        self.image_size = image_size
        self.model = Darknet(model_def, img_size=self.image_size).to(device)
        self.model.load_state_dict(torch.load(load_path))
        self.model.eval()
        self.reg_threshold = reg_threshold
        self.cls_threshold = cls_threshold
        self.nms_threshold = nms_threshold

        self.device = device

    @torch.no_grad()
    def __call__(self, image):
        original_size = image.shape[:2]
        tensor = torch.from_numpy(image).to(self.device).permute(2, 0, 1)
        tensor = tensor.contiguous().float().div_(255)
        _, h, w = tensor.shape
        dim_diff = np.abs(h - w)
        pad1, pad2 = dim_diff // 2, dim_diff - dim_diff // 2
        pad = (0, 0, pad1, pad2) if h <= w else (pad1, pad2, 0, 0)
        tensor = f.pad(tensor, pad, "constant", value=0)
        tensor = f.interpolate(tensor.unsqueeze(0), size=self.image_size, mode="nearest").squeeze_(0)

        result = self.model(tensor.unsqueeze_(0))
        detection = non_max_suppression(result, self.reg_threshold, self.nms_threshold)[0]
        if detection is not None:
            detection = detection[detection[:, -2] > self.cls_threshold]
            detection = rescale_boxes(detection, self.image_size, original_size)
        return detection


if __name__ == "__main__":
    import json
    import os
    from time import time

    import cv2
    import numpy as np

    # os.makedirs("outputs", exist_ok=True)
    os.environ['CUDA_VISIBLE_DEVICES'] = "0"


    def draw_result(image, detection, save_path):
        def change(value):
            return int(round(value.item()))

        if detection is not None:
            for x1, y1, x2, y2, conf, cls_conf, cls_pred in detection:
                color = classes_color[int(cls_pred)]
                x1, y1, x2, y2 = change(x1), change(y1), change(x2), change(y2)
                cv2.rectangle(image, (x1, y1), (x2, y2), color)
                label = "%s %.2f %.2f" % (classes_name[int(cls_pred)], conf, cls_conf)
                cv2.putText(image, label, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                # print("\t", label, x1, y1, x2, y2)
        cv2.imwrite(save_path, image)


    # Load model
    with open("config/prepare.json", 'r') as load_f:
        prepare_info = json.load(load_f)
    classes_define = prepare_info["classes"]
    classes_name = list(classes_define.keys())
    classes_color = [eval(classes_define[class_name]['color']) for class_name in classes_name]

    demo_detector_10fps = Detector(device=torch.device("cuda:0"),
                                   model_def="config/yolov3-10FPS.cfg",
                                   load_path="parameters/yolov3_ckpt_2333.pth",
                                   reg_threshold=0.9,
                                   cls_threshold=0.2,
                                   nms_threshold=0.2,
                                   image_size=96)

    demo_detector_1fps = Detector(device=torch.device("cuda:0"),
                                  model_def="config/yolov3-custom.cfg",
                                  load_path="parameters/yolov3_ckpt_base.pth",
                                  reg_threshold=0.9,
                                  cls_threshold=0.2,
                                  nms_threshold=0.2,
                                  image_size=416)

    # Load images
    path_list = os.listdir("custom/images/")[:100]
    for index, path in enumerate(path_list, 1):
        demo_image = cv2.imread("custom/images/" + path)

        time_stamp = time()
        demo_det = demo_detector_10fps(demo_image)
        print(index, "FPS:", 1 / (time() - time_stamp))

        draw_result(demo_image, demo_det, 'outputs/%s_10fps.jpg' % index)

        demo_image = cv2.imread("custom/images/" + path)

        time_stamp = time()
        demo_det = demo_detector_1fps(demo_image)
        print(index, "FPS:", 1 / (time() - time_stamp))

        draw_result(demo_image, demo_det, 'outputs/%s_1fps.jpg' % index)
