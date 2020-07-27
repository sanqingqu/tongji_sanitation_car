import rospy

from cv_bridge import CvBridge
from yolo_detect_pkg.msg import BBoxArray
from yolo_detect_pkg.msg import BBox
from sensor_msgs.msg import CompressedImage

import torch
from detector import Detector


class YOLODetect(object):

    def __init__(self,
                 sub_img_topic="/undistort_lower/compressed",
                 pub_result_topic="/yolo_detect/bbox_results",
                 model_def="config/yolov3-10FPS.cfg",
                 load_path="parameters/yolov3_ckpt_2333.pth"):
        # model
        self.cv_bridge = CvBridge()
        self.yolo = Detector(torch.device("cuda:0"), model_def, load_path,
                             reg_threshold=0.9, cls_threshold=0.2, nms_threshold=0.2, image_size=96)
        # ros
        self.img_subscriber = rospy.Subscriber(sub_img_topic, CompressedImage,
                                               self.img_extraction_callback, queue_size=3)
        self.bbox_publisher = rospy.Publisher(pub_result_topic, BBoxArray, queue_size=2)
        # other
        self.filter = [1, 2]

    def img_extraction_callback(self, compressed_img_msg):
        input_img = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_img_msg)
        detection = self.yolo(input_img)

        bbox_msg_list = []
        for x1, y1, x2, y2, conf, cls_conf, cls_pred in detection:
            if int(cls_pred) in self.filter:
                bbox_msg = BBox()
                bbox_msg.x_top_left = x1
                bbox_msg.y_top_left = y1
                bbox_msg.x_bottom_right = x2
                bbox_msg.y_bottom_right = y2
                bbox_msg_list.append(bbox_msg)

        bbox_array_msg = BBoxArray()
        bbox_array_msg.header.stamp = compressed_img_msg.header.stamp
        bbox_array_msg.bboxes = bbox_msg_list
        self.bbox_publisher.publish(bbox_array_msg)


if __name__ == "__main__":
    rospy.init_node("yolo_based_dumpster_detection", anonymous=True)
    yolo_detection = YOLODetect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS image extraction module.")
