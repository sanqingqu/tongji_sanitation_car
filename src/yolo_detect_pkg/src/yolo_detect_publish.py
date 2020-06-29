#! /usr/bin/env python
import roslib
import rospy

import numpy as np
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
from yolo_detect_pkg.msg import Ints
from yolo_detect_pkg.msg import Floats
from yolo_detect_pkg.msg import BBoxArray
from yolo_detect_pkg.msg import BBox
from sensor_msgs.msg import CompressedImage

class YOLODetect(object):
    """
    This class is used to wrapper the yolo object detection network with ROS.
    """
    def __init__(self, yolo_func=None, sub_img_topic="/undistort_lower/compressed", pub_result_topic="/yolo_detect/bbox_results"):

        self.yolo_func = yolo_func
        self.sub_img_topic = sub_img_topic
        self.pub_result_topic = pub_result_topic
        self.cv_bridge = CvBridge()
        self.img_subscriber = rospy.Subscriber(self.sub_img_topic, CompressedImage,
                                                self.img_extraction_callback, queue_size=3)

        self.bbox_pulisher = rospy.Publisher(self.pub_result_topic, BBoxArray, queue_size=2)
        self.input_img = None
        self.bbox_np = None
    
    def img_extraction_callback(self, compressed_img_msg):
        # ----------------------------------------------------------------------------
        # Extract the compressed input img and publish the detect bboxes results.
        # ----------------------------------------------------------------------------
        self.input_img = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_img_msg)
        
        self.pulish_detect_bbox(compressed_img_msg.header.stamp)

    def pulish_detect_bbox(self, msg_timestamp):
        # init the bbox_array_msg
        bbox_array_msg = BBoxArray()
        bbox_array_msg.header.stamp = msg_timestamp
        
        if self.yolo_func is not None:
            # the yolo_func output results must be np 1D array, and dtype=np.int32
            # the detection result is **bbox_np** defined as:
            #  [x_top_left, y_top_left, x_bottom_right, y_bottom_right] * n
            self.bbox_np = self.yolo_func(self.input_img)

        else:
            # this branch is just to test.

            self.bbox_np = np.array([393, 113, 584, 471],dtype=np.int32)
            self.bbox_np = np.concatenate((self.bbox_np, self.bbox_np))

        bbox_array_msg.bboxes = self.get_bbox_msg(self.bbox_np)
        self.bbox_pulisher.publish(bbox_array_msg)

    def get_bbox_msg(self, bbox_np):
        bbox_np = bbox_np.reshape(-1, 4)
        bbox_num = len(bbox_np)
        bbox_msg_list = []
        for bbox in bbox_np:
            bbox_msg = BBox()
            bbox_msg.x_top_left = bbox[0]
            bbox_msg.y_top_left = bbox[1]
            bbox_msg.x_bottom_right = bbox[2]
            bbox_msg.y_bottom_right = bbox[3]
            bbox_msg_list.append(bbox_msg)
        return bbox_msg_list

def main():
    rospy.init_node("yolo_based_dumpster_detection", anonymous=True)
    yolo_detection = YOLODetect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS image extraction module."

if __name__ == "__main__":
    main()