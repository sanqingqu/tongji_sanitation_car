#! /usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np

import roslib
import rospy
import time
import os
from sensor_msgs.msg import CompressedImage

class ImageExtraction(object):

    def __init__(self, sub_topic="/undistort_lower/compressed", save_folder=None):
        self.sub_topic = sub_topic
        self.cv_bridge = CvBridge()
        self.subscriber = rospy.Subscriber(self.sub_topic,
                            CompressedImage, self.img_extraction_callback, queue_size=1)
        
        self.save_folder = save_folder

    def img_extraction_callback(self, compressed_img_msg):
        # np_arr = np.fromstring(compressed_img.data, np.uint8)
        # img_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img_np = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_img_msg)
        if self.save_folder is not None:
            img_name = compressed_img_msg.header.stamp
            img_path = os.path.join(self.save_folder, str(img_name) + ".png")
            cv2.imwrite(img_path, img_np)
        cv2.imshow("camera_img",img_np)
        cv2.waitKey(20)


def main():
    img_ext = ImageExtraction(save_folder="/home/sanqingqu/Projects/huanwei_che/tongji_sanitation_car/test_data/record_imgs")
    rospy.init_node("image_extraction", anonymous=True)
    print "init the image extraction node"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS image extraction module."
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()