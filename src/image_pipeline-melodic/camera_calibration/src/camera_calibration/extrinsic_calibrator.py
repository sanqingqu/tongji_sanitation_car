#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

class ExCalibrator:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/undistort_lower/compressed",
            CompressedImage, self.callback,  queue_size = 1)


    def callback(self, ros_data):

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = ExCalibrator()
    rospy.init_node('extrinsic_calibrator', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
