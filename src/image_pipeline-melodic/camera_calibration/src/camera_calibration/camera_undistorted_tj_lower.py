#!/usr/bin/python

# http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

from __future__ import print_function
import cv2
import message_filters
import numpy
import os
import rospy
import threading
import time

import message_filters
import cv2
import rospy
import sys
from cv_bridge import CvBridge
 
from sensor_msgs.msg import CameraInfo, Image, CompressedImage

import numpy as np
from easydict import EasyDict
# 1920x1080
camera_mat = np.array([[367.0543074903704, 0.0, 990.6330750325744], [0.0, 366.7370079611347, 575.1183044201284], [0.0, 0.0, 1.0]])
dist_coeff = np.array([[0.06062150734934245], [-0.008279891153400248], [-0.0012545281813805395], [-0.0010038515782001421]])
new_camera_mat = np.array([[367.0543074903704 * 0.5, 0.0, 990.6330750325744 * 0.5], [0.0, 366.7370079611347 * 0.5, 575.1183044201284 * 0.5], [0.0, 0.0, 1.0]])

cap = cv2.VideoCapture(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
assert cap.isOpened(), "capture open failed"
(cap.set(cv2.CAP_PROP_FPS, 20))
(cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920))
(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080))
(cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G')))
(cap.set(cv2.CAP_PROP_GAIN, 0.4))
(cap.set(cv2.CAP_PROP_BRIGHTNESS, 0))
(cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)) # 0.25 mannual, 0.75 auto
#(cap.set(cv2.CAP_PROP_EXPOSURE, 0.5))

frame_shape = None
map1 = None
map2 = None
def undistort(frame):
    global frame_shape
    global map1
    global map2
    if frame_shape is None:
        frame_shape = tuple(i // 2 for i in frame.shape[1::-1])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            camera_mat, dist_coeff, np.eye(3, 3), new_camera_mat, frame_shape, cv2.CV_16SC2)
    undist_frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR);
    return undist_frame
 
def ros_main(raw=False, compressed=False, resized=True, viz=False):
  img_pub_raw = rospy.Publisher('/undistort_lower', Image, queue_size = 2)
  img_pub_compressed = rospy.Publisher('/undistort_lower/compressed', CompressedImage, queue_size = 2)
  img_pub_resized = rospy.Publisher('/undistort_lower_resized', Image, queue_size = 2)
  rospy.init_node('undistort_lower', anonymous=True)
  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
    ok, raw_frame = cap.read()
    timestamp = rospy.Time.now()
    undist_frame = undistort(raw_frame)

    # raw
    bridge = CvBridge()
    if raw:
      img_pub_raw.publish(bridge.cv2_to_imgmsg(undist_frame,"bgr8"))

    # compressed
    if compressed:
      cmsg = CompressedImage()
      cmsg.header.stamp = timestamp
      cmsg.format = "jpeg"
      cmsg.data = np.array(cv2.imencode('.jpg', undist_frame)[1]).tostring()
      img_pub_compressed.publish(cmsg)

    resized_frame = None
    if resized:
      resized_frame = cv2.resize(raw_frame, (0, 0), fx=0.5, fy=0.5)
      img_pub_resized.publish(bridge.cv2_to_imgmsg(resized_frame,"bgr8"))
    
    if viz:
      #if resized_frame is None:
      #  resized_frame = cv2.resize(undist_frame, (0, 0), fx=0.4, fy=0.4)
      cv2.imshow("undist_frame", undist_frame)
      cv2.imshow("resized", resized_frame)
      key = cv2.waitKey(1)

    rate.sleep()

def cv_main():
  #cap = cv2.VideoCapture(0)
  #assert cap.isOpened(), "capture open failed"
  #(cap.set(cv2.CAP_PROP_FPS, 20))
  #(cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920))
  #(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080))
  #(cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G')))
  #(cap.set(cv2.CAP_PROP_GAIN, 0.4))
  #(cap.set(cv2.CAP_PROP_BRIGHTNESS, 0))
  #(cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25))
  #(cap.set(cv2.CAP_PROP_EXPOSURE, 0.5))
  while True:
    ok, raw_frame = cap.read()
    #print(raw_frame.shape)
    undist_frame = undistort(raw_frame)
    #resized_frame = cv2.resize(undist_frame, (0, 0), fx=0.4, fy=0.4)
    cv2.imshow("undist_frame", undist_frame)
    key = cv2.waitKey(1)
    if key in [27, 'q']:
        break
  exit()

if __name__ == '__main__':
  #cv_main()
  try:
    ros_main(raw=False, compressed=True, resized=True, viz=True)
  except rospy.ROSInterruptException:
    pass
