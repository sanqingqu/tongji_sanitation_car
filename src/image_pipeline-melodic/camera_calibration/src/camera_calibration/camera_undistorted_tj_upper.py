#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
from cv_bridge import CvBridge
 
from sensor_msgs.msg import CameraInfo, Image

import numpy as np
from easydict import EasyDict

# 1920x1080
#camera_mat = np.array([[367.2867502156721, 0.0, 960.8303932411943], [0.0, 369.06857590098275, 562.6211777029157], [0.0, 0.0, 1.0]])
#dist_coeff = np.array([[0.05820771148994614], [0.011886625709185384], [-0.024279444196835236], [0.006027933828260825]])

# 3264x2448
camera_mat = np.array([[613.7186360965562, 0.0, 1614.0450603510321], [0.0, 616.7516654519629, 1256.4036912053894], [0.0, 0.0, 1.0]])
dist_coeff = np.array([[0.056647568760584514], [0.020531679323314195], [-0.029998310675196704], [0.008229496587195233]])

cap = cv2.VideoCapture(1)
assert cap.isOpened(), "capture open failed"
(cap.set(cv2.CAP_PROP_FPS, 20))
(cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920))
(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080))
(cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G')))
(cap.set(cv2.CAP_PROP_GAIN, 0.4))
(cap.set(cv2.CAP_PROP_BRIGHTNESS, 0))
(cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25))
(cap.set(cv2.CAP_PROP_EXPOSURE, 0.5))

frame_shape = None
map1 = None
map2 = None
def undistort(frame):
    global frame_shape
    global map1
    global map2
    if frame_shape is None:
        frame_shape = frame.shape[1::-1]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            camera_mat, dist_coeff, np.eye(3, 3), camera_mat, frame_shape, cv2.CV_16SC2)
    undist_frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR);
    return undist_frame

def callback(rgb_msg):
  global calib
  bridge = CvBridge()
  img_pub = rospy.Publisher('undistorted_2', Image, queue_size=1)
  
  raw_frame = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
  undist_frame = undistort(raw_frame)
  img_pub.publish(bridge.cv2_to_imgmsg(undist_frame,"bgr8"))
  
  resized_frame = cv2.resize(undist_frame, (0, 0), fx=0.4, fy=0.4)
  cv2.imshow("undist_frame", resized_frame)
  key = cv2.waitKey(1)

def ros_main():
  rospy.init_node('undistort_tj', anonymous=True)
  image_sub = message_filters.Subscriber('/image_raw', Image)
  ts = message_filters.ApproximateTimeSynchronizer([image_sub], 30, 0.05)
  ts.registerCallback(callback)
  rospy.spin()

def cv_main():
  cap = cv2.VideoCapture(0)
  assert cap.isOpened(), "capture open failed"
  (cap.set(cv2.CAP_PROP_FPS, 20))
  (cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3264))
  (cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2448))
  (cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G')))
  (cap.set(cv2.CAP_PROP_GAIN, 0.4))
  (cap.set(cv2.CAP_PROP_BRIGHTNESS, 0))
  (cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25))
  (cap.set(cv2.CAP_PROP_EXPOSURE, 0.05))
  while True:
    ok, raw_frame = cap.read()
    #print(raw_frame.shape)
    undist_frame = undistort(raw_frame)
    resized_frame = cv2.resize(undist_frame, (0, 0), fx=0.4, fy=0.4)
    cv2.imshow("undist_frame", resized_frame)
    key = cv2.waitKey(1)
    if key in [27, 'q']:
        break

if __name__ == '__main__':
  ros_main()