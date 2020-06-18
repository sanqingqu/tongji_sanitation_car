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


cfgs= EasyDict()
cfgs.CAMERA_ID = 1
cfgs.N_CHESS_BORAD_WIDTH = 7
cfgs.N_CHESS_BORAD_HEIGHT = 5
cfgs.CHESS_BOARD_SIZE = lambda: (cfgs.N_CHESS_BORAD_WIDTH, cfgs.N_CHESS_BORAD_HEIGHT)
cfgs.SQUARE_SIZE_M = 0.0583
cfgs.N_CALIBRATE_SIZE = 10
cfgs.FIND_CHESSBOARD_DELAY_MOD = 4
cfgs.FOCAL_SCALE = 1.0
cfgs.MAX_READ_FAIL_CTR = 10

flags = EasyDict()
flags.frame_id = 0

BOARD = np.array([ [(j * cfgs.SQUARE_SIZE_M, i * cfgs.SQUARE_SIZE_M, 0.)]
    for i in range(cfgs.N_CHESS_BORAD_HEIGHT) for j in range(cfgs.N_CHESS_BORAD_WIDTH) ])


class calib_t(EasyDict):
    def __init__(self):
        super(calib_t, self).__init__({
        "type":None,
        "camera_mat":None,
        "dist_coeff":None,
        "rvecs":None,
        "tvecs":None,
        "map1":None,
        "map2":None,
        "reproj_err":None,
        "ok":False,
        })

class Fisheye:
    def __init__(self):
        self.data = calib_t()
        self.inited = False
    def update(self, corners, frame_size):
        board = [BOARD] * len(corners)
        if not self.inited:
            self._update_init(board, corners, frame_size)
            self.inited = True
        else:
            self._update_refine(board, corners, frame_size)
        #self._calc_reproj_err(corners)
    def _update_init(self, board, corners, frame_size):
        data = self.data
        data.type = "FISHEYE"
        data.camera_mat = np.eye(3, 3)
        data.dist_coeff = np.zeros((4, 1))
        data.ok, data.camera_mat, data.dist_coeff, data.rvecs, data.tvecs = cv2.fisheye.calibrate(
            board, corners, frame_size, data.camera_mat, data.dist_coeff,
            flags=cv2.fisheye.CALIB_FIX_SKEW|cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC,
            criteria=(cv2.TERM_CRITERIA_COUNT, 30, 0.1))
        data.ok = data.ok and cv2.checkRange(data.camera_mat) and cv2.checkRange(data.dist_coeff)
    def _update_refine(self, board, corners, frame_size):
        data = self.data
        data.ok, data.camera_mat, data.dist_coeff, data.rvecs, data.tvecs = cv2.fisheye.calibrate(
            board, corners, frame_size, data.camera_mat, data.dist_coeff,
            flags=cv2.fisheye.CALIB_FIX_SKEW|cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC|cv2.CALIB_USE_INTRINSIC_GUESS,
            criteria=(cv2.TERM_CRITERIA_COUNT, 10, 0.1))
        data.ok = data.ok and cv2.checkRange(data.camera_mat) and cv2.checkRange(data.dist_coeff)
    def _calc_reproj_err(self, corners):
        if not self.inited: return
        data = self.data
        data.reproj_err = []
        for i in range(len(corners)):
            corners_reproj = cv2.fisheye.projectPoints(BOARD[i], data.rvecs[i], data.tvecs[i], data.camera_mat, data.dist_coeff)
            err = cv2.norm(corners_reproj, corners[i], cv2.NORM_L2);
            data.reproj_err.append(err)

class data_t(EasyDict):
    def __init__(self, raw_frame):
        super(data_t, self).__init__({
        "raw_frame":raw_frame,
        "corners":None,
        "ok":False,
        })
        # find chess board
        self.ok, self.corners = cv2.findChessboardCorners(self.raw_frame, cfgs.CHESS_BOARD_SIZE(),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK)
        if not self.ok: return
        # subpix
        gray = cv2.cvtColor(self.raw_frame, cv2.COLOR_BGR2GRAY)
        self.corners = cv2.cornerSubPix(gray, self.corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.1))
        
class history_t:
    def __init__(self):
        self.corners = []
        self.updated = False
    def append(self, current):
        if not current.ok: return
        self.corners.append(current.corners)
        self.updated = True
    def removei(self, i):
        if not 0 <= i < len(self): return
        del self.corners[i]
        self.updated = True
    def __len__(self):
        return len(self.corners)
    def get_corners(self):
        self.updated = False
        return self.corners


history = history_t()
fisheye = Fisheye()
calib = None

def callback(rgb_msg):
  global calib
  raw_frame = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
  if 0 == flags.frame_id % cfgs.FIND_CHESSBOARD_DELAY_MOD:
      current = data_t(raw_frame)
      history.append(current)

  if len(history) >= cfgs.N_CALIBRATE_SIZE and history.updated:
      fisheye.update(history.get_corners(), raw_frame.shape[1::-1])
      calib = fisheye.data
      calib.map1, calib.map2 = cv2.fisheye.initUndistortRectifyMap(
          calib.camera_mat, calib.dist_coeff, np.eye(3, 3), calib.camera_mat, raw_frame.shape[1::-1], cv2.CV_16SC2)

  if len(history) >= cfgs.N_CALIBRATE_SIZE:
      undist_frame = cv2.remap(raw_frame, calib.map1, calib.map2, cv2.INTER_LINEAR);
      cv2.imshow("undist_frame", undist_frame)

  cv2.imshow("raw_frame", raw_frame)
  key = cv2.waitKey(1)

if __name__ == '__main__':
  rospy.init_node('calibrator_tj', anonymous=True)
  image_sub = message_filters.Subscriber('/image_raw', Image)
  ts = message_filters.ApproximateTimeSynchronizer([image_sub], 10, 0.2)
  ts.registerCallback(callback)
  rospy.spin()


script = """
import cv2
import numpy as np
camera_mat = np.array({})
dist_coeff = np.array({})
frame_shape = None
def undistort(frame):
    global frame_shape
    if frame_shape is None:
        frame_shape = frame.shape[1::-1]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            camera_mat, dist_coeff, np.eye(3, 3), camera_mat, frame_shape, cv2.CV_16SC2)
    undist_frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR);
    return undist_frame
""".format(fisheye.data.camera_mat.tolist(), fisheye.data.dist_coeff.tolist())
print(script)
