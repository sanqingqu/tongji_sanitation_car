from __future__ import print_function
import cv2
import message_filters
import numpy as np
import os
import rospy
import threading
import time
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import message_filters
import cv2
import rospy
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import math

# 1920x1080
#camera_mat = np.array([[367.2867502156721, 0.0, 960.8303932411943], [0.0, 369.06857590098275, 562.6211777029157], [0.0, 0.0, 1.0]])
#dist_coeff = np.array([[0.05820771148994614], [0.011886625709185384], [-0.024279444196835236], [0.006027933828260825]])

# 3264x2448
camera_mat = np.array([[367.0543074903704, 0.0, 990.6330750325744], [0.0, 366.7370079611347, 575.1183044201284], [0.0, 0.0, 1.0]])
dist_coeff = np.array([[0.06062150734934245], [-0.008279891153400248], [-0.0012545281813805395], [-0.0010038515782001421]])

roll = 1
pitch = 1
yaw = 1
translation = np.array([1, 1, 1, 1])

camera_mat_1 = np.array([[367.0543074903704, 0.0, 990.6330750325744, 0], [0.0, 366.7370079611347, 575.1183044201284, 0], [0.0, 0.0, 1.0, 0], [0, 0, 0, 1]])

R_x = np.array([[1,         0,             0,           0],
                 [0,  math.cos(roll),  -math.sin(roll), 0],
                 [0,  math.sin(roll),   math.cos(roll), 0],
                 [0,        0,             0,           1]],dtype=float)
R_y = np.array([[math.cos(pitch),    0,    math.sin(pitch),       0],
                [0,                  1,       0,           0],
                [-math.sin(pitch),   0,  math.cos(pitch),  0],
                [0,         0,             0,           1]],dtype=float)
R_z = np.array([[math.cos(yaw),  -math.sin(yaw),   0,    0],
                [math.sin(yaw),   math.cos(yaw),   0,    0],
                [0,                  0,            1,    0],
                [0,                  0,            0,    1]],dtype=float)

cap = cv2.VideoCapture(0)
assert cap.isOpened(), "capture open failed"
(cap.set(cv2.CAP_PROP_FPS, 20))
(cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920))
(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080))
(cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G')))
(cap.set(cv2.CAP_PROP_GAIN, 0.4))
(cap.set(cv2.CAP_PROP_BRIGHTNESS, 0))
(cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25))
(cap.set(cv2.CAP_PROP_EXPOSURE, 0.1))

EX_lidar2Cam1_1 = np.dot(R_y, R_z)
EX_lidar2Cam1_1 = np.dot(R_x, EX_lidar2Cam1_1)
EX_lidar2Cam1_1[0][3] = translation[0]
EX_lidar2Cam1_1[1][3] = translation[1]
EX_lidar2Cam1_1[2][3] = translation[2]

frame_shape = None
map1 = None
map2 = None

#EX_lidar2Cam1_1 = np.array([[0.9999974076187184,0.0005767913845450714,0.0022027409157229353,10.10833144121436],[0.002208062725396978,-0.009394489717382415,-0.9999534329267292, -167.66183875064434],[-0.0005560708981755888,0.9999557044562768,-0.009395738954839663,-36.95058955930789],[0,0,0,1]])

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

def project_lidar2img(undist_frame, allpoints):
    frame_shape = undist_frame.shape[1::-1]

    transformed_pcd = np.dot(EX_lidar2Cam1_1, allpoints.T)
    transformed_pcd = transformed_pcd.T

    index = np.where(transformed_pcd[:, 2] < 0)[0].tolist()
    transformed_pcd = np.delete(transformed_pcd, index, axis=0)
    for j in range(len(transformed_pcd)):
        point = transformed_pcd[j]
        center2img = np.dot(camera_mat_1, point) / point[2]
        projectpoint = list(map(int, center2img[:2]))
        #if projectpoint[0] >= 0 and projectpoint[0] <= frame_shape[0] and projectpoint[1] >= 0 and projectpoint[1] <= frame_shape[1]:
        projectpoint = tuple(projectpoint)
        cv2.circle(undist_frame, projectpoint, 1, (250, 0, 0), 1)  # (250,250,250)
    resized_frame = cv2.resize(undist_frame, (0, 0), fx=0.8, fy=0.8)
    cv2.imshow("project cloud to img", resized_frame)
    
    key = cv2.waitKey(1)

def callback(data):
    rospy.loginfo(rospy.get_caller_id())
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x","y","z"), skip_nans=True)
    gen = np.array(list(gen)) * 1000
    points = np.ones((gen.shape[0], 4), dtype=float)
    points[:,:3] = gen[:,:]
    
    global calib
    ok, raw_frame = cap.read()
    undist_frame = undistort(raw_frame)
    project_lidar2img(undist_frame, points)

def read():
    rospy.init_node('transform', anonymous=True)
    rospy.loginfo(rospy.get_caller_id())
    rospy.Subscriber('/os_cloud_node/points', PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    read()
