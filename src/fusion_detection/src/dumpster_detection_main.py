#! /usr/bin/env python
import sys
import roslib
import rospy
import cv2
import copy
import argparse
import numpy as np

from cv_bridge import CvBridge
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Byte
from std_msgs.msg import ByteMultiArray
from deepstream_vision.msg import BBoxArray

import matplotlib.pyplot as plt
from camera_lidar_fusion_detection.msg import DumpsterInfo

from scipy.spatial.transform import Rotation as R

# import sys
# sys.path.append('/home/haitao/Project/tongji_sanitation_car/src/yolo_detect_pkg/src') 
# from dumpster_detect import ImageDumpsterDetector

class DumpsterDetection(object):

    # This class is used to detect the dumpster's location in the camera field with the help of 2D-LiDAR 
    def __init__(self, lower_camera_intrinic_param, lidar_to_camera_extrincic_param,
                 lower_image_topic="/undistort_lower/compressed",
                 laser_topic="/wr_scan",
                 lower_detect_topic='/yolo_detect_lower/bbox_results',
                 upper_detect_topic='/yolo_detect_upper/bbox_results',
                 output_topic="/dumpster_detection/dumpster_location", debug_mode=False):
        
        # parameters
        self.lower_camera_intrinic_param = lower_camera_intrinic_param 
        self.lidar_to_camera_extrincic_param = lidar_to_camera_extrincic_param
        self.lower_image_topic = lower_image_topic 
        self.laser_topic = laser_topic 
        self.output_topic = output_topic 
        self.debug_mode = debug_mode

        # inputs
        self.input_img_previous = None 
        self.input_img_current = None
        self.input_img_delta_time = None
        self.input_img_time_tick = None
        self.input_scan_current = None

        # states
        self.scan_img_coord = None
        # self.fusion_img = None 
        self.lower_detected_bboxes_list = None
        self.upper_detected_bboxes_list = None
        self.bbox_filter_flag_list = [] 

        self.geometric_filter_flag = None 
        self._ransac_min_sample = 0.3 
        self.dbscan_filter = DBSCAN(eps=0.1, min_samples=5) 

        self.opflow_based_static_flag = False

        
        frame_shape = (416, 416) # TUNABLE: hard-coded image size
        self.new_camera_mat = np.array([[200.0, 0.0, 416.0 * 0.5], [0.0, 200.0, 416.0 * 0.5], [0.0, 0.0, 1.0]])
        self.remap_map1, self.remap_map2 = cv2.initUndistortRectifyMap(
            self.lower_camera_intrinic_param, np.array([0.0, 0.0, 0.0, 0.0]), np.eye(3, 3), self.new_camera_mat, frame_shape, cv2.CV_16SC2)
        self.new_camera_mat = self.new_camera_mat.reshape(1, 3, 3)

        self.dumpster_detect_publisher = rospy.Publisher(self.output_topic, DumpsterInfo, queue_size=1)
        self.cv_bridge = CvBridge() 
        self.img_subscriber = rospy.Subscriber(lower_image_topic, CompressedImage,
                                                self.img_extraction_callback, queue_size=1)
        self.laser_subscriber = rospy.Subscriber(laser_topic, LaserScan,
                                                self.laser_extraction_callback, queue_size=1)
        self.lower_detect_subscriber =  rospy.Subscriber(lower_detect_topic, BBoxArray,
                                                self.lower_detect_callback, queue_size=1)

        # camera_mat = np.array([[367.0543074903704 * 0.5, 0.0, 990.6330750325744 * 0.5], [0.0, 366.7370079611347 * 0.5, 575.1183044201284 * 0.5], [0.0, 0.0, 1.0]])


    def img_extraction_callback(self, compressed_img_msg):

        # ----------------------------------------------------------------------------
        # Extract the compressed input img to cv2 type imgs.
        # Get the input img duration time 
        # Execute the opflow based speed estimation
        # ----------------------------------------------------------------------------
        self.input_img_previous = self.input_img_current
        uncompressed_img = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_img_msg)
        self.input_img_current = cv2.remap(uncompressed_img, self.remap_map1, self.remap_map2, cv2.INTER_LINEAR)
         
        if self.input_img_time_tick is not None:
            self.input_img_dtime = rospy.get_time() - self.input_img_time_tick
        self.input_img_time_tick = rospy.get_time()
        
        self.opflow_based_speed_estimation()


    def laser_extraction_callback(self, laser_msg):

        # ----------------------------------------------------------------------------
        # Extract the LaserScan msg to x, y corrdinates msg, as self.input_scan_np_cur of size [N, 4, 1]
        # Project the LaserScan points onto lower image.
        #
        # 2D LiDAR corrdinate definition
        #
        #          x  ^
        #             |   
        #             |
        #    y <-------
        # 
        # ----------------------------------------------------------------------------
        scan_rads = np.arange(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
        scan_ranges  = np.array(laser_msg.ranges)

        valid_filter_flag = scan_ranges < np.inf
        scan_ranges = scan_ranges[valid_filter_flag]
        scan_rads = scan_rads[valid_filter_flag]

        scan_x = (scan_ranges * np.cos(scan_rads)).reshape(-1, 1, 1)
        scan_y = (scan_ranges * np.sin(scan_rads)).reshape(-1, 1, 1)
        scan_z = np.zeros_like(scan_x)
        scan_h = np.ones_like(scan_x)

        self.input_scan_np_cur = np.concatenate((scan_x, scan_y, scan_z, scan_h), axis=1) # [-1, 4, 1]
        points_camera_coord = np.matmul(self.lidar_to_camera_extrincic_param, self.input_scan_np_cur) # [-1, 3, 1]
        points_camera_coord = points_camera_coord[points_camera_coord[:, 2] > 0.01] # [-1, 3, 1]
        points_camera_coord = points_camera_coord / points_camera_coord[:, 2:3] # [-1, 3, 1]
        self.scan_to_pixels = np.matmul(self.new_camera_mat, points_camera_coord)[:, :2, 0].astype(np.int32) # [-1, 2]
    
    def lower_detect_callback(self, lower_detect_message):
        # self.input_img_to_yolo = self.input_img_current
        # self.input_scan_np = self.input_scan_np_cur
        self.lower_detected_bboxes_list = lower_detect_message.bboxes

    def upper_detect_callback(self, upper_detect_message):
        # self.input_img_to_yolo = self.input_img_current
        # self.input_scan_np = self.input_scan_np_cur
        self.upper_detected_bboxes_list = upper_detect_message.bboxes

    # def laser_point_projection(self):
    #     # to address the input_laser_scan msg flush the msg
    #     # we use the input_scan to maintain the input_laser_scan points, 
    #     # only when we need to involke the detection function, we update the input_scan_np
        
    #     """In order to synchronize the input image and input scan points,
    #        the self.input_scan_np should be updated by the img_dumpster_detection function. 
    #        rather than in this function.
    #     """
    #     self.input_scan_np = self.input_scan_np_cur

    #     if self.input_scan_np is not None:
    #         zeros = np.zeros((len(self.input_scan_np), 1))
    #         ones = np.ones((len(self.input_scan_np), 1))
    #         # try:
    #         input_scan_points = np.concatenate((self.input_scan_np, zeros, ones), axis=1)
    #         transformed_points = np.dot(self.lidar_to_camera_extrincic_param, input_scan_points.T).T
    #         # filter out points that behind the camera
    #         self.geometric_filter_flag = transformed_points[:, 2] > 0
    #         # normalize input_points
    #         transformed_points[:, 0:2] /= transformed_points[:, 2:3]
    #         transformed_points[:, 2] = 1
    #         # project laser_scan points to image
    #         self.scan_to_pixels = np.dot(self.new_camera_mat, transformed_points.T).T.astype(np.int32)
    #         # except ValueError:
    #         #     print("[ERROR]", __line__()),
    #         #     print self.input_scan_np.shape
    #         #     print zeros.shape
    #         #     print ones.shape

    # def img_dumpster_detection(self):
    #     if self.img_dumpster_detect_func is not None:
    #         if self.input_img_current is None:
    #             print("[WARNING] no image received!")
    #             return

    #         self.input_img_to_yolo = self.input_img_current
    #         self.input_scan_np = self.input_scan_np_cur
            
    #         self.img_detected_bboxes_list = self.img_dumpster_detect_func(self.input_img_to_yolo)
    #         print("[INFO]", sys._getframe().f_lineno, self.img_detected_bboxes_list)
    #     else:
    #         self.img_detected_bboxes_list = [np.array([393, 113, 584, 471],dtype=np.int32),
    #                                         np.array([393, 113, 584, 471],dtype=np.int32)]
    #         self.bbox_np = np.array([393, 113, 584, 471],dtype=np.int32)
    #         self.bbox_np = np.concatenate((self.bbox_np, self.bbox_np))

    def opflow_based_speed_estimation(self):

        step = 10
        #Intercept the roadside part of frame
        if self.input_img_previous is None or self.input_img_dtime is None:
            return

        # ROI selection
        estimation_size = 100 # TUNABLE: bottom region height for calculating speed
        img_h, img_w, _ = self.input_img_current.shape        
        img_h_start = img_h - estimation_size
        img_h_end = img_h_start + estimation_size
        img_pre = self.input_img_previous[img_h_start:img_h_end, :, :]
        img_pre_gray = cv2.cvtColor(img_pre, cv2.COLOR_BGR2GRAY)
        img_cur = self.input_img_current[img_h_start:img_h_end, :, :]
        img_cur_gray = cv2.cvtColor(img_cur, cv2.COLOR_BGR2GRAY)
        
        # optical calculation in ROI
        op_flow = cv2.calcOpticalFlowFarneback(img_pre_gray, img_cur_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        h, w = img_cur_gray.shape[:2]
        y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)
        fx, fy = op_flow[y, x].T
        self.opflow_based_speed = abs(sum(fx)/np.size(fx)) / self.input_img_dtime
        
        self.opflow_based_static_flag = True if self.opflow_based_speed < 5 else False  # TUNABLE: threshold of speed
        
        # if self.debug_mode:
        #     lines = np.vstack([x, img_h_start + y, x + fx, img_h_start + y + fy]).T.reshape(-1, 2, 2)
        #     lines = np.int32(lines + 0.5)
        #     opflow_img = copy.copy(self.input_img_current)
        #     cv2.polylines(opflow_img, lines, 0, (0, 255, 0))
        #     cv2.waitKey(2)
        #     cv2.imshow("opflow_estimation", opflow_img)


    def dumpster_ransac_fit(self, laser_points):

        # return ransac_fited points
        ransac = RANSACRegressor(min_samples=self._ransac_min_sample)
        points_X = laser_points[:, 1].reshape(-1, 1)
        points_y = laser_points[:, 0]
        #print(points_X, points_y)
        ransac.fit(points_X, points_y)
        
        # normalized errs.
        fit_err = np.sum(abs(ransac.predict(points_X) - points_y)) / len(points_y)

        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        inlier_points_X = sorted(points_X[inlier_mask], reverse=True)

        points_x_start = inlier_points_X[0]
        points_y_start = ransac.predict([points_x_start])
        points_x_end = inlier_points_X[-1]
        points_y_end = ransac.predict([points_x_end])

        # This return results are the fitting two end points
        # return fit_err, np.array([[points_y_start, points_x_start], [points_y_end, points_x_end]]).squeeze()
        
        # This return results are the dumpster location info: lateral_dis, side_offset, object_width 
        lateral_dis = np.min([points_y_start, points_y_end])
        side_offset = np.mean([points_x_end, points_x_start])
        object_width = abs(points_x_start[0] - points_x_end[0])

        return fit_err, np.array([[points_y_start, points_x_start], [points_y_end, points_x_end]]).squeeze(),\
                        lateral_dis, side_offset, object_width


    def dumpster_linear_fit(self, laser_points):

        # return linear_fited points:
        linear_fit_func = LinearRegression()
        points_X = laser_points[:, 1].reshape(-1, 1)
        points_y = laser_points[:, 0]
        linear_fit_func.fit(points_X, points_y)

        # normalized fit errs.
        fit_err = np.sum(abs(linear_fit_func.predict(points_X) - points_y)) / len(points_X)
        
        points_X = sorted(points_X, reverse=True)
        points_x_start = points_X[0]
        points_y_start = linear_fit_func.predict([points_x_start])
        points_x_end = points_X[-1]
        points_y_end = linear_fit_func.predict([points_x_end])
        
        # This return results are the fitting two end points
        # return fit_err, np.array([[points_y_start, points_x_start], [points_y_end, points_x_end]]).squeeze()
        
        # This return results are the dumpster location info: lateral_dis, side_offset, object_width 
        lateral_dis = np.min([points_y_start, points_y_end])
        side_offset = np.mean([points_x_end, points_x_start])
        object_width = abs(points_x_start[0] - points_x_end[0])

        return fit_err, np.array([[points_y_start, points_x_start], [points_y_end, points_x_end]]).squeeze(), lateral_dis, side_offset, object_width

    def lshape_fitting(self, laser_points):
        
        # L-Shape fitting evaluation function
        def fitting_func(theta, points_X, points_y):
            
            c_1 = points_y * np.sin(theta) + points_X * np.cos(theta) 
            c_2 = points_y * np.cos(theta) - points_X * np.sin(theta) 
        
            c_1_min, c_1_max = np.min(c_1), np.max(c_1) 
            c_2_min, c_2_max = np.min(c_2), np.max(c_2) 
        
            d_to_c1_min = c_1 - c_1_min 
            d_to_c1_max = c_1_max - c_1 
        
            d_to_c2_min = c_2 - c_2_min 
            d_to_c2_max = c_2_max - c_2 

            score_d_to_c1_min = 1 / (1 + d_to_c1_min) 
            score_d_to_c1_max = 1 / (1 + d_to_c1_max) 

            score_d_to_c2_min = 1 / (1 + d_to_c2_min) 
            score_d_to_c2_max = 1 / (1 + d_to_c2_max) 

            score_c1 = score_d_to_c1_max if np.sum(score_d_to_c1_max) > np.sum(score_d_to_c1_min) else score_d_to_c1_min 
            score_c2 = score_d_to_c2_max if np.sum(score_d_to_c2_max) > np.sum(score_d_to_c2_min) else score_d_to_c2_min 

            compare_flag_1 = score_c1 > score_c2
            compare_flag_2 = np.logical_not(compare_flag_1)

            score = np.sum(score_c1[compare_flag_1]) + np.sum(score_c2[compare_flag_2])
            
            return score, c_1_min, c_1_max, c_2_min, c_2_max

        max_score = 0
        best_theta = 0
        points_X = laser_points[:, 1].reshape(-1, 1)
        points_y = laser_points[:, 0]

        for theta in np.arange(0, np.pi/2, np.pi/30):
            score, _, _, _, _ = fitting_func(theta, points_X, points_y)
            if score > max_score:
                max_score = score
                best_theta = theta
        
        _, c_1_min, c_1_max, c_2_min, c_2_max = fitting_func(best_theta, points_X, points_y)

        theta_param = np.array([[np.sin(best_theta), np.cos(best_theta)], [np.cos(best_theta), -np.sin(best_theta)]])
        c_param = np.array([[c_1_min, c_1_min, c_1_max, c_1_max],[c_2_min, c_2_max, c_2_min, c_2_max]])
        # cross_points = [y1, x1].T * 4
        cross_points = np.dot(theta_param, c_param) 
        left_points = cross_points[:, np.argmax(cross_points[1, :])].reshape(-1, 1)
        right_points = cross_points[:, np.argmin(cross_points[1, :])].reshape(-1, 1)
        near_points = cross_points[:, np.argmin(cross_points[0, :])].reshape(-1, 1)

        lateral_dis = near_points[0, 0]
        side_offset = np.mean([left_points[1, 0], right_points[1, 0]])
        object_width = abs(left_points[1, 0] - right_points[1, 0])

        if self.debug_mode:
            np.save("./tmp_point.npy", laser_points)
        
        return np.concatenate((left_points.T, right_points.T), axis=0), lateral_dis, side_offset, object_width

    def visualize_detection_result(self, detect_loc, scanning_points, scanning_points_filter, input_img):
        
        if scanning_points is not None and scanning_points_filter is not None:
            scanning_points = scanning_points[scanning_points_filter]
            input_points = np.concatenate((detect_loc, scanning_points), axis=0)

            zeros = np.zeros((len(input_points), 1))
            ones = np.ones((len(input_points), 1))
            input_points = np.concatenate((input_points, zeros, ones), axis=1)
            transformed_points = np.dot(self.lidar_to_camera_extrincic_param, input_points.T).T
            transformed_points[:, 0] /= transformed_points[:, 2]
            transformed_points[:, 1] /= transformed_points[:, 2]
            transformed_points[:, 2] = 1
            pixels = np.dot(self.new_camera_mat, transformed_points.T).T.astype(np.int32)
            
            # visualize the detected bboxes
            print("[INFO]", sys._getframe().f_lineno, self.lower_detected_bboxes_list)
            for bbox in self.lower_detected_bboxes_list:
                if bbox.class_id == 5:
                    cv2.rectangle(input_img, (bbox.x_top_left, bbox.y_top_left),
                                            (bbox.x_bottom_right, bbox.y_bottom_right),
                                            (0, 255, 0), 2)
            # visualize the detected keypoints
            for pixel in pixels[0:len(detect_loc),:]:
                cv2.circle(input_img, (pixel[0], pixel[1]), 6, (0, 0, 255), -1)
            # visualize the input laser points
            for pixel in pixels[len(detect_loc):, :]:
                cv2.circle(input_img, (pixel[0], pixel[1]), 3, (255, 0, 0), -1)

        if input_img is not None:
            cv2.imshow("dumpster_detection_result", input_img)
            cv2.waitKey(2)


    def laser_dumpster_detection(self):

        # self.laser_point_projection()
        #self.input_scan_np = self.input_scan_np_cur
        self.bbox_filter_flag_list = [] # filter for lidar scan to pixel points, each corresponding to one bbox

        if self.lower_detected_bboxes_list is not None:

            detected_keypoints_list = []
            detected_dumpster_loc_list = []

            for bbox in self.lower_detected_bboxes_list:

                if bbox.score < 0.95: continue # TUNABLE: yolo lower confidence score

                if self.scan_to_pixels is not None:
                    bbox_x_inside_flag = np.logical_and(self.scan_to_pixels[:, 0] >= bbox.x_top_left,
                                            self.scan_to_pixels[:, 0] <= bbox_list.x_bottom_right)
                    bbox_y_inside_flag = np.logical_and(self.scan_to_pixels[:, 1] >= bbox_list.y_top_left,
                                            self.scan_to_pixels[:, 1] <= bbox_list.y_bottom_right)
                    
                    bbox_inside_flag = np.logical_and(bbox_x_inside_flag, bbox_y_inside_flag)

                    self.bbox_filter_flag_list.append(np.logical_and(self.geometric_filter_flag, bbox_inside_flag))

            for bbox_filter_flag in self.bbox_filter_flag_list:

                # use DBSCAN clustering algorithm to filter out outlier points.
                bbox_points = self.input_scan_np[bbox_filter_flag,:]
                #print(bbox_points)
                if bbox_points.shape[0] == 0: continue
                filter_label = self.dbscan_filter.fit_predict(bbox_points)
                if (filter_label == -1).all():
                    print("[INFO]", sys._getframe().f_lineno, "all candidates are masked by dbscan!")
                    continue
                inlier_num = 0
                inlier_label = 0
                for i in range(np.max(filter_label)):
                    #label_sum = np.sum(filter_label[filter_label==i]) # TODO to delete
                    label_sum = np.sum(filter_label==i)
                    if label_sum > inlier_num:
                        inlier_num = label_sum
                        inlier_label = i
                inlier_points = bbox_points[filter_label==inlier_label, :]
                #print(inlier_points)
                fit_err, detect_keypoints, lateral_dis, side_offset, object_width\
                                    = self.dumpster_ransac_fit(inlier_points)
                # print(fit_err)
                # TODO:
                # The threshold should adjust

                # print("ransac")
                # print(lateral_dis, side_offset, object_width)
                if fit_err > 0.5:
                    detect_keypoints, lateral_dis, side_offset, object_width\
                                     = self.lshape_fitting(inlier_points)
                    # print("l-shape")
                    # print(lateral_dis, side_offset, object_width)

                detected_keypoints_list.append(detect_keypoints)
                detected_dumpster_loc_list.append((lateral_dis, side_offset, object_width))

            #if self.debug_mode:
            if True:
                # self.input_img_to_yolo = self.input_img_current
                detected_keypoints_list_np = np.array(detected_keypoints_list).reshape(-1, 2)
                self.visualize_detection_result(detected_keypoints_list_np, self.input_scan_np, self.geometric_filter_flag, self.input_img_current)

            if len(detected_dumpster_loc_list) > 1:

                continue_grab = 1
                center_dumpster = sorted(detected_dumpster_loc_list, key=lambda obj: abs(obj[1]))[0]
                left_dumpster = sorted(detected_dumpster_loc_list, key=lambda obj: -obj[1])[0]
                right_dumpster = sorted(detected_dumpster_loc_list, key=lambda obj: obj[1])[0]
                
                lateral_dis, side_offset, object_width = center_dumpster

                left_gap = (left_dumpster[1] - left_dumpster[2] / 2) - (side_offset + object_width / 2)
                right_gap = - (right_dumpster[1] + right_dumpster[2] / 2) + (side_offset - object_width / 2)
                
                if self.debug_mode:
                    print("Later Dis: {:.2f} \t Side Offset: {:.2f} \t Object Width: {:.2f}.".format(lateral_dis, side_offset, object_width))
                    print("Left Gap:{:.2f} \t Right Gap:{:.2f}".format(left_gap, right_gap))
                left_gap = left_gap * 1000 / 4
                right_gap = right_gap * 1000 / 4
                lateral_dis = lateral_dis * 1000 / 4
                
                left_gap = np.clip(left_gap, a_min=0, a_max=255)
                right_gap = np.clip(right_gap, a_min=0, a_max=255)
                lateral_dis = np.clip(lateral_dis, a_min=0, a_max=255)

                # byte_0 = np.uint8(2).tobytes()
                continuous_dumpster = 2
                
            else:
                continue_grab = 0
                if not detected_dumpster_loc_list:
                    #print('No detected dumpster loc list!')
                    return
                lateral_dis, side_offset, object_width = detected_dumpster_loc_list[0]
                if self.debug_mode:
                    print("Later Dis: {:.2f} \t Side Offset: {:.2f} \t Object Width: {:.2f}.".format(lateral_dis, side_offset, object_width))
                    print("Left Gap:{} \t Right Gap:{}".format(None, None))
                left_gap = 255 
                right_gap = 255
                lateral_dis = lateral_dis * 1000 / 4
                lateral_dis = np.clip(lateral_dis, a_min=0, a_max=255)     

                # byte_0 = np.uint8(0).tobytes()
                continuous_dumpster = 0
            
            # byte_1 = np.uint8(lateral_dis).tobytes()
            # byte_23 = np.float16(side_offset).tobytes()
            # byte_45 = np.float16(object_width).tobytes()
            # byte_6 = np.uint8(left_gap).tobytes()
            # byte_7 = np.uint8(right_gap).tobytes()

            # return [byte_0, byte_1, byte_23[0], byte_23[1] , byte_45[0], byte_45[1], byte_6, byte_7]
            return [continuous_dumpster, lateral_dis, side_offset, object_width, left_gap, right_gap]
        else:
            return None    
            

    def run(self):
        publish_rate = rospy.Rate(5)
        # publish speed info here
        opflow_based_speed_estimation_pub = rospy.Publisher("opflow_static", Bool, queue_size=1)
        # ToDo: msg configuration 
        # detection_msg = Frame(data="tongji")
        self.opflow_based_speed = None

        while not rospy.is_shutdown():
            
            # opflow_based_static info publish 
            opflow_based_speed_estimation_pub.publish(self.opflow_based_static_flag)

            # if self.opflow_based_static_flag:
            if True:
                # 2D img based dumpster bboxes detection
                #self.img_dumpster_detection()
                # img_laser fusion dumpster detection
                dumpster_info = self.laser_dumpster_detection()

                if dumpster_info is None:
                    print("There is no dumpster!")
                else:
                    if self.debug_mode:
                        print("The Hex representation of 8 bytes info\n:{}".format(dumpster_info))
                    # dumpster_msg = DumpsterCAN()
                    # dumpster_msg.header.stamp = rospy.Time.now()
                    # dumpster_msg.byte_string = ''.join(dumpster_info)
                    dumpster_msg = DumpsterInfo()
                    dumpster_msg.header.stamp = rospy.Time.now()
                    dumpster_msg.continuous_dumpster = dumpster_info[0]
                    dumpster_msg.lateral_dis = dumpster_info[1]
                    dumpster_msg.side_offset = dumpster_info[2]
                    dumpster_msg.object_width = dumpster_info[3]
                    dumpster_msg.left_gap = dumpster_info[4]
                    dumpster_msg.right_gap = dumpster_info[5]
                    # TODO:
                    # current the data convertion has problem, need to fix!!!
                    # 
                    # dumpster_info = map(lambda x:np.uint8(ord(x)), dumpster_info)
                    # dumpster_msg.byte_0 = dumpster_info[0]
                    # dumpster_msg.byte_1 = dumpster_info[1]
                    # dumpster_msg.byte_2 = dumpster_info[2]
                    # dumpster_msg.byte_3 = dumpster_info[3]
                    # dumpster_msg.byte_4 = dumpster_info[4]
                    # dumpster_msg.byte_5 = dumpster_info[5]
                    # dumpster_msg.byte_6 = dumpster_info[6]
                    # dumpster_msg.byte_7 = dumpster_info[7]
                    
                    self.dumpster_detect_publisher.publish(dumpster_msg)

            publish_rate.sleep()
                
def main(opts):
    # init ros node 
    rospy.init_node("dumpster_detection", anonymous=True)
    # param config
    lower_camera_mat = np.array([[367.0543074903704 * 0.5, 0.0, 990.6330750325744 * 0.5], 
                            [0.0, 366.7370079611347 * 0.5, 575.1183044201284 * 0.5],
                            [0.0, 0.0, 1.0]])
    # lidar_to_lower_camera =  np.array([[0, -1, 0, 0],[0, 0, -1, -0.1], [1, 0, 0, -0.025]])
    # angle more 5
    # lidar_to_lower_camera =  np.array([[-8.71557427e-02, -9.96194698e-01, -6.63360825e-17, 0],
    #                                     [6.12323400e-17,  6.12323400e-17, -1.00000000e+00, -0.1],
    #                                     [9.96194698e-01, -8.71557427e-02, 5.56625823e-17, -0.025]])
    r = R.from_euler('zyz', [4.5, -90, 90], degrees=True)
    t = [0, -0.1, -0.025]
    lidar_to_camera_extrincic_param = np.concatenate((r.as_dcm(), np.array(t).reshape((3, 1))), axis=1).reshape(1, 3, 4)
    # print(lidar_to_camera_extrincic_param)
    # exit()
    # init detector
    dumpster_detector = DumpsterDetection(lower_camera_mat, lidar_to_camera_extrincic_param,
                                        lower_image_topic=opts.lower_image_topic,
                                        laser_topic=opts.lidar_sub_topic,
                                        lower_detect_topic=opts.lower_detect_topic,
                                        upper_detect_topic=opts.upper_detect_topic,
                                        output_topic=opts.pub_dumpster_detect_topic,
                                        debug_mode=opts.debug_mode)
    # if dumpster_detector.input_img_current:
    dumpster_detector.run()
    
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print "Shutting down ROS image extraction module."

if __name__ == "__main__":

    opts = argparse.ArgumentParser("This scirpt is used to fuse the camera img and the 2D-LiDAR scanning points.",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    opts.add_argument("--lower_image_topic", default="/undistort_lower/compressed", help="default: %(default)s")
    opts.add_argument("--lidar_sub_topic", default="/wr_scan", help="default: %(default)s")
    opts.add_argument("--lower_detect_topic", default="/yolo_detect_lower/bbox_results", help="default:%(default)s")
    opts.add_argument("--upper_detect_topic", default=None, help="default:%(default)s")
    opts.add_argument("--pub_dumpster_detect_topic", default="/dumpster_detection/dumpster_location", help="default: %(default)s")
    opts.add_argument("--debug_mode", action='store_true', help="This debug flag is used to visualize the detection results.\n")
    input_opts = opts.parse_args()
   
    main(input_opts)
