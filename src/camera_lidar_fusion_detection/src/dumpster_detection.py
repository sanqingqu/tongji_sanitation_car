#! /usr/bin/env python

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

from yolo_detect_pkg.msg import BBoxArray
from yolo_detect_pkg.msg import BBox
from camera_lidar_fusion_detection.msg import DumpsterLoc
from camera_lidar_fusion_detection.msg import DumpsterLocArray


class DumpsterDetection(object):
    # This class is used to detect the dumpster's location in the camera field with the help of 2D-LiDAR 
    def __init__(self, camera_intrinic_param, lidar_to_camera_extrincic_param,
                 img_topic="/undistort_lower/compressed", laser_topic="/wr_scan",
                 yolo_detect_topic="/yolo_detect/bbox_results",
                 output_topic="/dumpster_detection/dumpster_location", debug=False):
        
        self.camera_intrinic_param = camera_intrinic_param
        self.lidar_to_camera_extrincic_param = lidar_to_camera_extrincic_param
        self.yolo_detect_topic = yolo_detect_topic

        self.img_topic = img_topic
        self.laser_topic = laser_topic
        self.output_topic = output_topic
        self.debug_mode = debug

        self.input_scan_np = None
        self.input_img = None
        self.scan_to_pixels = None
        # self.fusion_img = None
        self.detect_bboxes = None

        self.bbox_filter_flag_list = []
        self.geometric_filter_flag = None
        self._ransac_min_sample = 0.3
        self.dbscan_filter = DBSCAN(eps=0.2, min_samples=5)

        self.cv_bridge = CvBridge()

        self.img_subscriber = rospy.Subscriber(img_topic, CompressedImage,
                                        self.img_extraction_callback, queue_size=3)

        self.laser_subscriber = rospy.Subscriber(laser_topic, LaserScan,
                                        self.laser_extraction_callback, queue_size=3)

        self.yolo_detect_subscriber = rospy.Subscriber(yolo_detect_topic, BBoxArray,
                                        self.yolo_detect_callback, queue_size=3)

        self.dumpster_detect_publisher = rospy.Publisher(self.output_topic, DumpsterLocArray, queue_size=2)


    def img_extraction_callback(self, compressed_img_msg):
        # ----------------------------------------------------------------------------
        # Extract the compressed input img to cv2 type imgs.
        # ----------------------------------------------------------------------------
        self.input_img = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_img_msg)


    def yolo_detect_callback(self, yolo_bboxes_msg):
        # ----------------------------------------------------------------------------
        # Extract the image-based dumpster detection network detected bboxes.
        # ----------------------------------------------------------------------------
        self.detect_bboxes = yolo_bboxes_msg.bboxes


    def laser_extraction_callback(self, laser_msg):
        # ----------------------------------------------------------------------------
        # Extract the LaserScan msg to x, y corrdinates msg
        #
        # Execute the Dumpster Detection function.
        # ----------------------------------------------------------------------------
        scan_rads = np.arange(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
        scan_ranges  = np.array(laser_msg.ranges)

        filter_flag = scan_ranges < np.inf
        scan_ranges = scan_ranges[filter_flag]
        scan_rads = scan_rads[filter_flag]

        scan_x = (scan_ranges * np.cos(scan_rads)).reshape(-1, 1)
        scan_y = (scan_ranges * np.sin(scan_rads)).reshape(-1, 1)

        self.input_scan_np = np.concatenate((scan_x, scan_y), axis=1)

        self.laser_dumpster_detection()


    def laser_dumpster_detection(self):
        self.laser_point_projection()
        self.bbox_filter_flag_list = []
        if self.detect_bboxes is not None:

            detect_loc_list = []
            dumpsterloc_msg = DumpsterLocArray()
            dumpsterloc_msg.header.stamp = rospy.Time.now()

            for bbox in self.detect_bboxes:
                if self.scan_to_pixels is not None:
                    x_flag = np.logical_and(self.scan_to_pixels[:, 0] >= bbox.x_top_left,
                                            self.scan_to_pixels[:, 0] <= bbox.x_bottom_right)
                    y_flag = np.logical_and(self.scan_to_pixels[:, 1] >= bbox.y_top_left,
                                            self.scan_to_pixels[:, 1] <= bbox.y_bottom_right)
                    
                    filter_flag = np.logical_and(x_flag, y_flag)

                    self.bbox_filter_flag_list.append(np.logical_and(self.geometric_filter_flag, filter_flag))

            for bbox_filter_flag in self.bbox_filter_flag_list:
                bbox_points = self.input_scan_np[bbox_filter_flag,:]
                filter_label = self.dbscan_filter.fit_predict(bbox_points)
                inlier_num = 0
                inlier_label = 0
                for i in range(np.max(filter_label)):
                    label_sum = np.sum(filter_label[filter_label==i])
                    if label_sum > inlier_num:
                        inlier_num = label_sum
                        inlier_label = i
                inlier_points = bbox_points[filter_label==inlier_label, :]

                detect_loc = self.dumpster_ransac_fit(inlier_points)
                detect_loc_list.append(detect_loc)
            
            for detect_loc in detect_loc_list:

                dumpsterloc = DumpsterLoc()
                dumpsterloc.x_left_loc = detect_loc[0, 0]
                dumpsterloc.y_left_loc = detect_loc[0, 1]
                dumpsterloc.x_right_loc = detect_loc[1, 0]
                dumpsterloc.y_right_loc = detect_loc[1, 1]

                dumpsterloc_msg.dumpsters_list.append(dumpsterloc)

            detect_loc_list_np = np.array(detect_loc_list).reshape(-1, 2)

            if self.debug_mode:
                self.visualize_detection_result(detect_loc_list_np, self.input_scan_np[self.geometric_filter_flag,:], self.input_img)
            
            self.dumpster_detect_publisher.publish(dumpsterloc_msg)
            

    def laser_point_projection(self):
        if self.input_scan_np is not None:
            zeros = np.zeros((len(self.input_scan_np), 1))
            ones = np.ones((len(self.input_scan_np), 1))
            try:
                input_scan_points = np.concatenate((self.input_scan_np, zeros, ones), axis=1)
                transformed_points = np.dot(self.lidar_to_camera_extrincic_param, input_scan_points.T).T
                # filter out points that behind the camera
                self.geometric_filter_flag = transformed_points[:, 2] > 0
                # normalize input_points
                transformed_points[:, 0] /= transformed_points[:, 2]
                transformed_points[:, 1] /= transformed_points[:, 2]
                transformed_points[:, 2] = 1
                # project laser_scan points to image
                self.scan_to_pixels = np.dot(self.camera_intrinic_param, transformed_points.T).T.astype(np.int32)
            
            except ValueError:
                print self.input_scan_np.shape
                print zeros.shape
                print ones.shape


    def visualize_detection_result(self, detect_loc, scanning_points, input_img):
        
        input_points = np.concatenate((detect_loc, scanning_points), axis=0)
        
        zeros = np.zeros((len(input_points), 1))
        ones = np.ones((len(input_points), 1))
        input_points = np.concatenate((input_points, zeros, ones), axis=1)
        transformed_points = np.dot(self.lidar_to_camera_extrincic_param, input_points.T).T
        transformed_points[:, 0] /= transformed_points[:, 2]
        transformed_points[:, 1] /= transformed_points[:, 2]
        transformed_points[:, 2] = 1
        pixels = np.dot(self.camera_intrinic_param, transformed_points.T).T.astype(np.int32)
        
        # visualize the detected bboxes
        for bbox in self.detect_bboxes:
            cv2.rectangle(self.input_img, (bbox.x_top_left, bbox.y_top_left),
                                    (bbox.x_bottom_right, bbox.y_bottom_right),
                                    (0, 255, 0), 2)
        # visualize the detected keypoints
        for pixel in pixels[0:len(detect_loc),:]:
            cv2.circle(input_img, (pixel[0], pixel[1]), 6, (0, 0, 255), -1)
        # visualize the input laser points
        for pixel in pixels[len(detect_loc):, :]:
            cv2.circle(input_img, (pixel[0], pixel[1]), 3, (255, 0, 0), -1)

        cv2.imshow("dumpster_detection_result",self.input_img)
        cv2.waitKey(2)


    def dumpster_ransac_fit(self, laser_points):
        # return ransac_fited points
        ransac = RANSACRegressor(min_samples=self._ransac_min_sample)
        points_X = laser_points[:, 1].reshape(-1, 1)
        points_y = laser_points[:, 0]
        ransac.fit(points_X, points_y)
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        inlier_points_X = sorted(points_X[inlier_mask])
        inlier_pred = ransac.predict(inlier_points_X)
        inlier_start = ransac.predict([inlier_points_X[0]])
        inlier_end = ransac.predict([inlier_points_X[-1]])

        return np.array([[inlier_start, inlier_points_X[0]],[inlier_end, inlier_points_X[-1]]]).squeeze()


    def dumpster_linear_fit(self, laser_points):
        # return linear_fited points:
        linear_fit_func = LinearRegression()
        points_X = laser_points[:, 1].reshape(-1, 1)
        points_y = laser_points[:, 0]
        linear_fit_func.fit(points_X, points_y)
        points_X = sorted(points_X)
        points_x_start = points_X[0]
        points_y_start = linear_fit_func.predict([points_x_start])
        points_x_end = points_X[-1]
        points_y_end = linear_fit_func.predict([points_x_end])
        
        return np.array([[points_y_start, points_x_start], [points_y_end, points_x_end]]).squeeze()


def main(opts):
    lower_camera_mat = np.array([[367.0543074903704 * 0.5, 0.0, 990.6330750325744 * 0.5], 
                            [0.0, 366.7370079611347 * 0.5, 575.1183044201284 * 0.5],
                            [0.0, 0.0, 1.0]])

    lidar_to_lower_camera =  np.array([[0, -1, 0, 0],[0, 0, -1, -0.1], [1, 0, 0, -0.025]])
    
    rospy.init_node("dumpster_detection", anonymous=True)

    dumpter_detection = DumpsterDetection(lower_camera_mat, lidar_to_lower_camera,
                                        img_topic=opts.sub_img_topic,laser_topic=opts.sub_lidar_topic,
                                        yolo_detect_topic=opts.sub_yolo_detect_topic,
                                        output_topic=opts.pub_dumpster_detect_topic,
                                        debug=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS image extraction module."

if __name__ == "__main__":
    opts = argparse.ArgumentParser("This scirpt is used to fuse the camera img and the 2D-LiDAR scanning points.",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    opts.add_argument("--sub_img_topic", default="/undistort_lower/compressed", help="default: %(default)s")
    opts.add_argument("--sub_lidar_topic", default="/wr_scan", help="default: %(default)s")
    opts.add_argument("--sub_yolo_detect_topic", default="/yolo_detect/bbox_results", help="default: %(default)s")
    opts.add_argument("--pub_dumpster_detect_topic", default="/dumpster_detection/dumpster_location", help="default: %(default)s")
    opts.add_argument("--debug_mode", action='store_true', help="This debug flag is used to visualize the detection results.\n")
    input_opts = opts.parse_args()
   
    main(input_opts)
