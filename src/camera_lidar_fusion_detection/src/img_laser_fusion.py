#! /usr/bin/env python

import roslib
import rospy
import cv2
import copy
import numpy as np
import argparse

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge


class ImgLaserFusion(object):

    def __init__(self, camera_intrincic_param, lidar_to_camera_extrincic_param,
                 img_topic="/undistort_lower/compressed", laser_topic="/wr_scan",
                 fusion_pub_topic="/undistort_lower/fusion_img",
                 debug=False):
        # ----------------------------------------------------------------------------
        # init the configuration.
        # ----------------------------------------------------------------------------

        self.camera_intrinic_param = camera_intrincic_param
        self.lidar_to_camera_extrincic_param = lidar_to_camera_extrincic_param

        self.img_topic = img_topic
        self.laser_topic = laser_topic
        self.fusion_pub_topic = fusion_pub_topic
        self.debug_mode = debug

        self.cv_bridge = CvBridge()

        self.img_subscriber = rospy.Subscriber(img_topic, CompressedImage,
                                        self.img_extraction_callback, queue_size=3)
        self.laser_subscriber = rospy.Subscriber(laser_topic, LaserScan,
                                        self.laser_extraction_callback, queue_size=3)

        self.fusion_publisher = rospy.Publisher(fusion_pub_topic, CompressedImage, queue_size=2)
        
        self.input_scan_np = None
        self.input_scan_np_last = None
        self.input_img = None
        self.scan_to_pixels = None
        self.fusion_img = None
        self.fusioning_flag = False

    def img_extraction_callback(self, compressed_img_msg):
        # ----------------------------------------------------------------------------
        # Extract the compressed input img to cv2 type imgs.
        # ----------------------------------------------------------------------------
        self.input_img = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_img_msg)

        if self.debug_mode:
            cv2.imshow("input_camera_img",self.input_img)
            cv2.waitKey(2) 

        self.publish_fusion_img()

    def laser_extraction_callback(self, laser_msg):
        # ----------------------------------------------------------------------------
        # Extract the LaserScan msg to x, y corrdinates msg
        # ----------------------------------------------------------------------------
        scan_rads = np.arange(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
        scan_ranges  = np.array(laser_msg.ranges)

        filter_flag = scan_ranges < np.inf
        scan_ranges = scan_ranges[filter_flag]
        scan_rads = scan_rads[filter_flag]

        scan_x = (scan_ranges * np.cos(scan_rads)).reshape(-1, 1)
        scan_y = (scan_ranges * np.sin(scan_rads)).reshape(-1, 1)

        # if not self.fusioning_flag:
        #     # Sometimes, the fusion process will be disturbed by the input laser_scan
        #     # In this case, the input_scan_np will be ignored.
        #     print("WARNNING!")
        #     self.input_scan_np = np.concatenate((scan_x, scan_y), axis=1)
        # else:
        #     pass

        self.input_scan_np = np.concatenate((scan_x, scan_y), axis=1)
        


    def fusion(self):

        self.fusioning_flag = True
        self.input_scan_np_last = self.input_scan_np
        if self.input_scan_np_last is not None and self.input_img is not None:
            # if len(self.input_scan_np) < 10:
            #     return
            zeros = np.zeros((len(self.input_scan_np_last), 1))
            ones = np.ones((len(self.input_scan_np_last), 1))
            try:
                input_points = np.concatenate((self.input_scan_np_last, zeros, ones), axis=1)
                transformed_points = np.dot(self.lidar_to_camera_extrincic_param, input_points.T).T
                # normalize all points
                # filter out the camera's back points
                geometric_filter_flag = transformed_points[:, 2] > 0

                transformed_points[:, 0] /= transformed_points[:, 2]
                transformed_points[:, 1] /= transformed_points[:, 2]
                transformed_points[:, 2] = 1
                 # project points to img pixel
                self.scan_to_pixels = np.dot(self.camera_intrinic_param, transformed_points.T).T.astype(np.int32)

                filter_flag_1 = np.logical_and(0 < self.scan_to_pixels[:, 0], self.scan_to_pixels[:, 0] < 800)
                filter_flag_2 = np.logical_and(0 < self.scan_to_pixels[:, 1], self.scan_to_pixels[:, 1] < 450)
                filter_flag = np.logical_and(filter_flag_1, filter_flag_2)
                filter_flag = np.logical_and(geometric_filter_flag, filter_flag)
                plot_pixels = self.scan_to_pixels[filter_flag, :]

                self.fusion_img = copy.copy(self.input_img)

                for point in plot_pixels:
                    cv2.circle(self.fusion_img, (point[0], point[1]), 3, (255, 0, 0), -1)

                    # cv2.imshow("img_fusion",self.input_img)
                    # cv2.waitKey(2)

            except ValueError:
                print self.input_scan_np_last.shape
                print zeros.shape
                print ones.shape

        self.fusioning_flag = False

    def publish_fusion_img(self):
        # get the fusion img
        self.fusion()
        if self.fusion_img is not None:
            # publish the fusion img
            if self.debug_mode:
                cv2.imshow("output_img_fusion",self.fusion_img)
                cv2.waitKey(2)

            self.fusion_img = self.cv_bridge.cv2_to_compressed_imgmsg(self.fusion_img,dst_format='jpeg')
            self.fusion_publisher.publish(self.fusion_img)

def main(opts):

    lower_camera_mat = np.array([[367.0543074903704 * 0.5, 0.0, 990.6330750325744 * 0.5], 
                             [0.0, 366.7370079611347 * 0.5, 575.1183044201284 * 0.5],
                             [0.0, 0.0, 1.0]])

    lidar_to_lower_camera = np.array([[0, -1, 0, 0],[0, 0, -1, -0.1], [1, 0, 0, -0.025]])
    
    rospy.init_node("image_extraction", anonymous=True)

    img_fusion = ImgLaserFusion(lower_camera_mat, lidar_to_lower_camera,
                    img_topic=opts.sub_img_topic, laser_topic=opts.sub_lidar_topic,
                    fusion_pub_topic=opts.pub_fusion_topic,
                    debug=opts.debug_mode)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS image extraction module."
    
if __name__ == "__main__":
    opts = argparse.ArgumentParser("This scirpt is used to fuse the camera img and the 2D-LiDAR scanning points.",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    opts.add_argument("--sub_img_topic", default="/undistort_lower/compressed", help="default: %(default)s")
    opts.add_argument("--sub_lidar_topic", default="/wr_scan", help="default: %(default)s")
    opts.add_argument("--pub_fusion_topic", default="/fusion_img/compressed", help="default: %(default)s")
    opts.add_argument("--debug_mode", action='store_true', help="This debug flag is used to visualize the input img and fusion img.\n")
    input_opts = opts.parse_args()
    main(input_opts)