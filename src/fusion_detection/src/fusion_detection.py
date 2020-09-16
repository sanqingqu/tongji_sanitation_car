#!/usr/bin/env python
#pylint: disable=no-member
import configargparse
import cv2
import cv_bridge
import curses
import easydict
import numpy as np
import pprint
import rospy
import speed
import sys
import utils
import visualizer
import yaml

from argparse import Namespace
from deepstream_vision.msg import BBoxArray
from sensor_msgs.msg import LaserScan, CompressedImage
from time import time


class FusionDumpsterDetectorNode:

    def __init__(self, opts, node_name="dumpster_detection"):
        self.opts = opts
        self.cv_bridge = cv_bridge.CvBridge()
        self.extract_img = self.cv_bridge.compressed_imgmsg_to_cv2
        self.lower_reverter = utils.BBoxReverter(
            self.opts.lower_img_intrinsics,
            self.opts.lower_img_detect_intrinsics)
        self.speed = speed.SpeedEstimation()
        self.visualizer = visualizer.Visualizer(full_screen=self.opts.fullscreen)
        self.msgbuf = easydict.EasyDict({
            'wr_lidar' : utils.MsgBuffer(size=1),
            'lower_img' : utils.MsgBuffer(size=10),
            'lower_box' : utils.MsgBuffer(size=10),
        })
        rospy.init_node(node_name, anonymous=True)
        self.timer = utils.Timer()
        rospy.Subscriber(self.opts.wr_lidar_sub_topic, LaserScan, self.msgbuf_callback_factory('wr_lidar', self.extract_wr_lidar), queue_size=2)
        rospy.Subscriber(self.opts.lower_img_sub_topic, CompressedImage, self.msgbuf_callback_factory('lower_img', self.extract_img), queue_size=1)
        rospy.Subscriber(self.opts.lower_box_sub_topic, BBoxArray, self.msgbuf_callback_factory('lower_box', self.extract_bbox_array), queue_size=10)
        self.am = utils.AverageMeter()

    def msgbuf_callback_factory(self, msg_key, extract):
        def callback(msg):
            msg.header.stamp = rospy.get_rostime()
            self.msgbuf[msg_key].append(Namespace(header=msg.header, data=extract(msg)))
        return callback
        
    def extract_wr_lidar(self, laser_msg):
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

        points_lidar_coord  = np.concatenate((scan_x, scan_y, scan_z), axis=1) # [-1, 3, 1]
        points_camera_coord = np.matmul(self.opts.lidar_to_lower_extrinsics.R, points_lidar_coord) + \
                self.opts.lidar_to_lower_extrinsics.t # [-1, 3, 1]
        front_points_filter = np.logical_and(points_camera_coord[:, 2, 0] > 0.01,
                points_camera_coord[:, 2, 0] < np.inf) # filter the points behind camera
        points_camera_coord = points_camera_coord[front_points_filter] # [-1, 3, 1]
        points_camera_coord_homo = points_camera_coord / points_camera_coord[:, 2:3, :] # [-1, 3, 1]
        points_image_coord = np.matmul(self.opts.lower_img_intrinsics, points_camera_coord_homo)[:, :2, 0].astype(np.int32) # [-1, 2]

        return {"lidar":points_lidar_coord[front_points_filter],
                "camera":points_camera_coord, "image":points_image_coord}
    
    def extract_bbox_array(self, bbox_array_msg):
        return None

    def fusion_detect(self, vehicle_speed, lower_bboxes, point_cloud, lower_img_raw):
        result = easydict.EasyDict()
        result.dynamic_detected = vehicle_speed > self.opts.speed_threshold
        self.timer.count()
        return result
    
    def scr_monitor_msgbuf(self, scr):
        scr.addstr(2, 4, "=== Messages ===")
        for i, (k, buf) in enumerate(self.msgbuf.items()):
            scr.addstr(i+3, 4, "[%s:\t%03d]" % (k, len(buf)))

    def scr_monitor_timer(self, scr):
        if self.timer.msec() is None: return
        scr.addstr(2, 30, "=== Timer ===")
        scr.addstr(3, 30, "[msec = %.2f]" % self.timer.msec())
        scr.addstr(4, 30, "[fps  = %.2f]" % self.timer.fps())
    
    def spin_once(self, scr):
        self.scr_monitor_msgbuf(scr)
        self.scr_monitor_timer(scr)
        # sync messages to latest point_cloud frame.
        latest_point_cloud_msg = self.msgbuf.wr_lidar[-1]
        if latest_point_cloud_msg is not None:
            point_cloud = latest_point_cloud_msg.data
            synced_lower_image_msg = self.msgbuf.lower_img.find_nearest(latest_point_cloud_msg)
            synced_lower_box_msg = self.msgbuf.lower_box.find_nearest(latest_point_cloud_msg)
            if synced_lower_image_msg is not None and synced_lower_box_msg is not None: # core routine
                #t1 = time()
                lower_img_raw = synced_lower_image_msg.data
                #self.am.update(time()-t1)
                lower_bboxes = synced_lower_box_msg.data
                vehicle_speed = self.speed(new_frame=lower_img_raw, new_time=synced_lower_image_msg.header.stamp.to_sec())
                scr.addstr(10, 4, "vehicle_speed = %s    " % str(vehicle_speed))
                result = self.fusion_detect(vehicle_speed, lower_bboxes, point_cloud, lower_img_raw) # core function
                self.visualizer.update(lower_img_raw=lower_img_raw, dynamic_detected=result.dynamic_detected)
            elif synced_lower_image_msg is not None: # only do image update for visualization
                lower_img_raw = synced_lower_image_msg.data
                self.visualizer.update(lower_img_raw=lower_img_raw)
            else: cv2.waitKey(1)
        elif len(self.msgbuf.lower_img) > 1:
            oldest_lower_image_msg = self.msgbuf.lower_img[0]
            oldest_lower_image = oldest_lower_image_msg.data
            self.visualizer.update(lower_img_raw=oldest_lower_image)
        else: cv2.waitKey(1)
        #scr.addstr(11, 4, "msec1 = %s    " % str(self.am.avg))

def main(scr):
    pprint.pprint(vars(opts))
    detector = FusionDumpsterDetectorNode(opts)
    scr.clear()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if int(time()) % 10 == 0: scr.clear()
        detector.spin_once(scr)
        sys.stdout.flush()
        scr.refresh()
        rate.sleep()

if __name__ == "__main__":
    opts = configargparse.ArgParser()
    opts.add('-c', required=True, is_config_file=True, help='config file path')
    opts.add('--lower_img_sub_topic', default="/undistort_lower/compressed", help="default: %(default)s")
    opts.add('--upper_img_sub_topic', default="/undistort_upper/compressed", help="default: %(default)s")
    opts.add('--wr_lidar_sub_topic', default="/wr_scan", help="default: %(default)s")
    opts.add('--lower_box_sub_topic', default="/yolo_detect_lower/bbox_results", help="default:%(default)s")
    opts.add('--upper_boxes_sub_topic', default="/yolo_detect_upper/bbox_results", help="default:%(default)s")
    opts.add('--dumpster_pub_topic', default="/dumpster_detection/dumpster_location", help="default: %(default)s")
    opts.add('--lidar_to_lower_extrinsics', type=utils.cfg_extrinsic_type)
    opts.add('--lower_img_intrinsics', type=lambda x:np.array(eval(eval(x))))
    opts.add('--lower_img_detect_intrinsics', type=lambda x:np.array(eval(eval(x))))
    opts.add('--lower_img_detect_frame_shape', type=lambda x:tuple(eval(eval(x))))
    opts.add('--speed_threshold', type=float, default=5)
    opts.add('--fullscreen', action='store_true', default=False)
    # opts.add('--debug', action='store_true', help="This debug flag is used to visualize the detection results.\n")
    opts = opts.parse_args()
    curses.wrapper(main)
