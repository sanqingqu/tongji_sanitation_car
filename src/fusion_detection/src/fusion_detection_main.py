#!/usr/bin/env python
#pylint: disable=no-member
import configargparse
import cv_bridge
import curses
import dumpster_fitting
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
from fusion_detection.msg import DumpsterInfo
from sklearn.cluster import DBSCAN
from time import time


class FusionDumpsterDetectorNode:

    def __init__(self, opts, scr, node_name="dumpster_detection"):
        self.opts = opts
        self.scr = scr
        self.cv_bridge = cv_bridge.CvBridge()
        self.extract_img = self.cv_bridge.compressed_imgmsg_to_cv2
        self.lower_reverter = utils.BBoxTransformer(
            self.opts.lower_img_detect_intrinsics,
            self.opts.lower_img_intrinsics)
        self.upper_reverter = utils.BBoxTransformer(
            self.opts.upper_img_detect_intrinsics,
            self.opts.lower_img_intrinsics)
        #self.dumpster_fit = dumpster_fitting.DumpsterFitter()
        self.visualizer = visualizer.Visualizer(
                full_screen=self.opts.fullscreen, roi=self.opts.trashbin_roi_bbox)
        self.msgbuf = easydict.EasyDict({
            'wr_lidar' : utils.MsgBuffer(size=3),
            'lower_img' : utils.MsgBuffer(size=3),
            'lower_box' : utils.MsgBuffer(size=3),
            'upper_img' : utils.MsgBuffer(size=3),
            'upper_box' : utils.MsgBuffer(size=3),
        })
        self.detect_results = easydict.EasyDict({
            "vehicle_speed" : speed.SpeedEstimation(),
            "bin_track_cnt" : 0,
            "bin_point_cnt" : 0,
            "latest_binbox" : None,
            "nearest_dist" : None,
            "trashbin_width" : utils.MedianMeter(),
            "human_detected" : 0,
            "dynamic_detected" : False,
            #add human_bbox_avg_size for setting 
            "human_bbox_size":0,
        })
        self.visualizable_results_keys = ["vehicle_speed", "dynamic_detected", "bin_track_cnt", "nearest_dist", "bin_point_cnt", "human_detected","human_bbox_size",]
        self.mainloop_rate_timer = utils.Timer()
        self.fusion_synced_timer = utils.Timer()
        self.fusion_detect_timer = utils.Timer()
        self.vspeed_detect_timer = utils.Timer()
        self.visual_update_timer = utils.Timer()
        rospy.init_node(node_name, anonymous=True)
        rospy.Subscriber(self.opts.wr_lidar_sub_topic, LaserScan, self.msgbuf_callback_factory('wr_lidar', self.extract_wr_lidar), queue_size=3)
        rospy.Subscriber(self.opts.lower_img_sub_topic, CompressedImage, self.msgbuf_callback_factory('lower_img', self.extract_img), queue_size=3)
        rospy.Subscriber(self.opts.lower_box_sub_topic, BBoxArray, self.msgbuf_callback_factory('lower_box', self.extract_bbox_array), queue_size=3)
        rospy.Subscriber(self.opts.upper_img_sub_topic, CompressedImage, self.msgbuf_callback_factory('upper_img', self.extract_img), queue_size=3)
        rospy.Subscriber(self.opts.upper_box_sub_topic, BBoxArray, self.msgbuf_callback_factory('upper_box', lambda x:self.extract_bbox_array(x, lower=False)), queue_size=3)
        self.dumpster_detect_publisher = rospy.Publisher(self.opts.dumpster_pub_topic, DumpsterInfo, queue_size=1)

    def pdb(self):
        utils.pdb(self.scr)

    def msgbuf_callback_factory(self, msg_key, extract):
        def callback(msg):
            msg.header.stamp = rospy.get_rostime()
            #t1 = time()
            self.msgbuf[msg_key].append(Namespace(header=msg.header, data=extract(msg)))
            #self.am.update(time()-t1)
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
    
    def extract_bbox_array(self, bbox_array_msg, lower=True):
        reverter = self.lower_reverter if lower else self.upper_reverter
        bboxes = list(map(reverter, bbox_array_msg.bboxes))
        return bboxes

    def largest_cluster(self, lidar_points):
        assert(len(lidar_points))
        clustering = DBSCAN(eps=0.1, min_samples=20).fit(lidar_points[:, :2, 0]) # [-1, 2]
        num_cluster = clustering.labels_.max() + 1
        return np.argmax([sum(i == clustering.labels_) for i in range(num_cluster)] + [-2]) == clustering.labels_

    def compose_detected_bin_message(self):
        # find left and right neighbour transhbin
        bboxes_id_sorted_by_center_x = np.argsort([(b.x_top_left + b.x_bottom_right)/2 for b in self.detect_results.cand_trash_boxes])
        bboxes_id_sorted_by_center_x_nearest_trashbin_id = np.argwhere(
            bboxes_id_sorted_by_center_x == self.detect_results.nearest_trashbin_id).item()
        if bboxes_id_sorted_by_center_x_nearest_trashbin_id != 0:
            left_trashbin_id = bboxes_id_sorted_by_center_x[bboxes_id_sorted_by_center_x_nearest_trashbin_id - 1]
            left_trashbin_selector = self.detect_results.cand_trash_selectors[left_trashbin_id]
            left_trashbin_points = self.detect_results.point_cloud["lidar"][left_trashbin_selector]
            if 0 == len(left_trashbin_points): left_trashbin_points = None
        else:
            left_trashbin_points = None
        if bboxes_id_sorted_by_center_x_nearest_trashbin_id != len(bboxes_id_sorted_by_center_x) - 1:
            right_trashbin_id = bboxes_id_sorted_by_center_x[bboxes_id_sorted_by_center_x_nearest_trashbin_id + 1]
            right_trashbin_selector = self.detect_results.cand_trash_selectors[right_trashbin_id]
            right_trashbin_points = self.detect_results.point_cloud["lidar"][right_trashbin_selector]
            if 0 == len(right_trashbin_points): right_trashbin_points = None
        else:
            right_trashbin_points = None
        nearest_x = self.detect_results.lidar_bin_points[:, 1, 0]
        # estimate accurate position
        dumpster_msg = DumpsterInfo()
        dumpster_msg.header.stamp = rospy.Time.now()
        dumpster_msg.continuous_dumpster = left_trashbin_points is not None
        dumpster_msg.emergency_stop = False
        dumpster_msg.lateral_dis = self.detect_results.nearest_dist
        dumpster_msg.side_offset = - nearest_x.mean().item()
        dumpster_msg.object_width = self.detect_results.trashbin_width.value()
        dumpster_msg.left_gap = (left_trashbin_points[:, 1, 0].min() - nearest_x.max()) if left_trashbin_points is not None else 1.5
        dumpster_msg.right_gap = (nearest_x.min() - right_trashbin_points[:, 1, 0].max()) if right_trashbin_points is not None else 1.5
        return dumpster_msg

    def compose_emergency_stop_message(self):
        dumpster_msg = DumpsterInfo()
        dumpster_msg.header.stamp = rospy.Time.now()
        dumpster_msg.emergency_stop = True
        return dumpster_msg
    def always_message(self):
        dumpster_msg = DumpsterInfo()
        dumpster_msg.header.stamp = rospy.Time.now()
        dumpster_msg.continuous_dumpster = False
        dumpster_msg.emergency_stop = False
        dumpster_msg.lateral_dis = 0
        dumpster_msg.side_offset = 0
        dumpster_msg.object_width = 0
        dumpster_msg.left_gap = 0
        dumpster_msg.right_gap = 0
        return dumpster_msg   
    def bin_points_in_box(self, point_cloud, bbox):
        x, y = point_cloud["image"][:, 0], point_cloud["image"][:, 1]
        selector = np.logical_and(
                np.logical_and(bbox.x_top_left - 5 < x, x < bbox.x_bottom_right + 5),
                np.logical_and(bbox.y_top_left - 5 < y, y < bbox.y_bottom_right + 5))
        lidar_bin_points = point_cloud['lidar'][selector]
        if 0 == len(lidar_bin_points): return selector
        largest_cluster = self.largest_cluster(lidar_bin_points)
        selector[selector] = largest_cluster
        return selector
    
    def human_detect(self, upper_bboxes):
        if upper_bboxes is None: return
        
        #------------------------human detected-----------------
        self.detect_results.cand_human_boxes = [bbox for bbox in upper_bboxes if bbox.class_id in [0] and bbox.score > self.opts.human_threshold]
        self.detect_results.hazard_human_boxes = [bbox for bbox in self.detect_results.cand_human_boxes
                if bbox.x_top_left > self.opts.trashbin_roi_bbox.x_top_left-50 and \
                bbox.x_bottom_right < self.opts.trashbin_roi_bbox.x_bottom_right+50]
        if len(self.detect_results.hazard_human_boxes) > 2: 
            self.detect_results.human_detected = 1 #0
            #get average human bbox size when detecting human in area
            self.detect_results.human_bbox_size = int(sum(abs(bbox.x_bottom_right-bbox.x_top_left)*abs(bbox.y_bottom_right-bbox.y_top_left) for bbox in self.detect_results.hazard_human_boxes)/len(self.detect_results.hazard_human_boxes))
            
        elif self.detect_results.human_detected > 0: 
            self.detect_results.human_detected = 0 #-= 1
            self.detect_results.human_bbox_size=0
        else: pass
                
    def fusion_detect_trashbin(self, vehicle_speed, lower_bboxes, upper_bboxes, point_cloud, lower_img_raw):
        # get all candidates trash bin and their points.
        self.detect_results.cand_trash_boxes = [bbox for bbox in lower_bboxes
                if bbox.class_id in [0] and bbox.score > self.opts.trashbin_threshold]
        self.detect_results.cand_trash_selectors = [self.bin_points_in_box(point_cloud, bbox)
                for bbox in self.detect_results.cand_trash_boxes]
        self.detect_results.point_cloud = point_cloud
        # detect nearest_trashbin which is in ROI.
        nearest_trashbin = None
        if len(self.detect_results.cand_trash_boxes):
            nearest_trashbin_id = utils.argmin([abs(bbox.x_top_left + bbox.x_bottom_right - lower_img_raw.shape[1])
                    for bbox in self.detect_results.cand_trash_boxes])
            nearest_trashbin = self.detect_results.cand_trash_boxes[nearest_trashbin_id]
            trashbin_in_roi = nearest_trashbin.x_top_left > self.opts.trashbin_roi_bbox.x_top_left and \
                    nearest_trashbin.x_bottom_right < self.opts.trashbin_roi_bbox.x_bottom_right
            if not trashbin_in_roi: nearest_trashbin = None
            self.detect_results.nearest_trashbin_id = nearest_trashbin_id
        # disable nearest_trashbin when dynamic_detected
        self.detect_results.dynamic_detected = vehicle_speed > self.opts.speed_threshold
        if not self.opts.disable_dynamic_filter:
            if self.detect_results.dynamic_detected: nearest_trashbin = None
        # disable nearest_trashbin when human detected
        if not self.opts.disable_human_detection:
            self.human_detect(upper_bboxes)
            #if self.detect_results.dynamic_detected or self.detect_results.human_detected > 0:
            if not self.detect_results.dynamic_detected and self.detect_results.human_detected > 0:
                emergency_stop_message = self.compose_emergency_stop_message()
                self.dumpster_detect_publisher.publish(emergency_stop_message)
                nearest_trashbin = None
        # disable nearest_trashbin when no corresponding point cloud in box
        if nearest_trashbin is not None:
            nearest_selector = self.detect_results.cand_trash_selectors[nearest_trashbin_id]
            self.detect_results.lidar_bin_points = point_cloud["lidar"][nearest_selector]
            self.detect_results.image_bin_points = point_cloud["image"][nearest_selector]
            self.detect_results.bin_point_cnt = len(self.detect_results.lidar_bin_points)
            if 0 == self.detect_results.bin_point_cnt: nearest_trashbin = None
        else:
            self.detect_results.lidar_bin_points = None
            self.detect_results.image_bin_points = None
            self.detect_results.bin_point_cnt = None
        # disable nearest_trashbin when point cloud too far
        if nearest_trashbin is not None:
            self.detect_results.nearest_dist = round(self.detect_results.lidar_bin_points[:, 0].max(), 2)
            #if self.detect_results.nearest_dist > self.opts.trashbin_nearest_dist_upper_bound: nearest_trashbin = None
        else:
            self.detect_results.nearest_dist = None
        # track nearest_trashbin which is in ROI
        if nearest_trashbin is not None: # new observation
            if self.detect_results.latest_binbox is None: # first observation
                self.detect_results.bin_track_cnt = 1
                self.detect_results.latest_binbox = nearest_trashbin
            elif utils.bbox_distance(nearest_trashbin, self.detect_results.latest_binbox) \
                    < self.opts.trashbin_tracking_distance_threshold: # continuous observation
                self.detect_results.bin_track_cnt += 1
                self.detect_results.latest_binbox = nearest_trashbin
            self.detect_results.trashbin_width.update(
                self.detect_results.lidar_bin_points[:, 1, 0].ptp().item())
        else: # lose observation
            self.detect_results.trashbin_width.reset()
            self.detect_results.bin_track_cnt = 0
            self.detect_results.latest_binbox = None
        # best_valid_trashbin
        if self.detect_results.bin_track_cnt > self.opts.trashbin_tracking_count_threshold:
            self.detect_results.best_valid_trashbin = nearest_trashbin
            detected_bin_message = self.compose_detected_bin_message()
            self.dumpster_detect_publisher.publish(detected_bin_message)
        else:
            self.detect_results.best_valid_trashbin = None
            always_zero_message = self.always_message()
            self.dumpster_detect_publisher.publish(always_zero_message)
    
    def scr_monitor_msgbuf(self):
        self.scr.addstr(2, 4, "=== Messages ===")
        for i, (k, buf) in enumerate(self.msgbuf.items()):
            self.scr.addstr(i+3, 4, "[%s:\t%03d]    " % (k, len(buf)))

    def scr_monitor_timer(self):
        self.scr.addstr(2, 30, "=== Timer/[msec] ===")
        self.scr.addstr(3, 30, "[mainloop_rate = %s]    " % self.mainloop_rate_timer.fmt_msec())
        self.scr.addstr(4, 30, "[fusion_synced = %s]    " % self.fusion_synced_timer.fmt_msec())
        self.scr.addstr(5, 30, "[fusion_detect = %s]    " % self.fusion_detect_timer.fmt_msec())
        self.scr.addstr(6, 30, "[vspeed_detect = %s]    " % self.vspeed_detect_timer.fmt_msec())
        self.scr.addstr(7, 30, "[visual_update = %s]    " % self.visual_update_timer.fmt_msec())

    def scr_monitor_results(self):
        self.scr.addstr(2, 64, "=== Results ===")
        i = 0
        for k, result in self.detect_results.items():
            if k not in self.visualizable_results_keys: continue
            self.scr.addstr(i+3, 64, "[%s: %s]    " % (k, str(result)))
            i += 1
    
    def spin_once(self):
        self.mainloop_rate_timer.count()
        self.scr_monitor_msgbuf()
        self.scr_monitor_timer()
        oldest_point_cloud_msg = self.msgbuf.wr_lidar[0]
        if oldest_point_cloud_msg is not None:
            # sync messages to latest point_cloud frame.
            synced_lower_image_msg = self.msgbuf.lower_img.find_nearest(oldest_point_cloud_msg)
            synced_lower_box_msg = self.msgbuf.lower_box.find_nearest(oldest_point_cloud_msg)
            synced_upper_image_msg = self.msgbuf.upper_img.find_nearest(oldest_point_cloud_msg)
            synced_upper_box_msg = self.msgbuf.upper_box.find_nearest(oldest_point_cloud_msg)
            if synced_lower_image_msg is not None and synced_lower_box_msg is not None:
                # messages synchronize succeeded.
                point_cloud = oldest_point_cloud_msg.data
                lower_img_raw = synced_lower_image_msg.data
                lower_bboxes = synced_lower_box_msg.data
                upper_img_raw = synced_upper_image_msg.data if synced_upper_image_msg else None
                upper_bboxes = synced_upper_box_msg.data if synced_upper_box_msg else []
                self.fusion_synced_timer.count()
                # update vehicle_speed
                with self.vspeed_detect_timer:
                    self.detect_results.vehicle_speed(new_frame=lower_img_raw, new_time=synced_lower_image_msg.header.stamp.to_sec())
                # start camera lidar fusion detection
                with self.fusion_detect_timer:
                    self.fusion_detect_trashbin(self.detect_results.vehicle_speed(), lower_bboxes, upper_bboxes, point_cloud, lower_img_raw)
                # update visualization
                with self.visual_update_timer:
                    self.visualizer.update(lower_img_raw=lower_img_raw, upper_img_raw=upper_img_raw, detect_results=self.detect_results)
            elif synced_lower_image_msg is not None:
                # synchronization failed, only do image update for visualization
                lower_img_raw = synced_lower_image_msg.data
                with self.visual_update_timer:
                    self.visualizer.update(lower_img_raw=lower_img_raw)
            elif synced_upper_image_msg is not None:
                # synchronization failed, only do image update for visualization
                upper_img_raw = synced_upper_image_msg.data
                upper_bboxes = synced_upper_box_msg.data if synced_upper_box_msg else None
                self.human_detect(upper_bboxes)
                with self.visual_update_timer:
                    self.visualizer.update(upper_img_raw=upper_img_raw)
            else: self.visualizer.waitKey() # keep window responsive
        elif len(self.msgbuf.lower_img) > 1:
            oldest_lower_image_msg = self.msgbuf.lower_img[0]
            oldest_lower_image = oldest_lower_image_msg.data
            synced_upper_image_msg = self.msgbuf.upper_img.find_nearest(oldest_lower_image_msg)
            upper_img_raw = synced_upper_image_msg.data if synced_upper_image_msg else None
            with self.vspeed_detect_timer:
                self.detect_results.vehicle_speed(new_frame=oldest_lower_image, new_time=oldest_lower_image_msg.header.stamp.to_sec())
            with self.visual_update_timer:
                self.visualizer.update(lower_img_raw=oldest_lower_image, upper_img_raw=upper_img_raw)
        else: self.visualizer.waitKey() # keep window responsive
        self.scr_monitor_results()

def main(scr):
    detector = FusionDumpsterDetectorNode(opts, scr)
    scr.nodelay(1)
    scr.clear()
    rate = rospy.Rate(opts.main_loop_rate)
    while not rospy.is_shutdown():
        if int(time()) % 10 == 0: scr.clear()
        if 27 == scr.getch(): break
        detector.spin_once()
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
    opts.add('--upper_box_sub_topic', default="/yolo_detect_upper/bbox_results", help="default:%(default)s")
    opts.add('--dumpster_pub_topic', default="/dumpster_detection/dumpster_location", help="default: %(default)s")
    opts.add('--lidar_to_lower_extrinsics', type=utils.cfg_extrinsic_type)
    opts.add('--lower_img_intrinsics', type=lambda x:np.array(eval(eval(x))))
    opts.add('--lower_img_detect_intrinsics', type=lambda x:np.array(eval(eval(x))))
    opts.add('--upper_img_detect_intrinsics', type=lambda x:np.array(eval(eval(x))))
    opts.add('--lower_img_detect_frame_shape', type=lambda x:tuple(eval(eval(x))))
    opts.add('--speed_threshold', type=float, default=5)
    opts.add('--fullscreen', action='store_true', default=False)
    opts.add('--disable_dynamic_filter', action='store_true', default=False)
    opts.add('--disable_human_detection', action='store_true', default=False)
    opts.add('--main_loop_rate', type=int, default=25)
    opts.add('--trashbin_threshold', type=float, default=0.98)
    opts.add('--human_threshold', type=float, default=0.2)
    opts.add('--trashbin_roi_bbox', type=utils.cfg_bbox_type)
    opts.add('--trashbin_tracking_count_threshold', type=float, default=8)
    opts.add('--trashbin_tracking_distance_threshold', type=float, default=20)
    opts.add('--trashbin_nearest_dist_upper_bound', type=float, default=1.5)
    # opts.add('--debug', action='store_true', help="This debug flag is used to visualize the detection results.\n")
    opts = opts.parse_known_args()[0]
    pprint.pprint(vars(opts))
    curses.wrapper(main)
