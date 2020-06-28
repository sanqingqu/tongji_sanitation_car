#! /usr/bin/env python

import numpy as np

import roslib
import rospy

from sensor_msgs.msg import LaserScan


class LaserExtraction(object):

    def __init__(self, sub_topic="/wr_scan", save_data=False):
        
        self.sub_topic = sub_topic
        self.save_data = save_data
        self.subscriber = rospy.Subscriber(self.sub_topic, LaserScan,
                            self.laser_extraction_callback, queue_size=1)


    def laser_extraction_callback(self, laser_msg):
        scan_rads = np.arange(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
        scan_ranges  = np.array(laser_msg.ranges)

        filter_flag = scan_ranges < np.inf
        scan_ranges = scan_ranges[filter_flag]
        scan_rads = scan_rads[filter_flag]

        scan_x = (scan_ranges * np.cos(scan_rads)).reshape(-1, 1)
        scan_y = (scan_ranges * np.sin(scan_rads)).reshape(-1, 1)

        scan_np = np.concatenate((scan_x, scan_y), axis=1)
        print(scan_np)

def main():
    laser_ext = LaserExtraction()
    rospy.init_node("laser_extraction", anonymous=True)
    print "init the laser_scan msg extraction node"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the ROS laser_scan msg extraction module"
    
if __name__ == "__main__":
    main()