#!/bin/bash -ex

SDIR=$(dirname "$0")
source $SDIR/env.sh

#rosparam set use_sim_time true
#rosbag play -l --topics /undistort_lower/compressed /undistort_upper/compressed /wr_scan -r 1.0 --clock -- $@
rosbag play -l --topics /undistort_lower/compressed /undistort_upper/compressed /wr_scan -r 0.5 -- $@
