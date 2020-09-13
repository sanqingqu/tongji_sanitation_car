#!/bin/bash -ex

SDIR=$(dirname "$0")
source $SDIR/env.sh

rosbag play -l --topics /undistort_lower/compressed /undistort_upper/compressed /wr_scan -- $@
