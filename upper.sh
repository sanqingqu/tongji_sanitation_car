#!/bin/bash

### BEGIN INIT INFO
# Provides:          launch upper node
# Required-Start:    $local_fs  $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: launch upper node
# Description:       launch all upper node
### END INIT INFO

cd Desktop/tongji_sanitation_car/
./scripts/network.sh
source ./scripts/env.sh
#gnome-terminal -x bash -c "source ./scripts/env.sh;v4l2-ctl -d /dev/video0 -c sharpness=0;rosrun undistortion camera_undistorted_tj_upper.py;exec bash"

#gnome-terminal -x bash -c "source ./scripts/env.sh;rosrun fusion_detection fusion_detect_main.py -c /home/zhgz/Desktop/tongji_sanitation_car/src/fusion_detection/config/default.yml;exec bash"

#gnome-terminal -x bash -c "source ./scripts/env.sh;echo "sudo 1" | sudo  -S chmod 666 /dev/ttyUSB0;rosrun can_adapter serial_communication.py;exec bash"
echo "sudo 1" | sudo  -S chmod 666 /dev/ttyUSB0
v4l2-ctl -d /dev/video0 -c sharpness=0
sleep 20
roslaunch ./src/upper.launch

exit 0

 
