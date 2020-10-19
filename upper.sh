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
echo "sudo 1" | sudo  -S chmod 666 /dev/ttyUSB0
v4l2-ctl -d /dev/video0 -c sharpness=0
sleep 30s
roslaunch ./src/upper.launch

exit 0 


 
