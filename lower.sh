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

# set network ip
cd /home/zhgz/tongji_sanitation_car/
./scripts/network.sh
source ./scripts/env.sh
roscore&
#./scripts/network.sh
gnome-terminal -x bash -c "source ./scripts/env.sh;v4l2-ctl -d /dev/video0 -c sharpness=0;roslaunch ./src/lower.launch;exec bash"

exit 0 


 
