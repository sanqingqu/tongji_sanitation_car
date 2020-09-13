#!/bin/bash -ex

### Configurations ###
SUBNET=192.168.181
ROS_MASTER_IP=$SUBNET.1
ROS_CLIENT_0_IP=$SUBNET.2
GATEWAY_IP=$SUBNET.254


### Globals ###
export HOSTNAME=$(hostname)
export ROS_HOSTNAME=$HOSTNAME
export ROS_MASTER_URI=http://$ROS_MASTER_IP:11311

### Setups ###

if [ $HOSTNAME == "tjsan_lower_nano" ]; then
  export ROS_IP=$ROS_MASTER_IP
elif [ $HOSTNAME == "tjsan_upper_nano" ]; then
  export ROS_IP=$ROS_CLIENT_0_IP
else
  echo "ERROR: Unknown hostname $HOSTNAME!"
  exit 1
fi
