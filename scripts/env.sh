#!/bin/bash -ex

### Configurations ###
SUBNET=192.168.0
ROS_MASTER_IP=$SUBNET.1
ROS_CLIENT_0_IP=$SUBNET.2
DEBUG_DESKTOP_IP=0
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
  for ip_addr in $(hostname -I); do
      if [ ${ip_addr:0:${#SUBNET}} == $SUBNET ]; then
          DEBUG_DESKTOP_IP=$ip_addr
      fi
  done
  if [ $DEBUG_DESKTOP_IP != 0 ]; then
      echo $DEBUG_DESKTOP_IP "ok"
      export ROS_IP=$DEBUG_DESKTOP_IP
  else
      echo "ERROR: Unknown hostname $HOSTNAME!"
      exit 1
  fi
fi

SETUP_BASH=$(dirname ${BASH_SOURCE[0]})/../devel/setup.bash
if [ -e $SETUP_BASH ]; then source "$SETUP_BASH"; fi
