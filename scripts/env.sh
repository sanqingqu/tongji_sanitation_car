#!/bin/bash -ex

### Configurations ###
ROS_MASTER_IP=192.168.101.1
ROS_CLIENT_0_IP=192.168.101.2

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

setup_network () {
  local DEVICE_NAME=$1
  local NEW_IP=$2
  local NEW_GATEWAY=$3
  ip link set dev $DEVICE_NAME down
  ip addr flush $DEVICE_NAME
  ip route flush dev $DEVICE_NAME
  ip addr add $NEW_IP/24 dev $DEVICE_NAME
  ip link set dev $DEVICE_NAME up
  ip addr show dev $DEVICE_NAME
  ip route add default via $NEW_GATEWAY
  ip route show default
}

setup_network eth0 $ROS_IP $ROS_MASTER_IP

exit 0
