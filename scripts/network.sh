#!/bin/bash -ex

SDIR=$(dirname "$0")
source $SDIR/env.sh

setup_network () {
  local DEVICE_NAME=$1
  local NEW_IP=$2
  local NEW_GATEWAY=$3
  local PASSWORD=1
  echo $PASSWORD | sudo -S echo "password set"
  sudo ip link set dev $DEVICE_NAME down
  sudo ip addr flush $DEVICE_NAME
  sudo ip route flush dev $DEVICE_NAME
  sudo ip addr add $NEW_IP/24 dev $DEVICE_NAME
  sudo ip link set dev $DEVICE_NAME up
  sudo ip addr show dev $DEVICE_NAME
  sudo ip route add default via $NEW_GATEWAY
  sudo ip route show default
}

setup_network eth0 $ROS_IP $GATEWAY_IP
