#!/bin/bash -ex

SDIR=$(dirname "$0")
source $SDIR/env.sh

if [ ! -e /usr/include/opencv ] && [ -e /usr/include/opencv4 ] ; then 
    ln -s /usr/include/opencv4/ /usr/include/opencv
fi

cd $SDIR/..

if [ $HOSTNAME == "tjsan_lower_nano" ]; then
  catkin_make --cmake-args -DLOWERCAM=ON
elif [ $HOSTNAME == "tjsan_upper_nano" ]; then
  catkin_make --cmake-args -DLOWERCAM=OFF
else
  echo "ERROR: Unknown hostname $HOSTNAME!"
  exit 1
fi

