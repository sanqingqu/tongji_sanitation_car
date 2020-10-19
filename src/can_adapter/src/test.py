#!/usr/bin/env python

ENABLE_SERIAL = True

import numpy as np
import rospy
import serial
import serial.tools.list_ports
import struct
import time
import threading



port_list = list(serial.tools.list_ports.comports())
serial_list = list(port_list[0])
print(port_list)
serialName = serial_list[0]
ser = serial.Serial(serialName, 460800, timeout=0.5)
