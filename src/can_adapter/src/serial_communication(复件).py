#!/usr/bin/env python

ENABLE_SERIAL = True

import numpy as np
import rospy
import serial
import serial.tools.list_ports
import struct
import time
import threading

from can_adapter.msg import DumpsterCAN
from fusion_detection.msg import DumpsterInfo
from std_msgs.msg import String

def open_first_serial_device():
    port_list = list(serial.tools.list_ports.comports())
    serial_list = list(port_list[0])
    serialName = serial_list[0]
    ser = serial.Serial(serialName, 460800, timeout=0.5)
    if ser.isOpen():
        print(serialName,' is open', ser.isOpen())
    else:
        ser.open()
        print('port is open', ser.isOpen())
    return ser

def clamp(x, smallest, largest):
    return max(smallest, min(largest, x))

def thread_job():
    rospy.spin()

def callback(data):
    byte_0 = np.uint8((1<<6) if data.continuous_dumpster else 0 + (1<<7) if data.emergency_stop else 0).tobytes()
    byte_1 = np.uint8(clamp(round(data.lateral_dis * 1000 / 4), 0, 255)).tobytes()
    byte_23 = np.float16(data.side_offset ).newbyteorder('<').tobytes()
    byte_45 = np.float16(data.object_width).newbyteorder('<').tobytes()
    byte_6 = np.uint8(clamp(round(data.left_gap  * 1000 / 4), 0, 255)).tobytes()
    byte_7 = np.uint8(clamp(round(data.right_gap * 1000 / 4), 0, 255)).tobytes()
    can_msg = b''.join([byte_0, byte_1, byte_23, byte_45, byte_6, byte_7])
    for i in can_msg: print(hex(ord(i))),
    print('')
    if ENABLE_SERIAL:
        ser.write(can_msg)

def main():
    rospy.init_node('serial_data_control', anonymous=True)
    pub = rospy.Publisher('control_msg', String, queue_size=3)
    rospy.Subscriber('/dumpster_detection/dumpster_location', DumpsterInfo, callback) 
    if ENABLE_SERIAL:
        ser = open_first_serial_device()
    #print(ser)
    rate = rospy.Rate(100) 
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    while not rospy.is_shutdown(): 
        if ENABLE_SERIAL:
            serial_data = ser.read(8)
            if serial_data != '':
                #Accept the message from the control module, the test msg type is string.
                rospy.loginfo("Received a control message!")
                rospy.loginfo(serial_data)
                #Change the publish data as required.
                pub.publish(serial_data)
        rate.sleep()

if __name__ == '__main__':
    main()
