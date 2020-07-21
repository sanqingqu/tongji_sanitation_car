#!/usr/bin/env python

import rospy
import serial
import serial.tools.list_ports
from port_serial.msg import DumpsterCAN
from std_msgs.msg import String
import time
import threading

port_list = list(serial.tools.list_ports.comports())
#Select the serial port as required
serial_list = list(port_list[0])
serialName = serial_list[0]
ser = serial.Serial(serialName, 460800,timeout=0.5)
#publish control message
pub = rospy.Publisher('control_msg', String, queue_size=3)

if ser.isOpen():
    print(serialName,' is open', ser.isOpen())
else:
    ser.open()
    print('port is open', ser.isOpen())

def thread_job():
    rospy.spin()

def callback(data):
    ser.write(data.byte_string)

def SubscribeAndPublish():
    rospy.init_node('serial_data_control', anonymous=True)
    #change topic name as required
    rospy.Subscriber('/dumpster_detection/dumpster_location', DumpsterCAN, callback) 
    rate = rospy.Rate(10) 
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    while not rospy.is_shutdown(): 
        serial_data = ser.readline()
        if serial_data != '':
            #Accept the message from the control module, the test msg type is string.
            rospy.loginfo("Received a control message!")
            rospy.loginfo(serial_data)
            #Change the publish data as required.
            pub.publish(serial_data)
    rate.sleep()

if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass
