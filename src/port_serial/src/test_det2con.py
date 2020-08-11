#!/usr/bin/env python

import rospy
#from camera_lidar_fusion_detection.msg import DumpsterCAN
from camera_lidar_fusion_detection.msg import DumpsterInfo
import numpy as np

#byte_0 = np.uint8(0).tobytes()
#byte_1 = np.uint8(1).tobytes()
#byte_2 = np.uint8(2).tobytes()
#byte_3 = np.uint8(3).tobytes()
#byte_4 = np.uint8(4).tobytes()
#byte_5 = np.uint8(5).tobytes()
#byte_6 = np.uint8(6).tobytes()
#byte_7 = np.uint8(7).tobytes()
#dumpster_info = [byte_0,byte_1,byte_2,byte_3,byte_4,byte_5,byte_6,byte_7]
#test msg(received from fusion detection)
dumpster_info = [0,1,2,3,4,5]

def talker():
    rospy.init_node('detection', anonymous=True)
    pub = rospy.Publisher('/dumpster_detection/dumpster_location', DumpsterInfo, queue_size=3)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        dumpster_msg = DumpsterCAN()
        dumpster_msg.header.stamp = rospy.Time.now()
        #dumpster_msg.byte_string = ''.join(dumpster_info)
        dumpster_msg.continuous_dumpster = dumpster_info[0]
        dumpster_msg.lateral_dis = dumpster_info[1]
        dumpster_msg.side_offset = dumpster_info[2]
        dumpster_msg.object_width = dumpster_info[3]
        dumpster_msg.left_gap = dumpster_info[4]
        dumpster_msg.right_gap = dumpster_info[5]
        #dumpster_info = map(lambda x:np.uint8(ord(x)), dumpster_info)

        #dumpster_msg.byte_0 = dumpster_info[0]
        #dumpster_msg.byte_1 = dumpster_info[1]
        #dumpster_msg.byte_2 = dumpster_info[2]
        #dumpster_msg.byte_3 = dumpster_info[3]
        #dumpster_msg.byte_4 = dumpster_info[4]
        #dumpster_msg.byte_5 = dumpster_info[5]
        #dumpster_msg.byte_6 = dumpster_info[6]
        #dumpster_msg.byte_7 = dumpster_info[7]

        rospy.loginfo(dumpster_msg)
        pub.publish(dumpster_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
