#!/usr/bin/env python
import rospy
from speed_detection.msg import Speedflag

pub=rospy.Publisher("speed_detection_result",Speedflag,queue_size=10)#publish to channel"speed_detection_result"
rospy.init_node("speed_detection",anonymous=True)#define node name


#publish message
def talker():
    speed_flag=False
    pub_speed_detection=Speedflag()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_speed_detection.header.stamp=rospy.Time.now()
        pub_speed_detection.flag=speed_flag
        pub.publish(pub_speed_detection)
        rate.sleep()


if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
	pass

