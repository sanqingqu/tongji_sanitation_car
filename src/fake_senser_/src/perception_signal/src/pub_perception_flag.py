#!/usr/bin/env python
import rospy
from perception_signal.msg import Perceptionflag

pub=rospy.Publisher("perception_signal_result",Perceptionflag,queue_size=10)#publish to channel"perception_signal_result"
rospy.init_node("perception_signal",anonymous=True)#define node name


#publish message
def talker():
    signal=0#0:start work;1:stop work
    pub_signal=Perceptionflag()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_signal.header.stamp=rospy.Time.now()
        pub_signal.flag=signal
        pub.publish(pub_signal)
        rate.sleep()


if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
	pass

