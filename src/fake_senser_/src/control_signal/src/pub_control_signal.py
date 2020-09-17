#!/usr/bin/env python
import rospy
from control_signal.msg import Signal

pub=rospy.Publisher("control_signal_result",Signal,queue_size=10)#publish to channel"control_signal_result"
rospy.init_node("control_signal",anonymous=True)#define node name


#publish message
def talker():
    signal=0#0:start work;1:stop work
    pub_signal=Signal()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_signal.header.stamp=rospy.Time.now()
        pub_signal.signal=signal
        pub.publish(pub_signal)
        rate.sleep()


if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
	pass

