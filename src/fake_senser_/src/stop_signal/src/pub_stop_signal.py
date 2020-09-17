#!/usr/bin/env python
import rospy
from stop_signal.msg import Stopflag

pub=rospy.Publisher("stop_signal_result",Stopflag,queue_size=10)#publish to channel"stop_signal_result"
rospy.init_node("stop_signal",anonymous=True)#define node name


#publish message
def talker():
    signal=0#0:run;1:stop
    pub_signal=Stopflag()
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

