#!/usr/bin/env python
import rospy
from up_camera_detection.msg import BboxArray,Bbox

pub=rospy.Publisher("up_detection_result",BboxArray,queue_size=10)#publish to channel"up_detection_result"
rospy.init_node("up_camera_detection",anonymous=True)#define node name


#publish message
def talker():
    bboxes=[[2,2,60,60,0],[30,30,99,99,1]]
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_bboxarray=BboxArray()
        pub_bboxes=[]
        for bbox in bboxes:
            pub_bbox=Bbox()
            pub_bbox.x_top_left=bbox[0]
            pub_bbox.y_top_left=bbox[1]
            pub_bbox.x_bottom_right=bbox[2]
            pub_bbox.y_bottom_right=bbox[3]
            pub_bbox.label=bbox[4]
            pub_bboxes.append(pub_bbox)
        pub_bboxarray.header.stamp=rospy.Time.now()
        pub_bboxarray.bboxes=pub_bboxes
        pub.publish(pub_bboxarray)
        rate.sleep()


if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
	pass

