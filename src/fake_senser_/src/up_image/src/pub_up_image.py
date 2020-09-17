#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

pub=rospy.Publisher("up_image",Image,queue_size=10)#publish to channel"up_image"
rospy.init_node("pub_up_camera_image",anonymous=True)#define node name
bridge=CvBridge()

#img=cv2.imread("/home/neousys/Downloads/image.jpg")
#pub.publish(bridge.cv2_to_imgmsg(img))
#cv2.imshow("img",img)
#cv2.waitKey(0)

vc=cv2.VideoCapture(0)


#publish message
def talker():
    rate=rospy.Rate(50)
    while not rospy.is_shutdown():
	if vc.isOpened():
	    rval,frame=vc.read()
            pub.publish(bridge.cv2_to_imgmsg(frame)) 
	    #cv2.imshow("img",frame)
	    #cv2.waitKey(10)       
            rate.sleep()
        else:
   	    return -1


if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
	pass

