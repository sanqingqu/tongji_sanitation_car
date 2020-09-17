#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

def call_back(data):
    #print(data.header.stamp)
    bridge=CvBridge()
    img=bridge.imgmsg_to_cv2(data)
    cv2.imshow("img",img)
    cv2.waitKey(50)
    return 1


rospy.init_node("sub_up_camera_image",anonymous=True)#define node name
rospy.Subscriber("up_image",Image,call_back)#subscribe to channel"up_image"

rospy.spin()

#shut down windows when end
cv2.destroyAllWindows()

