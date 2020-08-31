#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
#subscribe up_image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
#subscribe up_camera_detection
from deepstream_vision.msg import BboxArray,Bbox
#subscribe perception_signal
#from perception_signal.msg import Perceptionflag
from camera_lidar_fusion_detection.msg import DumpsterInfo
#subscribe speed_detection##True:static;False:dynamic
#from speed_detection.msg import Speedflag

#subscribe control signal##0:start work;1:stop work
from control_signal.msg import Signal

#subscribe stop_signal
#from stop_signal.msg import Stopflag
from std_msgs.msg import Bool

#draw chinese on image
from PIL import Image, ImageDraw, ImageFont
import numpy

#visualize
import cv2


def updetection_callback(data):
    global character
    global state
    #global persion
    global frame
    if frame is None:
	return 

    bboxes=data.bboxes
    persion_bboxes=[]
    dumpster_bboxes=[]
    
    if len(bboxes)==0:
	return

    for num in range(len(bboxes)):
        if bboxes[num].class_id==0:#label 0:代表行人
            persion_bboxes.append([bboxes[num].x_top_left,bboxes[num].y_top_left,
                                 bboxes[num].x_bottom_right,bboxes[num].y_bottom_right])
        elif bboxes[num].class_id==1:#label 1:代表垃圾桶
            dumpster_bboxes.append([bboxes[num].x_top_left, bboxes[num].y_top_left,
                                 bboxes[num].x_bottom_right, bboxes[num].y_bottom_right])
    
    #show caution on image
    if state==0:
        if len(dumpster_bboxes)!=0:#如果收到垃圾桶检测框的信息
            character = "检测出垃圾桶，请将垃圾桶位于目标区，并停车！"

    if state==1:
        if len(persion_bboxes)!=0:#如果检测出行人
	    character="危险！注意行人！"
            #persion=1

    if state==2:
        pass
     
    

def controlsignal_callback(data):
    global state
    global character
    if current_signal==0:
    	state=1
    	character="开始作业！"
    if current_signal==1:
    	state=2
        if continuous_dumpster==0:#表示只有一个垃圾桶
            character="垃圾桶全部被处理完成！"
        if continuous_dumpster==1:#表示有连续垃圾桶
            character="仍有垃圾桶未被处理！"


def perception_callback(data):
    global continuous_dumpster
    global character
    global parking_signal
    if character=="检测出垃圾桶，请将垃圾桶位于目标区，并停车!" and parking_signal==1:
        continuous_dumpster=data.continuous_dumpster


#def zhuche_callback(data):
#    global zhuche_signal
#    global perception_listen
#    global character
#    if character=="检测出垃圾桶，请将垃圾桶位于目标区，并停车!" and zhuche_signal==0:
#    	stop_flag=data.flag
#    	if stop_flag==1:#如果收到了驻车信号
#            zhuche_signal=1
#            perception_listen=1


def speed_callback(data):
    global state
    global parking_signal
    if state == 0:
	if data:#true:static
	    parking_signal=1		

    if state == 2:
        if not data:#false:dynamic
            parking_signal=0
            state=0


def upimage_callback(data):
    global character
    #global persion
    global dumpster_bboxes
    global persion_bboxes
    bridge=CvBridge()
    frame=bridge.compressed_imgmsg_to_cv2(data)

    #draw rectangle on image
    if dumpster_bboxes:
	for bbox in dumpster_bboxes:
            cv2.rectangle(frame,(bbox[0],bbox[1]),(bbox[2],bbox[3]),(0,0,255),2)
    if persion_bboxes:
	for bbox in persion_bboxes:
            cv2.rectangle(frame,(bbox[0],bbox[1]),(bbox[2],bbox[3]),(255,0,0),2) 
    
    #show caution on image
    if character:
	frame=frame_chinese(frame,character.decode("utf-8"))

    cv2.imshow("img",frame)
    cv2.waitKey(10)


def listener():
    global character
    global parking_signal
    global state

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/undistort_upper/compressed", CompressedImage, upimage_callback)#实时接收上摄像头的视频
    rospy.Subscriber("/yolo_detect_upper/bbox_results", BboxArray, updetection_callback)#实时接收检测结果信号
    #rospy.Subscriber("stop_signal_result", Stopflag, zhuche_callback)#接收驻车信号
    rospy.Subscriber("/dumpster_detection/dumpster_location", DumpsterInfo, perception_callback)#接收感知信号
    rospy.Subscriber("control_signal_result",Signal,controlsignal_callback)#控制器信号
    rospy.Subscriber("opflow_static", Bool, speed_callback)#速度信号

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def frame_chinese(frame,text,x=100,y=100,text_size=20,text_color=(255,0,0),fontText_path="/home/neousys/Downloads/simsun.ttc"):
    frame = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(frame)
    fontText = ImageFont.truetype(fontText_path, text_size, encoding="utf-8")
    draw.text((x,y), text, text_color, font=fontText)
    frame=cv2.cvtColor(numpy.array(frame), cv2.COLOR_RGB2BGR)
    return frame


if __name__ == '__main__':
    #persion=0#0为无行人，1为有行人
    state=0#0为行驶阶段，1为工作阶段，2为衔接阶段
    frame=None
    dumpster_bboxes=[]
    persion_bboxes=[]
    character=None#输出文字显示
    continuous_dumpster=0#区别连续作业或单独作业
    parking_signal=0
    
    listener()    

    cv2.destroyAllWindows()
