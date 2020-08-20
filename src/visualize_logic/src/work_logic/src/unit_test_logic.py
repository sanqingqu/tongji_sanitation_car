#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
#subscribe up_image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as IMAGE
#subscribe up_camera_detection
from up_camera_detection.msg import BboxArray,Bbox
#subscribe control signal##0:start work;1:stop work
from control_signal.msg import Signal
#subscribe speed_detection##True:static;False:dynamic
from speed_detection.msg import Speedflag
#subscribe stop_signal
from stop_signal.msg import Stopflag
#subscribe perception_signal
from perception_signal.msg import Perceptionflag
#draw chinese on image
from PIL import Image, ImageDraw, ImageFont
import numpy
#visualize
import cv2


def updetection_callback(data):
    global frame
    global character
    global state
    global persion
    bboxes=data.bboxes
    persion_bbox=[]
    dumpster_bbox=[]
    
    if len(bboxes)==0:
	return 0
    else:
    	for num in range(len(bboxes)):
            if bboxes[num].label==0:#label 0:代表行人
                persion_bbox.append([bboxes[num].x_top_left,bboxes[num].y_top_left,
                                 bboxes[num].x_bottom_right,bboxes[num].y_bottom_right])
            elif bboxes[num].label==1:#label 1:代表垃圾桶
                dumpster_bbox.append([bboxes[num].x_top_left, bboxes[num].y_top_left,
                                 bboxes[num].x_bottom_right, bboxes[num].y_bottom_right])
    if state==0:
        if len(dumpster_bbox)!=0:#如果收到垃圾桶检测框的信息
            character = "检测出垃圾桶，请将垃圾桶位于目标区，并停车！"

    if state==1:
        if len(persion_bbox)!=0:#如果检测出行人
            persion=1

    if state==2:
        pass
    
    #draw rectangle on image
    if dumpster_bbox:
	for bbox in dumpster_bbox:
            cv2.rectangle(frame,(bbox[0],bbox[1]),(bbox[2],bbox[3]),(0,0,255),2)
    if persion_bbox:
	for bbox in persion_bbox:
            cv2.rectangle(frame,(bbox[0],bbox[1]),(bbox[2],bbox[3]),(255,0,0),2) 
     
    

def controlsignal_callback(data):
    global state
    global character
    global gongkuang
    current_signal=data.signal
    if current_signal==0:
    	state=1
    	character="开始作业！"
    if current_signal==1:
    	state=2
        if gongkuang==0:#表示只有一个垃圾桶
            character="垃圾桶全部被处理完成！"
        if gongkuang==1:#表示有连续垃圾桶
            character="仍有垃圾桶未被处理！"


def perception_callback(data):
    global perception_listen
    global gongkuang
    flag=data.flag
    if perception_listen==1:#接收到驻车信号后，会将其置为1,这时我们只查看感知信号中的连续工况还是单独工况
        if flag==1:#连续工况
            gongkuang=1
            perception_listen=0
        elif flag==0:#单独工况
            gongkuang=0
            perception_listen=0


def zhuche_callback(data):
    global zhuche_signal
    global perception_listen
    stop_flag=data.flag
    if stop_flag==1:#如果收到了驻车信号
        zhuche_signal=1
        perception_listen=1


def speedflag_callback(data):
    global state
    global zhuche_signal
    speedflag=data.flag
    if not speedflag:#dynamic
        state=0
        zhuche_signal=0


def upimage_callback(data):
    global character
    global zhuche_signal
    global state
    global frame
    global persion
    bridge=CvBridge()
    frame=bridge.imgmsg_to_cv2(data)
    rospy.Subscriber("up_detection_result", BboxArray, updetection_callback)#实时接收检测结果信号
    if character=="检测出垃圾桶，请将垃圾桶位于目标区，并停车!" and zhuche_signal==0:
        rospy.Subscriber("stop_signal_result", Stopflag, zhuche_callback)#接收驻车信号
        rospy.Subscriber("perception_signal_result", Perceptionflag, perception_callback)#接收感知信号
    if zhuche_signal==1:
        rospy.Subscriber("control_signal_result",Signal,controlsignal_callback)#控制器信号
    if state == 2:
        rospy.Subscriber("speed_detection_result", Speedflag, speedflag_callback)#速度信号
    
    if persion==1:
        character="危险！注意行人！"

    if character:
	frame=frame_chinese(frame,character.decode("utf-8"))

    cv2.imshow("img",frame)
    cv2.waitKey(10)


def listener():
    global character
    global zhuche_signal
    global state

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("up_image", IMAGE, upimage_callback)#实时接收上摄像头的视频

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

    frame=None#image show
    persion=0#0为无行人，1为有行人
    state=0#0为行驶阶段，1为工作阶段，2为衔接阶段
    gongkuang=0#区别连续作业或单独作业
    character=None#输出文字显示
    zhuche_signal=0
    perception_listen=0
    
    listener()    

    cv2.destroyAllWindows()
