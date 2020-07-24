#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from port_serial.msg import DumpsterCAN
from yolo_detect_pkg.msg import BBoxArray
#转换中文需要用到
from PIL import Image, ImageDraw, ImageFont
import numpy

def updetection_callback(data):
    global frame
    global character
    global state
    global persion
    persion_bbox=[]
    dumpster_bbox=[]
    bboxes=data.bboxes
    labels=data.labels
    for num in range(len(bboxes)):
        if labels[num]==0:#label 0:代表行人
            persion_bbox.append([bboxes[num].x_top_left,bboxes[num].y_top_left,
                                 bboxes[num].x_bottom_right,bboxes[num].y_bottom_right])
        if labels[num]==1:#label 1:代表垃圾桶
            dumpster_bbox.append([bboxes[num].x_top_left, bboxes[num].y_top_left,
                                 bboxes[num].x_bottom_right, bboxes[num].y_bottom_right])
    if state==0:
        if len(dumpster_bbox)!=0:#如果收到垃圾桶检测框的信息

            frame = frame #画出检测框
            character = "检测出垃圾桶，请将垃圾桶位于目标区，并停车！"
            return
        else:
            return

    if state==1:
        if len(dumpster_bbox)!=0:#如果收到垃圾桶检测框的信息
            frame=frame#画出检测框
            if len(persion_bbox)!=0:#如果检测出行人
                persion=1
                return
            return
        else:
            return

    if state==2:
        if len(dumpster_bbox)!=0:#如果收到垃圾桶检测框的信息
            frame=frame#画出检测框
            return
        else:
            return

def charge_callback(data):
    global state
    global character
    #如果收到开始作业信号
    state=1
    character="开始作业！"
    #如果收到作业停止信号
    state=2
    global gongkuang
    if gongkuang==0:#表示只有一个垃圾桶
        character="垃圾桶全部被处理完成！"
    if gongkuang==1:#表示有连续垃圾桶
        character="仍有垃圾桶未被处理！"

def perception_callback(data):
    global perception_listen
    global gongkuang
    if perception_listen==1:#接收到驻车信号后，会将其置为1,这时我们只查看感知信号中的连续工况还是单独工况
        if data[0]==0:#连续工况
            gongkuang=1
            perception_listen=0
            return
        if data[0]==1:#单独工况
            gongkuang=0
            perception_listen=0
            return
    else:
        return

def zhuche_callback(data):
    global zhuche_signal
    global perception_listen
    if data_stop:#如果收到了驻车信号
        zhuche_signal=1
        perception_listen=1
        return
    else:
        return


def speed_callback(data):
    global state
    global zhuche_signal
    if data.data:#如果速度超过一定阈值True
        state=0
        zhuche_signal=0
        return
    else:
        return

def listener():
    global character
    global zhuche_signal
    global state

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/yolo_detect/bbox_results", BBoxArray, updetection_callback)#实时接收上摄像头的视频以及检测结果信号

    if character=="检测出垃圾桶，请将垃圾桶位于目标区，并停车!":
        rospy.Subscriber("stop_signal", String, zhuche_callback)#接收驻车信号
        rospy.Subscriber("detection_signal", String, perception_callback)#接收感知信号

    if zhuche_signal==1:
        rospy.Subscriber("charge_signal",String,charge_callback)#控制器信号

    if state == 2:
        rospy.Subscriber("opflow_static", Bool, speed_callback)#速度信号

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def frame_ch(frame,text,x=100,y=100,text_size=20,text_color=(255,0,0),fontText_path="C:\\Windows\\Fonts\\simsun.ttc"):
    if (isinstance(frame, numpy.ndarray)):#判断是否OpenCV图片类型
        frame = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(frame)
        fontText = ImageFont.truetype(fontText_path, text_size, encoding="utf-8")
        draw.text((x,y), text, text_color, font=fontText)
        frame=cv2.cvtColor(numpy.array(frame), cv2.COLOR_RGB2BGR)
        return frame
    else:
        raise ValueError("type(frame) is not numpy.ndarray")#这里有待进一步确认

if __name__ == '__main__':
    persion=0#0为无行人，1为有行人
    state=0#0为行驶阶段，1为工作阶段，2为衔接阶段
    gongkuang=0#区别连续作业或单独作业
    character=None#输出文字显示
    zhuche_signal=0
    perception_listen=0
    vc = cv2.VideoCapture(0)
    if vc.isOpened():
        rval, frame = vc.read()
    while rval:#判断是否有图片
        if persion==1:
            state=3
        rval, frame = vc.read()
        listener()#接收上摄像头的检测结果
        if state==3:
            character="危险！注意行人！"
        frame=frame_ch(frame,character)
        cv2.imshow("capture", frame)
        if cv2.waitKey(10) == ord("1"):  # 每个图片延时10ms,输入1退出
            break
    vc.release()
    cv2.destroyAllWindows()
