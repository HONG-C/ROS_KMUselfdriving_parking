#! /usr/bin/env python
# -*- coding: utf-8 -*-
#parking system using pid control
#PID제어를 이용한 주차 시스템 
import rospy, math
import cv2, time, rospy
import numpy as np
import matplotlib.pyplot as plt

x_axis=[]
y_axis=[]
running_time=0

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

roll, pitch, yaw = 0, 0, 0


def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

class PID_CONTROL():#pid제어를 위한 클래스 

    run_time=0.001
    err=0	#dx값과 yaw값을 0에 근사시켜야 하는 에러라고 지정 
    err_prev=0
    def __init__(self,KP,KI,KD):#pid제어 계수 초기화 
        self.KP=KP
        self.KI=KI
        self.KD=KD

    def pid_val(self,err,err_prev):#pid제어를 실행하는 함수 
        P_CON=self.KP*err
        I_CON=self.KI*err*self.run_time
        D_CON=self.KD*(err-err_prev)/self.run_time
        return P_CON+I_CON+D_CON



rospy.init_node('ar_drive')

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size =1 )

xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    img = np.zeros((100, 500, 3))

    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1)) 
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

#기본 속도 설정 및 클래스 초기화 
    speed=15
    DX=PID_CONTROL(1.2,2.5,3.2)
    YAW=PID_CONTROL(1.2,2.5,3.2)
    DX.err=arData["DX"]
    YAW.err=round(yaw,1)


#주차 시 정지 
    if arData["DY"]<=70:
    	speed=0
    else:
	pass


#dx값과 yaw값을 이용한 pid제어 값 통합 
    angle=DX.pid_val(DX.err,DX.err_prev)-YAW.pid_val(YAW.err,YAW.err_prev)*15

#속도와 조향각 퍼블리싱 
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
    DX.err_prev=DX.err
    YAW.err_prev=YAW.err

#차선 인지 확인을 위한 그래프 작성 
    running_time=running_time+1
    x_axis.append(running_time)
    y_axis.append(round(yaw,1))
    plt.plot(x_axis,y_axis)
    plt.pause(0.0000000001)


cv2.destroyAllWindows()

