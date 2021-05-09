#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

roll, pitch, yaw = 0, 0, 0


DX_KP=1.2
DX_KI=2.5
DX_KD=3.2
DX_err=0
DX_err_prev=0

YAW_KP=1.2
YAW_KI=2.5
YAW_KD=3.2
YAW_err=0
YAW_err_prev=0


run_time=0.001
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

def DX_PID_CONTROL():
	P_CON=DX_KP*DX_err
	I_CON=DX_KI*DX_err*run_time
	D_CON=DX_KD*(DX_err-DX_err_prev)/run_time
	return P_CON+I_CON+D_CON

def YAW_PID_CONTROL():
	P_CON=YAW_KP*YAW_err
	I_CON=YAW_KI*YAW_err*run_time
	D_CON=YAW_KD*(YAW_err-YAW_err_prev)/run_time
	return P_CON+I_CON+D_CON

def RE_PARKING():
	parking_time=700	
	while parking_time>0:	
		angle=0
		speed=-5		
		parking_time=parking_time-1
    		xycar_msg.data = [angle, speed]
    		motor_pub.publish(xycar_msg)
	


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

#move using dx
    speed=15
    DX_err=arData["DX"]
    YAW_err=round(yaw,1)

#brake when it is parked


    if arData["DY"]<=70:
	speed=0
    	if arData["DY"]<34:
		RE_PARKING()
	else:
		pass
	
    else:
	pass



    angle=DX_PID_CONTROL()-YAW_PID_CONTROL()*6
    if YAW_err>=15 or YAW_err<=-15:
	angle=angle
    else:
	pass
    print(angle)
    
    

    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
    DX_err_prev=DX_err
    YAW_err_prev=YAW_err


cv2.destroyAllWindows()



