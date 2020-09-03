#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import tf2_ros
import math
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Bool

x = None
y = None
z = None
x_p = 0
y_p = 0
cnt = 0

tool_clear = None
tool_mode = None
stop_motor = 0

theta = None
r = None

step_check_l = None
step_check_r = None
step_left_true = False
step_right_true = False

def TF_callback(msg):
    global x, y, z, x_p, y_p
    if msg.transforms[0].child_frame_id == "torso":
        x = msg.transforms[0].transform.translation.x
        y = msg.transforms[0].transform.translation.y
        x = x-1.3
        x = ((x * 0.1)+(x_p * 0.9)) / 2
        x_p = x

        y = ((y * 0.1)+(y_p * 0.9)) / 2
        y_p = y

def Scan_callback(detect):
    global left, right, back
    left = detect.data[0]
    right = detect.data[1]
    back = detect.data[2]
    
def tool_mode(tool_select):
    global tool_clear, tool_mode, stop_motor
    #도구 선택이 감지되면 모터를 정지하기 위한 설정
    tool_clear = tool_select.data[0] #clear 받기
    tool_mode = tool_select.data[1] #tool_mode 받기
    if tool_clear == 1 and tool_mode == 1:
        stop_motor = 1
    else:
        stop_motor = 0

def step_motor_left(stepOK_left):
    global step_left_true
    step_check_l = stepOK_left.data # 데이터 받아오고 Ready 확인 변수
    if step_check_l == "Ready_left":
        step_left_true = True # 준비가 되면 true

def step_motor_right(stepOK_right):
    global step_right_true
    step_check_r = stepOK_right.data # 데이터 받아오고 Ready 확인 변수
    if step_check_r == "Ready_right":
        step_right_true = True # 준비가 되면 true

rospy.Subscriber('tf', TFMessage, TF_callback)
rospy.Subscriber('obstacle', Float32MultiArray, Scan_callback)
rospy.Subscriber('tool', Float32MultiArray, tool_mode)
rospy.Subscriber('stepOK_left', String, step_motor_left)
rospy.Subscriber('stepOK_right', String, step_motor_right)


if __name__ == '__main__':
    pub = rospy.Publisher("GoTobldc", Float32MultiArray, queue_size = 10)
    rospy.init_node('main_bldc')

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if step_left_true == True and step_right_true == True:
            if x is not None and y is not None:
                r = math.sqrt(float(x**2) + float(y**2))

                if stop_motor == 0:
                    if back == 0:
                        cnt = 0
                        if x < -0.02:
                            r = (-1) * r
                        
                        elif x > 0.02:
                            r = r

                        else:
                            r = 0

                        GoTobldc = Float32MultiArray()
                        bldc_v = round(r * 1000)
                        GoTobldc.data = [bldc_v, bldc_v, bldc_v, bldc_v]
                        pub.publish(GoTobldc)
                    
                    elif back == 1:
                        if cnt < 9:
                            GoTobldc = Float32MultiArray()
                            GoTobldc.data = [0,0,0,0]
                            pub.publish(GoTobldc)

                        cnt += 1

                        if cnt > 10:
                            print(y)
                            if y < -0.02:
                                r = -1 * r
                            
                            elif y > 0.02:
                                r = r
                            
                            else:
                                r = 0

                            bldc_v = round(r * 1000)
                            GoTobldc.data = [bldc_v,bldc_v,bldc_v,bldc_v]
                            pub.publish(GoTobldc)

                elif stop_motor == 1:
                    # 도구선택 모드 작동
                    GoTobldc = Float32MultiArray()
                    bldc_v = 0
                    GoTobldc.data = [0,0,0,0]
                    pub.publish(GoTobldc)
        time.sleep(0.1)