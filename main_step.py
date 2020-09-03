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

out = 0

tool_clear = None
tool_mode = None
stop_motor = 0

left = None
right = None
back = None

theta = None
r = None
tstep_p = 0

step_check_l = None
step_check_r = None
step_left_true = None
step_right_true = None

def TF_callback(msg):
    global x, y, z, x_p, y_p
    if msg.transforms[0].child_frame_id == "torso":
        x = msg.transforms[0].transform.translation.x
        y = msg.transforms[0].transform.translation.y
        x = x-1.3
        x = ((x*0.1) + (x_p*0.9))/2
        x_p = x

        y = ((y*0.1) + (y_p*0.9))/2
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
    step_check_l = stepOK_left.data
    if step_check_l == "Ready_left":
        step_left_true = True
    
def step_motor_right(stepOK_right):
    global step_right_true
    step_check_r = stepOK_right.data
    if step_check_r == "Ready_right":
        step_right_true = True

rospy.Subscriber('tf', TFMessage, TF_callback)
rospy.Subscriber('obstacle', Float32MultiArray, Scan_callback)
rospy.Subscriber('tool', Float32MultiArray, tool_mode)
rospy.Subscriber('stepOK_left', String, step_motor_left)
rospy.Subscriber('stepOK_right', String, step_motor_right)

# 메인함수
if __name__ == '__main__':
    pub = rospy.Publisher("GoTostep",Float32MultiArray,queue_size = 10)
    pub2 = rospy.Publisher("stepstart", Bool, queue_size = 10)
    rospy.init_node('main_step')

    stepstart = Bool()
    stepstart.data = True
    time.sleep(1)

    pub2.publish(stepstart)
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if step_left_true == True and step_right_true:
            if x is not None and y is not None:
                theta = math.atan2(float(y),float(x))
                tstep = theta * (180/math.pi) # radian을 degree로
                if stop_motor == 0:
                    if back == 0:
                        if abs(tstep-tstep_p) > 3:
                            tstep_p = tstep # 전 값과 비교하여서 변화가 3도 이상인 경우만 전송
                            # 1펄스 = 곱하기 1.8, round는 소수점 잘라내기
                            step_theta = round(tstep)

                            if x < -0.02:
                                if step_theta > 0 :
                                    step_theta = (-1)*(180-step_theta)

                                else:
                                    step_theta = (step_theta+180)
                                
                            elif x > 0.02:
                                step_theta = step_theta

                            elif step_theta > 240 and x > 0:
                                # 240도를 넘어가면 그냥 240도 값 계속 전송
                                step_theta = 240

                            elif step_theta < -240 and x > 0:
                                # -240도를 넘어가면 그냥 -240도 값 계속 전송
                                step_theta = -240

                            else:
                                step_theta = 0

                            GoTostep_arr = Float32MultiArray()
                            GoTostep_arr.data = [step_theta, step_theta, step_theta, step_theta]
                            pub.publish(GoTostep_arr)

                    elif back == 1:
                        GoTostep_arr = Float32MultiArray()
                        step_theta = 245

                        GoTostep_arr.data = [step_theta, step_theta, step_theta, step_theta]
                        pub.publish(GoTostep_arr)
                    
                else:
                    # 도구선택 모드 작동
                    GoTostep_arr = Float32MultiArray()
                    GoTostep_arr.data = [0, 0, 0, 0]
                    pub.publish(GoTostep_arr)
        time.sleep(0.1)