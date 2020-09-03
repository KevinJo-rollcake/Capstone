#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import tf2_ros
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from copy import copy
import time

back = 1
left = 1
right = 1

left_chk = 0
right_chk = 0
back_chk = 0

def Scan_callback(msg):
    global back, left, right, back_chk, right_chk, left_chk

    back_area = msg.ranges[150:210]

    for i in range(len(back_area)):
        if (back_area[i] != 0):
            back = back_area[i]
            if back < 0.6:
                back_chk = 1
            else:
                back_chk = 0 
        

rospy.Subscriber('scan', LaserScan, Scan_callback)

if __name__ == '__main__':
    pub = rospy.Publisher("obstacle",Float32MultiArray,queue_size = 10)
    rospy.init_node("lidar_node")
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown(): 
        detect = Float32MultiArray()
        detect.data = [left_chk,right_chk,back_chk]
        pub.publish(detect)
        #print(detect)
        back = 1
        #left = 1
        #right = 1

        time.sleep(0.1)