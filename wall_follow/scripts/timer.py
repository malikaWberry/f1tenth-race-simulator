#!/usr/bin/env python3
# timer designed by Xiyuan Zhu at Lehigh University, Mar 3, 2020
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

Moved=False
loop_num=0
prev_pos = 0
secs = 0
nsecs = 0

class Timer():
    def __init__(self):
        rospy.Subscriber("odom", Odometry, self.odom_callback) 
    def odom_callback(self, odom_msg):
        global Moved
        global prev_pos
        global secs
        global nsecs
        if Moved == False:
            speed = odom_msg.twist.twist.linear.x
            if speed > 0.1:
                Moved = True
                print("Movement detected, start timing.")
                secs = odom_msg.header.stamp.secs
                nsecs = odom_msg.header.stamp.nsecs
                prev_pos=0.01
        else:
            if prev_pos<=0:
                pos=odom_msg.pose.pose.position.x
                if pos>0:
                    global loop_num
                    loop_num=loop_num+1
                    end_secs=odom_msg.header.stamp.secs-secs
                    end_nsecs=odom_msg.header.stamp.nsecs-nsecs
                    time=end_secs*1.0+end_nsecs*0.000000001
                    print("Loop",loop_num,":",time,"secs.")
            prev_pos=odom_msg.pose.pose.position.x


def main():
    rospy.init_node("Timer_node")
    Ti=Timer()
    rospy.spin()

if __name__=='__main__':
    main()
