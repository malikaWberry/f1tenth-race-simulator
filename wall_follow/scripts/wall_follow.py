from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
dt = 0.1

kp = 1
kd = 0.04
ki = 0.03
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
error_integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_LEFT = 0.8

VELOCITY = 3 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = 'nav'
        self.initialized = False

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) 
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) 

        self.angle_min = 0 
        self.angle_max = 0
        self.angle_increment = 0

        self.angle_theta = 1.0472 # 60 degree
        self.angle_b = 1.571 # 90 degree
        self.angle_a  = self.angle_b- self.angle_theta
        
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        if angle > self.angle_max or angle < self.angle_min or self.angle_increment == 0:
            return -1
        return  data.ranges[int((angle-self.angle_min)/self.angle_increment)]

    def pid_control(self, error):
        global error_integral
        global prev_error
        global kp
        global ki
        global kd
        global dt
        global VELOCITY
        error_derivative  = (error-prev_error)/dt
        prev_error = error

        angle = error*kp + error_derivative*kd 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -1*angle
        drive_msg.drive.speed = VELOCITY 
        self.drive_pub.publish(drive_msg)
    
    def followLeft(self, data, left_dist):
        global dt
        global VELOCITY
        range_a = self.getRange(data, self.angle_a) 
        range_b = self.getRange(data, self.angle_b) 
        angle_alpha = math.atan((range_b-range_a*math.cos(self.angle_theta))/(range_a*math.sin(self.angle_theta)))
        l = VELOCITY * dt 
        d_p = range_b*math.cos(angle_alpha) - l*math.sin(angle_alpha)
        return left_dist - d_p

    def lidar_callback(self, data):
        global DESIRED_DISTANCE_LEFT
        if self.initialized == False:
            self.angle_min = data.angle_min
            self.angle_max = data.angle_max 
            self.angle_increment = data.angle_increment 
            self.initialized = True
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        self.pid_control(error)

def main(args):
    global dt
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)