#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back_odom')

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def run(...):

def rotate(...):

# assign variable values
run_time
distance
rotate_time 
rotate_angle

# initiate action clients
rotate_client
run_client

run(...)

rotate(...)

run(...)

rotate(...)

stop_robot()

rospy.spin()