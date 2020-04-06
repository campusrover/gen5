#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back_time')

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def run(...):
    pass

def rotate(...):
    pass

def stop_robot(...):
    pass

# suggested variables
run_time = ...
distance = ...
rotate_time = ...
rotate_angle = ...
rate = ...

start = time.time() # reset time
run(...)

start = time.time() # reset time
rotate(...)

start = time.time() # reset time
run(...)

start = time.time() # reset time
rotate(...)

stop_robot()

rospy.spin()