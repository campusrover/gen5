#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back')

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def timed_action(start, duration, twist, rate):
    while time.time() - start < duration:
        print time.time() - start
        cmd_vel_pub.publish(forward_command)
        rate.sleep()

run_time = 15.0
distance = 3
rotate_time = 10.0
rotate_angle = 180
rate = rospy.Rate(30)

forward_command = Twist()
forward_command.linear.x = distance / run_time

rotate_command = Twist()
rotate_command.angular.z = math.radians(rotate_angle) / rotate_time

start = time.time() # reset time
timed_action(start, run_time, forward_command, rate) # Moves straight

start = time.time() # reset time
timed_action(start, rotate_time, forward_command, rate) # Rotate

start = time.time() # reset time
timed_action(start, run_time, forward_command, rate) # Moves straight

start = time.time() # reset time
timed_action(start, rotate_time, forward_command, rate) # Rotate

cmd_vel_pub.publish(Twist()) # stops the robot

rospy.spin()