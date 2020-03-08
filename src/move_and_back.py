#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back')

rate = rospy.Rate(30)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

start = time.time()
run_time = 10
rotate_time = 6
forward_command = Twist()
forward_command.linear.x = 0.15

while time.time() - start < run_time:
    print time.time() - start
    cmd_vel_pub.publish(forward_command)
    rate.sleep()


start = time.time() # reset time
rotate_command = Twist()
rotate_command.angular.z = math.pi / rotate_time

while time.time() - start < rotate_time:
    print time.time() - start
    cmd_vel_pub.publish(rotate_command)
    rate.sleep()


start = time.time() # reset time
while time.time() - start < run_time:
    print time.time() - start
    cmd_vel_pub.publish(forward_command)
    rate.sleep()

cmd_vel_pub.publish(Twist()) # stops the robot


rospy.spin()