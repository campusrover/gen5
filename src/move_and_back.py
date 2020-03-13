#!/usr/bin/env python

import rospy
import actionlib
import time
import math
from gen5.msg import RotateAction, RotateGoal, RotateResult
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back')

rate = rospy.Rate(30)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def forward(parameter_list):
    
start = time.time()
run_time = 6.0
distance = 1
rotate_time = 5.0
rotate_angle = 180
forward_command = Twist()

forward_command.linear.x = distance / run_time

while time.time() - start < run_time:
    print time.time() - start
    cmd_vel_pub.publish(forward_command)
    rate.sleep()

rotateClient = actionlib.SimpleActionClient('rotate', RotateAction)
rotateClient.wait_for_server()
goal = RotateGoal
goal.degrees_to_rotate = 180
goal.angular_velocity = abs(math.radians(rotate_angle) / rotate_time)

rotateClient.send_goal(goal)
# Wait for confirmation
rotateClient.wait_for_result()

start = time.time() # reset time
while time.time() - start < run_time:
    print time.time() - start
    cmd_vel_pub.publish(forward_command)
    rate.sleep()

rotateClient.send_goal(goal)
# Wait for confirmation
rotateClient.wait_for_result()

cmd_vel_pub.publish(Twist()) # stops the robot


rospy.spin()