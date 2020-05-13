#!/usr/bin/env python
import rospy
import actionlib
import math
from gen5.msg import RotateAction, RotateGoal, RotateResult
from gen5.msg import RunAction, RunGoal, RunResult
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back')

rate = rospy.Rate(30)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def rotate(rotate_client, rotate_angle, rotate_time):

    goal = RotateGoal
    goal.degrees_to_rotate = rotate_angle
    goal.angular_velocity = abs(math.radians(rotate_angle) / rotate_time)

    rotate_client.send_goal(goal)
    # Wait for confirmation
    rotate_client.wait_for_result()

def run(run_client, distance, run_time):

    goal = RunGoal
    goal.distance = distance
    goal.linear_velocity = distance / run_time

    run_client.send_goal(goal)
    # Wait for confirmation
    run_client.wait_for_result()

def stop_robot():
    cmd_vel_pub.publish(Twist()) # stops the robot

run_time = 6.0
distance = 1
rotate_time = 5.0
rotate_angle = 180

rotate_client = actionlib.SimpleActionClient('rotate', RotateAction)
rotate_client.wait_for_server()

run_client = actionlib.SimpleActionClient('run', RunAction)
run_client.wait_for_server()


run(run_client, distance, run_time)

rotate(rotate_client, rotate_angle, rotate_time)

run(run_client, distance, run_time)

rotate(rotate_client, rotate_angle, rotate_time)

stop_robot()

rospy.spin()