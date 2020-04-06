#!/usr/bin/env python
import rospy
import actionlib
import math
from gen5.msg import RotateAction, RotateGoal, RotateResult
from gen5.msg import RunAction, RunGoal, RunResult
from geometry_msgs.msg import Twist

rospy.init_node('move_and_back_odom')

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def run(...): # invoke run action
    pass

def rotate(...): # invoke rotate action
    pass

def stop_robot(...): 
    pass

# suggested variable values
run_time = ...
distance = ...
rotate_time = ...
rotate_angle = ...

# initiate action clients
rotate_client
run_client
rotate_client.wait_for_server()
run_client.wait_for_server()

run(...)

rotate(...)

run(...)

rotate(...)

stop_robot()

rospy.spin()