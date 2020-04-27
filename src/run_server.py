#! /usr/bin/env python
import rospy
import actionlib
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gen5.msg import RunAction, RunGoal, RunResult
from nav_msgs.msg import Odometry

# Action Request Comes in
def do_run(goal):

    global start_position, current_position
    
    run_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    run_msg = Twist()
    distance = goal.distance
    run_msg.linear.x = goal.linear_velocity

    # Verify success condition
    while not verify_success(start_position, current_position, distance):
        run_publisher.publish(run_msg)
        rate.sleep()
        
    # Stops the robot once exit condition is met
    run_msg.linear.x = 0
    run_publisher.publish(run_msg)
    result = RunResult()
    server.set_succeeded()

# Get current orientation
def odometryCb(msg):
    #Euler from Quaternion angles
    global current_position
    current_position = msg.pose.pose.position

def verify_success(start_position, current_position, distance):
    current_distance = euclidean_distance(start_position, current_position)
    if  current_distance >= distance:
        return True
    return False

def euclidean_distance(start_position, current_position):
    return math.sqrt(math.pow(current_position.x - start_position.x ,2) 
        + math.pow(current_position.y - start_position.y ,2) 
        + math.pow(current_position.z - start_position.z ,2))


# Declare that we are a node
rospy.init_node('run_server')
rospy.Subscriber('odom', Odometry, odometryCb)

start_position = Point()
current_position = Point()
rate = rospy.Rate(10)

# Declare that this node will handle actions
# When action requests come in, call do_timer method
server = actionlib.SimpleActionServer('rotate', RotateAction, do_rotate, False)

# Start it up
server.start()

# Wait until ^c
rospy.spin()