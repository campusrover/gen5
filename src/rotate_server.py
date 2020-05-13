#! /usr/bin/env python
import rospy
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from gen5.msg import RotateAction, RotateGoal, RotateResult
from nav_msgs.msg import Odometry

# Action Request Comes in
def do_rotate(goal):
    global z_rotation, z_rotation_turned, rate
    z_rotation_turned = False

    # Converting from angles to radians
    rotation_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rotate_msg = Twist()

    relative_angle = math.radians(goal.degrees_to_rotate)

    # Initialize starting angle
    start_angle = z_rotation
    target_angle = start_angle + relative_angle
    if target_angle > math.radians(180):
        target_angle -= math.radians(360)

    # Verify success condition
    if z_rotation < target_angle:
        while z_rotation < target_angle:
            rotate_msg.angular.z = goal.angular_velocity
            rotation_publisher.publish(rotate_msg)
            rate.sleep()
    else:
        while z_rotation < target_angle or z_rotation_turned == False:
            rotate_msg.angular.z = goal.angular_velocity
            rotation_publisher.publish(rotate_msg)
            rate.sleep()
        
    # Stops the robot once exit condition is met
    rotate_msg.angular.z = 0
    rotation_publisher.publish(rotate_msg)
    result = RotateResult()
    server.set_succeeded()

# Get current orientation
def odometryCb(msg):
    #Euler from Quaternion angles
    global roll, pitch, yaw, z_rotation, z_rotation_turned
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #Detect if yaw has gone full circle
    if yaw + math.radians(180) < z_rotation :
        z_rotation_turned = True
    z_rotation = yaw

# Declare that we are a node
rospy.init_node('rotate_server')
rospy.Subscriber('odom', Odometry, odometryCb)

z_rotation = 0
z_rotation_turned = False
rate = rospy.Rate(10)

# Declare that this node will handle actions
# When action requests come in, call do_timer method
server = actionlib.SimpleActionServer('rotate', RotateAction, do_rotate, False)

# Start it up
server.start()

# Wait until ^c
rospy.spin()