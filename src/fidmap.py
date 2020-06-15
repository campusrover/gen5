#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy 
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray, FiducialArray
import math

# initialize node
rospy.init_node('fid_nav')

# FSM model for states
possible_states = ["NO_FID", "FINDING_FID_PATH", "NAV_TO_FID", "DONE"]
current_state = possible_states[0]
final_fiducial_id = 1

# odom updates
last_odom = None

# fiducial transform array callback
def fid_mapper(msg):
    global current_state
    print(current_state)

    # if there are fiducials to use to navigate
    if len(msg.transforms) > 0:

        # send goal fid (highest fiducial id tag) to nav method
        current_state = possible_states[1]
        tf_goal = max(msg.transforms, key=lambda x: x.fiducial_id)
        nav_to_fid(tf_goal)
    else:
        current_state = possible_states[0]
        t = Twist()
        t.angular.z = 0.4
        cmd_vel_pub.publish(t)

# send cmd_vel to navigate to current fiducial
def nav_to_fid(tf_msg):

    # use translation and rotation to fiducial to determine
    translation = tf_msg.transform.translation
    rotation = tf_msg.transform.rotation
    global min_distance

    # check that distance to goal > some distance and that we have odometry information
    distance_to_goal = (translation.x **2 + translation.z ** 2) ** 0.5
    if distance_to_goal > 0.75 and last_odom:

        # convert rotation quaternion to roll, pitch, yaw in degrees
        roll, pitch, yaw = rotatation_to_rpy(rotation)

        # distance from camera to fid
        fid_distance = (translation.x ** 2 + translation.z ** 2) ** 0.5

        # save minimum distance to fiducial as the odometry twist.angular.z
        if fid_distance < min_distance[0]:
            min_distance = (fid_distance, last_odom.angular.z * 180 / math.pi)

        # use tolerance to check when robot should switch states
        tolerance = 0.1
        global current_state; global final_fiducial_id
        if current_state != possible_states[3] and last_odom.angular.z * 180 / math.pi < min_distance[1] + tolerance and last_odom.angular.z * 180 / math.pi > min_distance[1] - tolerance:
            current_state = possible_states[2]

        # give cmd_vel based on FSM
        if current_state == possible_states[1]:
            t = Twist()
            t.angular.z = 0.4
            cmd_vel_pub.publish(t)
        elif current_state == possible_states[2]:
            t = Twist()
            t.linear.x = 0.2
            cmd_vel_pub.publish(t)
        elif tf_msg.fiducial_id == final_fiducial_id:
            current_state = possible_states[3]
            cmd_vel_pub.publish(Twist())

# convert rotation quaternion to roll, pitch, yaw in degrees
def rotatation_to_rpy(rot):
    quat = [rot.x, rot.y, rot.z, rot.w]
    r, p, y = euler_from_quaternion(quat)
    return (r * 180 / math.pi, p * 180 / math.pi, y * 180 / math.pi)

# odom callback to save recent yaw values
def odom_cb(msg):
    global last_odom
    last_odom = msg.twist.twist
        
 # keep track of distances to fiducial
min_distance = (9999999, None)
last_odom = None      

# sub/pubs
fid_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, fid_mapper)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)

# update at 10 hz
rate = rospy.Rate(10)

# wait for callbacks
rospy.spin()