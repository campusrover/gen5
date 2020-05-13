#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy 
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray, FiducialArray
import math

# initialize node
rospy.init_node('fid_nav')

# fiducial transform array callback
def fid_mapper(msg):

    # if there are fiducials to use to navigate
    if len(msg.transforms) > 0:

        # send goal fid (highest fiducial id tag) to nav method
        tf_goal = max(msg.transforms, key=lambda x: x.fiducial_id)
        nav_to_fid(tf_goal)

# send cmd_vel to navigate to current fiducial
def nav_to_fid(tf_msg):

    # use translation and rotation to fiducial to determine
    # twist to pbulish
    translation = tf_msg.transform.translation
    rot = tf_msg.transform.rotation
    distance_to_goal = (translation.x **2 + translation.z ** 2) ** 0.5
    if distance_to_goal > 1:

        # calculate angle in degrees to fiducial
        # print angle for now
        # keep working on using angle to determine if 
        # vehicle is on right direction to fiducial
        a = translation.x
        b = translation.z
        c = (a** 2 + b**2) ** 0.5
        angle = math.acos(b/c)
        print(str(angle * 180 / math.pi))
        if angle > 85 and angle < 95:
            t = Twist()
            t.linear.x = 0.2
            cmd_vel_pub.publish(t)
        else:
            t = Twist()
            t.angular.z = 0.2
            cmd_vel_pub.publish(t)
        
        

fid_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, fid_mapper)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# update at 10 hz
rate = rospy.Rate(10)

# wait for callbacks
rospy.spin()
