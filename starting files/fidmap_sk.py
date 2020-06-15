#!/usr/bin/env python
import rospy
import math
import numpy 
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray, FiducialArray

# initialize node
rospy.init_node('fid_nav')

# fiducial transform array callback
# start writing code here to look at transform
# between camera on robot and fiducial
# you can also publish cmd_vel's from here 
def fid_mapper(msg):
    print(msg)

# sub/pubs
# feel free to subscribe to additional topics /odom /tf etc.
fid_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, fid_mapper)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# update at 10 hz
rate = rospy.Rate(10)

# wait for callbacks
rospy.spin()