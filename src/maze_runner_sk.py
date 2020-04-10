#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# scan callback to update longest/shortest range and the index (angle)
# of the longest range in front of the robot
def scan_cb(msg):

# subscriber/publisher declarations
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('maze_runner')

# starting movement parameters
driving_forward = False

# update at 10 hz
rate = rospy.Rate(10)

# control loop
while not rospy.is_shutdown():

    # drive forward or turn accordingly 
    twist = Twist()
    if driving_forward:
        pass
    else:
        pass

    # publish at 10z
    cmd_vel_pub.publish(twist)
    rate.sleep()