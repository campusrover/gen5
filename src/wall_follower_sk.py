#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Follow():

    def scan_callback(self, msg):
        ''' Wall following algorithm here.
            Feel free to use helper functions for modularity'''

    def __init__(self):

        scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        ## suggested parameters
        # response variable(s) that reacts to the call back function should also be declared
        self.safe_distance = ... ## should be a constant
        self.twist = Twist()
        self.twist.linear.x = ... ## should be a constant
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            '''set your angular velocity based on the response variable(s)'''
            self.twist.angular.z = ...

            cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    try:
        follow = Follow()
    except rospy.ROSInterruptException:  pass