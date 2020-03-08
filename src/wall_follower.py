#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Follow():

    def scan_callback(self, msg):

        distanceL15 = msg.ranges[14]
        distanceR15 = msg.ranges[344]

        distanceL90 = msg.ranges[89]
        distanceR90 = msg.ranges[269]
        distanceL45 = msg.ranges[44]
        distanceR45 = msg.ranges[314]

        print 'L90: %f' % distanceL90
        print 'L45: %f' % distanceL45
        print 'R90: %f' % distanceR90
        print 'R45: %f' % distanceR45
        print '------'

        if distanceL90 != 0 or distanceR90 != 0:

            if distanceR90 == 0 or distanceL90 < distanceR90 and distanceL90 > 0:

                if distanceL45 == 0:
                    distanceL45 = 2 * distanceL90

                else:
                    if distanceL15 == 0:
                        self.obstacle_offset_distance = 0
                    elif distanceL15 <= 0.4:
                        self.obstacle_offset_distance = 0.3
                    elif distanceL15 <= 0.8:
                        self.obstacle_offset_distance = 0.15
                    else:
                        self.obstacle_offset_distance = 0

                p_distance = distanceL90
                a_distance = distanceL45
                self.isLeft = True
                
            else:

                if distanceR45 == 0:
                    distanceR45 = 2 * distanceR90

                else:
                    if distanceR15 == 0:
                        self.obstacle_offset_distance = 0
                    elif distanceR15 <= 0.4:
                        self.obstacle_offset_distance = 0.3
                    elif distanceR15 <= 0.8:
                        self.obstacle_offset_distance = 0.15
                    else:
                        self.obstacle_offset_distance = 0

                p_distance = distanceR90
                a_distance = distanceR45
                self.isLeft = False

            self.linear_offset_distance = self.safe_distance - p_distance
            self.angular_offset_distance = p_distance * math.sqrt(2) - a_distance
        

            
        print 'linear offset: %f' % self.linear_offset_distance
        print 'angular offset: %f' % self.angular_offset_distance
        print 'obstacle offset: %f' % self.obstacle_offset_distance
        print '------'

    def __init__(self):

        debug = True

        scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.safe_distance = 0.2
        twist = Twist()
        twist.linear.x = 0.10

        self.angular_offset_distance = 0
        self.linear_offset_distance = 0
        self.obstacle_offset_distance = 0
        self.isLeft = True
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            
            angular_velocity = -1 * (self.linear_offset_distance + self.angular_offset_distance + self.obstacle_offset_distance) if self.isLeft \
                else self.linear_offset_distance + self.angular_offset_distance + self.obstacle_offset_distance
            
            if abs(angular_velocity) != self.safe_distance:

                if angular_velocity > 0.5:
                    angular_velocity = 0.5

                elif angular_velocity < -0.5:
                    angular_velocity = -0.5

                twist.angular.z = angular_velocity

            cmd_vel_pub.publish(twist)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    try:
        follow = Follow()
    except rospy.ROSInterruptException:  pass