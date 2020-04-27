#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# constants
SAFE_DISTANCE = 0.3 # meters
MAX_RANGE = 10 # meters
INDEX_THRESHOLD = 10 # degrees
MOVE = True # bool

# get scan values of 180 degree cone in front of the robot
# add comment
def range_ahead(ranges):
    return list(reversed(ranges[0:22])) + list(reversed(ranges[336:359]))

# scan callback to update longest/shortest range and the index (angle)
# of the longest range in front of the robot
def scan_cb(msg):
    global driving_forward; global max_range_index; global remove_inf_ranges

    # use forward facing scan to check for collisions
    ranges_ahead = range_ahead(list(msg.ranges))

    # get max range index as the new direction
    max_range = max(msg.ranges)
    max_range_index = msg.ranges.index(max_range)

    # set 0 ranges to max range
    # comment lambda
    remove_inf_ranges = map(lambda x: MAX_RANGE if x == 0 else x, ranges_ahead)


# check if we are facing in the direction of the maximum free range
def facing():
    return max_range_index < INDEX_THRESHOLD - 1 or max_range_index > 360 - INDEX_THRESHOLD - 1

# subscriber/publisher declarations
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.init_node('maze_runner')

# starting movement parameters
driving_forward = False
remove_inf_ranges = []
max_range_index = 0

# update at 10 hz
rate = rospy.Rate(10)

# control loop
while not rospy.is_shutdown():

    # drive forward or turn accordingly 
    twist = Twist()
    if driving_forward:

        # check if safe to move forward
        # add comment here
        twist.linear.x = 0.2
        if remove_inf_ranges: driving_forward = min(remove_inf_ranges) > SAFE_DISTANCE
    else:

        # keep spinning until facing correct direction
        twist.angular.z = 1.0
        driving_forward = facing()

    # publish at 10z
    if MOVE: cmd_vel_pub.publish(twist)
    rate.sleep()