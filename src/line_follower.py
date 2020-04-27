#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv

# constants
MOVE = True
PUBLISH_ROS_IMGS = True
BRIDGE = CvBridge()
LINE_WIDTH = 230                    # pixels
LEFT_BOUND = 320 - LINE_WIDTH/2     # pixels
RIGHT_BOUND = 320 + LINE_WIDTH/2    # pixels
MOVE_SPEED = 0.15                   # m/s
TURN_SPEED = 0.5                    # m/s
CONTOUR_SIZE_THRESHOLD = 0          # pixels ^ 2

# callback for each camera frame
def cv_cb(msg):
    cv_image = BRIDGE.compressed_imgmsg_to_cv2(msg)
    detect_line(cv_image)

# draw bounding boxes
def detect_line(img):
    global driving_forward; global turn_direction

    # correct image rotation
    img = cv.flip(img, -1)

    # gray scale + blur image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray,(5,5),0)
    ret, thresh = cv.threshold(blur,60,255,cv.THRESH_BINARY_INV)

    # get contours from image
    a, contours, b = cv.findContours(thresh.copy(), 1, cv.CHAIN_APPROX_NONE)

    # if any contours exist
    if len(contours) > 0:

        # get bounding box of largest contour 
        c = max(contours, key = cv.contourArea)
        x, y, w, h = cv.boundingRect(c)
        size = w * h
        print(str(size))
        # if contour is significant
        if size > CONTOUR_SIZE_THRESHOLD:

            # get mid points of bounding box to draw vertical line
            lower_center_pt = (x + w/2, y + h)
            upper_center_pt = (x + w/2, y)

            # adjust movement to match relative position of line
            if x > LEFT_BOUND and x < RIGHT_BOUND:
                driving_forward = True
            elif x < LEFT_BOUND:
                driving_forward = False
                turn_direction = 1
            elif x > RIGHT_BOUND:
                driving_forward = False
                turn_direction = -1
            cv.line(img, lower_center_pt, upper_center_pt, (255, 0, 0))

            # publish image with line detection to ROS
            if PUBLISH_ROS_IMGS:
                ros_img = BRIDGE.cv2_to_imgmsg(img, 'rgb8')
                img_pub.publish(ros_img)

# subscriber/publisher declarations
cam_sub = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, cv_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
img_pub = rospy.Publisher('cv_line_image', Image, queue_size=1)

rospy.init_node('line_follower')

# update at 10 hz
rate = rospy.Rate(10)
driving_forward = True
turn_direction = 1

# control loop
while not rospy.is_shutdown():

    # drive forward or turn accordingly 
    twist = Twist()
    if driving_forward:
        twist.linear.x = MOVE_SPEED
    else:
        twist.angular.z = TURN_SPEED * turn_direction


    # publish at 10z
    if MOVE: cmd_vel_pub.publish(twist)
    rate.sleep()