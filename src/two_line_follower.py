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
TURN_SPEED = 0.1                    # rad/s
CONTOUR_SIZE_THRESHOLD = 2000       # pixels ^ 2

# callback for each camera frame
def cv_cb(msg):
    cv_image = BRIDGE.compressed_imgmsg_to_cv2(msg)
    detect_lines(cv_image)

# draw bounding boxes
def detect_lines(img):
    global driving_forward; global turn_direction

    # correct image rotation
    img = cv.flip(img, -1)

    # gray scale + blur image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray,(5,5),0)
    ret, thresh = cv.threshold(blur,60,255,cv.THRESH_BINARY_INV)

    # get contours from image
    a, contours, b = cv.findContours(thresh.copy(), 1, cv.CHAIN_APPROX_NONE)

    # if any contours exist            mid_line_x = (line_centers[0] + ) / 2
    if len(contours) > 0:

        # get bounding box of largest contour 
        # c = max(contours, key = cv.contourArea)
        line_count = 0
        line_centers = []
        for c in contours:
            
            # get dimensions of bounding box
            x, y, w, h = cv.boundingRect(c)
            size = w * h
            
            # if contour is significant
            if size > CONTOUR_SIZE_THRESHOLD:
                

                # get mid points of bounding box to draw vertical line
                lower_center_pt = (x + w/2, y + h)
                upper_center_pt = (x + w/2, y)
                line_count += 1
                line_centers.append(x + w/2)
                
        # if we can detect one or two distinct lines find mid line and follow
        if line_count == 2 or line_count == 1:

            # find x coordinate of mid_line
            mid_line_x = line_centers[0]
            if len(line_centers) > 1:
                mid_line_x = (mid_line_x + line_centers[1]) / 2

            # publish image with mid line
            if PUBLISH_ROS_IMGS:
                cv.line(img, (mid_line_x, 0), (mid_line_x, 640), (255, 0, 0))

            # adjust movement to match position of mid line
            if mid_line_x > LEFT_BOUND and mid_line_x < RIGHT_BOUND:
                driving_forward = True
            elif mid_line_x < LEFT_BOUND:
                driving_forward = False
                turn_direction = -1
            elif mid_line_x > RIGHT_BOUND:
                driving_forward = False
                turn_direction = 1
                   

        # publish image with line detection to ROS
        if PUBLISH_ROS_IMGS:
            ros_img = BRIDGE.cv2_to_imgmsg(img, 'rgb8')
            img_pub.publish(ros_img)

# subscriber/publisher declarations
cam_sub = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, cv_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
img_pub = rospy.Publisher('cv_line_image', Image, queue_size=1)

rospy.init_node('two_line_follower')

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
        twist.linear.x = MOVE_SPEED
        twist.angular.z = TURN_SPEED * turn_direction


    # publish at 10z
    if MOVE: cmd_vel_pub.publish(twist)
    rate.sleep()