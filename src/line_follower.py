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
MOVE_SPEED = 0.1                   # m/s
CONTOUR_SIZE_THRESHOLD = 0          # pixels ^ 2, currently not in use

# callback for each camera frame
def cv_cb(msg):
    cv_image = BRIDGE.compressed_imgmsg_to_cv2(msg)
    detect_line(cv_image)

# draw bounding boxes
def detect_line(img):
    global driving_forward, x_center

    # gray scale + blur image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray,(5,5),0)

    # ret, thresh = cv.threshold(blur,60,255,cv.THRESH_BINARY_INV)
    thresh = cv.adaptiveThreshold(blur,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)

    # get contours from image
    a, contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    try: hierarchy = hierarchy[0]
    except: hierarchy = []

    height, width, _ = img.shape
    min_x, min_y = width, height
    max_x = max_y = 0

    # computes the bounding box for the contour, and draws it on the frame,
    for contour, hier in zip(contours, hierarchy):
        (x,y,w,h) = cv.boundingRect(contour)
        min_x, max_x = min(x, min_x), max(x+w, max_x)
        min_y, max_y = min(y, min_y), max(y+h, max_y)
        if w > 80 and h > 80:
            cv.rectangle(img, (x,y), (x+w,y+h), (255, 0, 0), 2)

    if max_x - min_x > 0 and max_y - min_y > 0:
        cv.rectangle(img, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)

    print "contours: %s" % (len(contours))
    
    valid_contours = []
    x_sum = 0
    # highlight the contour box
    for c in contours:
        x, y, w, h = cv.boundingRect(c)
        lower_center_pt = (x + w/2, y + h)
        upper_center_pt = (x + w/2, y)
        cv.line(img, lower_center_pt, upper_center_pt, (255, 0, 0))

        # determine valid contours
        if y < 40 and y+h > 200:
            valid_contours.append(c)
            x_sum = x_sum + x + w/2

    if len(valid_contours) > 0:
        driving_forward = True
        x_center = x_sum / len(valid_contours)
        print 'x_center: %s' % (x_center)
        lower_center_pt = (x_center, 240)
        upper_center_pt = (x_center, 0)
        cv.line(img, lower_center_pt, upper_center_pt, (0, 255, 0))
    else:
        driving_forward = False
    print 'valid contours: %s' % (len(valid_contours))

    # publish image with line detection to ROS
    if PUBLISH_ROS_IMGS:
        ros_img = BRIDGE.cv2_to_imgmsg(img, 'rgb8')
        img_pub.publish(ros_img)

rospy.init_node('line_follower')

# subscriber/publisher declarations
cam_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, cv_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
img_pub = rospy.Publisher('cv_line_image', Image, queue_size=1)
x_center = 0

# update at 10 hz
rate = rospy.Rate(10)
driving_forward = False

# control loop
while not rospy.is_shutdown():

    # drive forward or turn based on the average center position of the x value of contour the 
    twist = Twist()
    if driving_forward:
        twist.linear.x = MOVE_SPEED
        twist.angular.z = (160 - x_center) / 360.0

    # publish at 10z
    if MOVE: cmd_vel_pub.publish(twist)
    rate.sleep()