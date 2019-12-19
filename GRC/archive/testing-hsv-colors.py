#!/usr/bin/env python

# testing out hsv colors

import cv2, cv_bridge
from std_msgs.msg import Float32MultiArray, String, Float32, MultiArrayDimension
from sensor_msgs.msg import Image
import rospy, cv2, cv_bridge, numpy as np
import imutils

bridge = cv_bridge.CvBridge()

def image_callback(msg):
    #Pulls image messages, converts them to OpenCV messages in HSV color space
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # blurred = cv2.GaussianBlur(image, (5, 5), 0) # blur to reduce noise
    # kernel = np.ones((5,5),np.uint8)
    # closing = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, kernel) #dilate then erode
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red_0 = np.array([0, 100, 100]) # red needs two separate ranges since it wraps around 0
    upper_red_0 = np.array([15, 255, 255])
    lower_red_1 = np.array([180 -15, 100, 100])
    upper_red_1 = np.array([180, 255, 255])

    lower_yellow = np.array([23, 41, 133])
    upper_yellow = np.array([40, 150, 255])

    mask_0 = cv2.inRange(hsv, lower_red_0 , upper_red_0)
    mask_1 = cv2.inRange(hsv, lower_red_1 , upper_red_1 )
    mask_red = cv2.bitwise_or(mask_0, mask_1)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = mask_yellow
    masked = cv2.bitwise_and(image, image, mask = mask)

    # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('image', 600, 600)
    cv2.imshow('image', mask_yellow)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

rospy.init_node('testing_hsv')
image_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image,image_callback)
rospy.spin()
