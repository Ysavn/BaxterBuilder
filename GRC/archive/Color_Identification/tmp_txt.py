#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

bridge = cv_bridge.CvBridge()
def callback(msg):
	cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
	lower_yellow = np.array([23, 41, 133])
	upper_yellow = np.array([40, 150, 255])
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	masked = cv2.bitwise_and(cv_img, cv_img, mask = mask)
	cv2.imshow('dst', mask)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()

rospy.init_node('read_target_text')
sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
rospy.spin()
