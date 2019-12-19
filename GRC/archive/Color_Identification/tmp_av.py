#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

bridge = cv_bridge.CvBridge()
def callback(msg):
	cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	
	cv2.imshow('dst', cv_img)
	cv2.waitKey(0)
	#cv2.destroyAllWindows()

rospy.init_node('my_camera_read')
sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
rospy.spin()
