#!/usr/bin/env python

#subscribes to left hand camera
# publishes to 'target_color' topic

import rospy
from sensor_msgs.msg import Image
import cv2 as cv, cv_bridge
import numpy as np
import subprocess
import pytesseract
from matplotlib import pyplot as plt
from std_msgs.msg import String

row = 140
height = 150
column = 200
width = 300


bridge = cv_bridge.CvBridge()
def callback(msg):
	img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	cv.imwrite("ashwad.png", img)
	#img = cv.imread('input6.jpg')
	#cv.imshow('dat', img)
	#cv.waitKey(0)
	img = img[row:row+height, column:column+width]
	gf_img = cv.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)
	cv.imwrite('sampleTargetFiltered.png', gf_img)
	pass_arg = []
	pass_arg.append("./whiteboardClean.sh")
	pass_arg.append("sampleTargetFiltered.png")
	pass_arg.append("cleanedTarget.png")
	subprocess.check_call(pass_arg)

	pass_arg2 = []
	pass_arg2.append("./textclean.sh")
	subprocess.check_call(pass_arg2)
	cln_img = cv.imread('output.png')
	conf = ('-l eng --oem 3 --psm 11')
	text = pytesseract.image_to_string(cln_img, config=conf)
	f = open("tmpWrite.txt", "a")
	if text=='':
		f.write("Ashwad")
	else:
		f.write(text)
	f.close()
	while text!='' and not rospy.is_shutdown():
		pub.publish(text)
	#cv.imshow('dst', cln_img)
	#cv.waitKey(0)
	#cv2.destroyAllWindows()

rospy.init_node('my_camera_read')
#rospy.Rate(2)
pub = rospy.Publisher('target_color', String, queue_size=10)
sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
rospy.spin()
