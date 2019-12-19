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

#crop dimensions
row = 140
height = 150
column = 200
width = 300


bridge = cv_bridge.CvBridge()
def callback(msg):
	#read the rgb image
	img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	
	#crop the image to region containing the text
	img = img[row:row+height, column:column+width]

	#denoise filter
	gf_img = cv.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)

	#save denoised image
	cv.imwrite('sampleTargetFiltered.png', gf_img)

	#runs the whiteboardClean script taking denoised image as input
	pass_arg = []
	pass_arg.append("./whiteboardClean.sh")
	pass_arg.append("sampleTargetFiltered.png")
	pass_arg.append("cleanedTarget.png")
	subprocess.check_call(pass_arg)

	#pass the output cleanedTarget.png from whiteboardClean script to textclean by ImageMagick further clean the text
	pass_arg2 = []
	pass_arg2.append("./textclean.sh")
	subprocess.check_call(pass_arg2)
	cln_img = cv.imread('output.png')

	#configuration for usage of tesseract library
	#-l => language, -oem 3 => default trained engine, --psm 11 => look for sparse text
	conf = ('-l eng --oem 3 --psm 11')
	text = pytesseract.image_to_string(cln_img, config=conf)
        #publish the identified text otherwise try again
	while text!='' and not rospy.is_shutdown():
		pub.publish(text)
	#cv.imshow('dst', cln_img)
	#cv.waitKey(0)
	#cv2.destroyAllWindows()

rospy.init_node('my_camera_read')
pub = rospy.Publisher('target_color', String, queue_size=10)
sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
rospy.spin()
