#!/usr/bin/env python

import sys, rospy, baxter_interface


#Enable Baxter
print("Enabling Baxter...")
baxter_interface.RobotEnable().enable()
rospy.sleep(1)
print("Baxter enabled successfully.")

#Turning off any unneeded cameras
rightCam = baxter_interface.CameraController('right_hand_camera')
leftCam = baxter_interface.CameraController('left_hand_camera')
print("Turning off right-hand camera...")
rightCam.close()
print("Right-hand camera off.")
print("Turning on left-hand camera...")
leftCam.open()
print("Left-hand camera on.")

print("Baxter ready to begin.")






