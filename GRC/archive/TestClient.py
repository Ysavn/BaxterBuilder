#!/usr/bin/env python


import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import moveit_msgs.msg, geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import Pose, Point, Quaternion

#Initiate all_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  
rospy.init_node('pick_and_place', anonymous=True)
print("Initializing...")

#Enable Baxter
#print("Enabling Baxter...")
#baxter_interface.RobotEnable().enable()
#rospy.sleep(1.)
#print("Baxter enabled successfully.")

print("Initiating service clients...")
print("Waiting for block_location_service...")
rospy.wait_for_service("object_location_service")
global blockLocationService
blockLocationService = rospy.ServiceProxy("object_location_service", ObjLocation)
#global targetLocationService
#print("Waiting for target_location_service...")
#rospy.wait_for_service("target_location_service")
#targetLocationService = rospy.ServiceProxy("target_location_service", #[message type])
#moveit_commander.roscpp_initialize(sys.argv)
#global robot
#robot = moveit_commander.RobotCommander()
#global scene
#scene = moveit_commander.PlanningSceneInterface()
print("Clients initiated.")


while not rospy.is_shutdown():
	blockLoc = blockLocationService.call(ObjLocationRequest())
	print("Original: " + blockLoc)
	editedBlockLoc = blockLocationService.call(ObjLocationRequest())
	print("Edited: x = " + editedBlockLoc.xb + " y = " + editedBlockLoc.yb)
	finalBlockLoc = [editedBlockLoc.xb, editedBlockLoc.yb, -0.25]
	print("Final xyz: " + finalBlockLoc)
	rospy.sleep(2)







