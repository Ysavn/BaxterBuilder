#!/usr/bin/env python

import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import moveit_msgs.msg, geometry_msgs.msg
import baxter_interface


def init():
	#Enable Baxter
	#print("Enabling Baxter...")
	#baxter_interface.RobotEnable().enable()
	#rospy.sleep(1.)
	#print("Baxter enabled successfully.")

	print("Initiating service clients...")
	#global blockLocationService
	#blockLocationService = rospy.ServiceProxy("block_location_service", blockLoc)
	#global targetLocationService
	#targetLocationService = rospy.ServiceProxy("target_location_service", targetLoc)
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	global scene
	scene = moveit_commander.PlanningSceneInterface()
	print("Clients initiated.")
	
	#instantiate a MoveGroupCommander object for the arm and a Gripper object for the hand
	#Tolerance setting recommended by multiple sites; appropriate value taken from Northwestern group
	print("Initiating Baxter's hand...")
	global hand
	hand = baxter_interface.Gripper("right")
	hand.calibrate() #CHECK IF THIS REQUIRES US TO DO ANYTHING
	print("Hand ready.")

def grip_test():
	hand.open()
	rospy.sleep(3)
	hand.close()
	
def main():
	rospy.init_node('pick_and_place', anonymous=True)
	print("Initializing...")
	init()
	print("Initialized.")
	rospy.sleep(5)
	while not rospy.is_shutdown():
		grip_test()
		rospy.sleep(5)

if __name__ == '__main__':
	main()	

