#!/usr/bin/env python

import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import moveit_msgs.msg, geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
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
	global robot
	robot = moveit_commander.RobotCommander()
	global scene
	scene = moveit_commander.PlanningSceneInterface()
	print("Clients initiated.")
	
	#instantiate a MoveGroupCommander object for the arm
	#Tolerance setting recommended by multiple sites; appropriate value taken from Northwestern group
	print("Initiating Baxter's arm...")
	global arm
	arm = moveit_commander.MoveGroupCommander("right_arm")
	arm.set_goal_position_tolerance(0.01)
	arm.set_goal_orientation_tolerance(0.01)
	print("Arm ready.")

def move_to_ready(): #Movement scripting taken from MoveIt! tutorial "Move Group Python Interface"
	ready = Pose(Point(0.55, -0.55, -0.05), Quaternion(0, 1, 0, 0))
	arm.set_start_state_to_current_state()
	arm.set_pose_target(ready)
	arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	
def move_to_block(blockLoc):
	blockReady = Pose(Point([0.5, 0.2, -0.05], Quaternion(0, 1, 0, 0))
	arm.set_start_state_to_current_state()
	arm.set_pose_target(blockReady)
	arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()

def move_to_target(targetLoc):
	targetReady = Pose(Point([0.4, -0.2, -0.05], Quaternion(0, 1, 0, 0))
	arm.set_start_state_to_current_state()
	arm.set_pose_target(targetReady)
	arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()

def main():
	rospy.init_node('pick_and_place', anonymous=True)
	print("Initializing...")
	init()
	rospy.sleep(5)
	print("Initialized.")
	move_to_ready()
	print("Ready to begin.")
	rospy.sleep(5)
	while not rospy.is_shutdown():
		#blockLoc = [0.5, 0.2, -0.243]
		print("Block selected.")
		#targetLoc = [0.4, -0.2, -0.243] 
		print("Target selected.")
		rospy.sleep(2)
		move_to_block(blockLoc)
		rospy.sleep(2)
		move_to_target(targetLoc)
		rospy.sleep(2)
		move_to_ready()
		rospy.sleep(2)

if __name__ == '__main__':
	main()	

