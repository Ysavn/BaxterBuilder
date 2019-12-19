#!/usr/bin/env python

import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import moveit_msgs.msg, geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import Pose, Point, Quaternion


rospy.init_node('pick_and_place', anonymous=True)
print("Initializing...")
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

print("Initialized.")



ready = Pose(Point(0.55, -0.55, -0.05), Quaternion(0, 1, 0, 0))
arm.set_start_state_to_current_state()
arm.set_pose_target(ready)
plan = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()
print("Ready to begin.")
rospy.sleep(3)																					#Pause

while not rospy.is_shutdown():
	global blockLoc
	blockLoc = [0.5, 0.2, -0.245]																#Hard-code block location
	print("Block selected.")
	global targetLoc
	targetLoc = [0.4, -0.2, -0.245] 															#Hard-code target location
	print("Target selected.")
	rospy.sleep(3)																				#Pause
	
	blockReady = Pose(Point(blockLoc[0], blockLoc[1], -0.05), Quaternion(0, 1, 0, 0))			#Move to block
	arm.set_start_state_to_current_state()
	arm.set_pose_target(blockReady)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	rospy.sleep(3)																				#Pause
	
	targetReady = Pose(Point(targetLoc[0], targetLoc[1], -0.05), Quaternion(0, 1, 0, 0))		#Move to target
	arm.set_start_state_to_current_state()
	arm.set_pose_target(targetReady)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	rospy.sleep(3)																				#Pause
	
	ready = Pose(Point(0.55, -0.55, -0.05), Quaternion(0, 1, 0, 0))								#Move to ready
	arm.set_start_state_to_current_state()
	arm.set_pose_target(ready)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	rospy.sleep(3)																				#Pause

