#!/usr/bin/env python

#THIS IS THE FUNCTIONAL VERSION, DO NOT CHANGE

#Make sure both services are global
#Make sure to index the locations received from the service and make sure they have the correct z coordinate

#Make sure there are rospy.sleep after each hand command
#Add a waypoint to keep arm from knocking block after placing it


import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import moveit_msgs.msg, geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import Pose, Point, Quaternion

print("Testing - is this thing on?")

#Initiate all_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  
rospy.init_node('pick_and_place', anonymous=True)
print("Initializing...")
sys.argv.append("joint_states:=/robot/joint_states")
#Enable Baxter
#print("Enabling Baxter...")
#baxter_interface.RobotEnable().enable()
#rospy.sleep(1.)
#print("Baxter enabled successfully.")


#print("Initiating service clients...")
#global blockLocationService
#blockLocationService = rospy.ServiceProxy("block_location_service", #[message type])
#global targetLocationService
#targetLocationService = rospy.ServiceProxy("target_location_service", #[message type])

#print("Waiting for target_location_service...")
#rospy.wait_for_service("target_location_service")
#targetLoc = targetLocationService()
targetLoc = [0.4, -0.2, -0.245] 
print("Target selected.")
targetCol = 0 # red, need to set through service call


#blockResponse = blockLocationService.call(ObjLocationRequest())
#if blockResponse.objfound == True:
#blockLoc = [blockResponse.xb, blockResponse.yb, -0.25]
#blockCol = blockResponse.objcolor
blockLoc = [0.5, 0.2, -0.245]
print("Block selected.")


moveit_commander.roscpp_initialize(sys.argv)
global robot
robot = moveit_commander.RobotCommander()
global scene
scene = moveit_commander.PlanningSceneInterface()
print("Clients initiated.")

#instantiate a MoveGroupCommander object for the arm and a Gripper object for the hand
#Tolerance setting recommended by multiple sites; appropriate value taken from Northwestern group
print("Initiating Baxter's arm and hand...")
global arm
arm = moveit_commander.MoveGroupCommander("right_arm")
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.01)
global hand
hand = baxter_interface.Gripper("right")
hand.calibrate()
print("Arm and hand ready.")

print("Initialized.")


#Move to ready_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  
orient = Quaternion(0, 1, 0, 0)
ready = Pose(Point(0.55, -0.55, -0.05), orient)
arm.set_start_state_to_current_state()
arm.set_pose_target(ready)
plan = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()

print("Ready to begin.")
rospy.sleep(2)


#Loop through pick-and-place_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  
while not rospy.is_shutdown():

	#print("Waiting for block_location_service...")
	#rospy.wait_for_service("block_location_service")

	#Move to block__________________________________________________________________
	blockReady = Pose(Point(blockLoc[0], blockLoc[1], -0.05), orient)
	arm.set_start_state_to_current_state()
	arm.set_pose_target(blockReady)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	
	#Grip block____________________________________________________________________
	hand.open()
	rospy.sleep(1)
	blockGrip = Pose(Point(blockLoc[0], blockLoc[1], -0.245), orient)
	arm.set_start_state_to_current_state()
	arm.set_pose_target(blockGrip)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	hand.close()
	rospy.sleep(1)
	
	#Move back up___________________________________________________________________
	arm.set_start_state_to_current_state()
	arm.set_pose_target(blockReady)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	
	#Move to target_________________________________________________________________
	targetReady = Pose(Point(targetLoc[0], targetLoc[1], -0.05), orient)
	arm.set_start_state_to_current_state()
	arm.set_pose_target(targetReady)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	
	#Place block____________________________________________________________________
	targetPlace = Pose(Point(targetLoc[0], targetLoc[1], -0.245), orient)
	arm.set_start_state_to_current_state()
	arm.set_pose_target(targetPlace)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	hand.open()
	rospy.sleep(1)
	
	#Move back up___________________________________________________________________
	arm.set_start_state_to_current_state()
	arm.set_pose_target(targetReady)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	
	#Move back to ready_____________________________________________________________
	arm.set_start_state_to_current_state()
	arm.set_pose_target(ready)
	plan = arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()
	
