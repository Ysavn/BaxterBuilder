#!/usr/bin/env python

# Updated Dec. 13, 2019

#THIS IS THE FUNCTIONAL VERSION, DO NOT CHANGE

#Make sure both services are global
#Make sure to index the locations received from the service and make sure they have the correct z coordinate

#Make sure there are rospy.sleep after each hand command
#Add a waypoint to keep arm from knocking block after placing it

import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String, Int32
import moveit_msgs.msg, geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_builder.srv import *
# imports all ObjLocation, ObjLocationRequest, ObjLocationResponse

global blockLocationService
global robot
global scene
global arm
global hand

xb = 0.0
yb = 0.0
zb = 0.0
objfound = False
objcolor = 0
gripper_offset_x = -0.125
gripper_offset_y = 0.05


target_color = ""

def set_color(msg):
    global target_color
    if msg.data == "RED" or msg.data == "GREEN" or msg.data == "BLUE":
        target_color = msg.data

#Initiate all_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _
rospy.init_node('pick_and_place', anonymous=True)
print("Initializing...")
sys.argv.append("joint_states:=/robot/joint_states")
#Enable Baxter
#print("Enabling Baxter...")
#baxter_interface.RobotEnable().enable()
#rospy.sleep(1.)
#print("Baxter enabled successfully.")

print("Initiating service clients...")
rospy.wait_for_service('block_location_service')
blockLocationService = rospy.ServiceProxy("block_location_service", ObjLocation)

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
print("Clients initiated.")

#instantiate a MoveGroupCommander object for the arm and a Gripper object for the hand
#Tolerance setting recommended by multiple sites; appropriate value taken from Northwestern group
print("Initiating Baxter's arm and hand...")

arm = moveit_commander.MoveGroupCommander("right_arm")
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.01)

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

target_color_sub = rospy.Subscriber('target_color', String, set_color)

#Loop through pick-and-place_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _
while not rospy.is_shutdown():
    try:
        print("I am in the try")
	print(target_color)
        if len(target_color) > 0: # only move if it's ready and has a color
            targetLoc = [0.4, -0.2, -0.245] 
            print("Target selected.")
            print("Waiting for block_location_service...")
            request = ObjLocationRequest()
            print("request created")
            rospy.wait_for_service("block_location_service")
            print("finished waiting for service")
            blockResponse = blockLocationService.call(request)
            print("called the service successfully")
            if blockResponse.objfound == True:
                print("Block selected.")
                rospy.sleep(1)
                blockLoc = [blockResponse.xb, blockResponse.yb, -0.25]
		print(blockLoc)
                blockCol = blockResponse.objcolor
                #blockLoc = [0.5, 0.2, -0.245]
		blockLoc[0] += gripper_offset_x
		blockLoc[1] += gripper_offset_y
                print("Block selected.")

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

    except rospy.ServiceException, e:
        print("Service call failed: %s", e)
