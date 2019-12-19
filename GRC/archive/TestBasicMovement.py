#!/usr/bin/env python

import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import moveit_msgs.msg, geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('move_right_arm', anonymous=True)
print("Initializing...")
print("Initiating service clients...")
sys.argv.append("joint_states:=/robot/joint_states")
moveit_commander.roscpp_initialize(sys.argv)
global robot
robot = moveit_commander.RobotCommander()
global scene
scene = moveit_commander.PlanningSceneInterface()
print("Clients initiated.")

print("Initiating Baxter's arm...")
global arm
arm = moveit_commander.MoveGroupCommander("right_arm")
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.01)
print("Arm ready.")

print("Initialized.")



#Ready
ready = Pose(Point(0.55, -0.55, -0.05), Quaternion(0, 1, 0, 0))
arm.set_start_state_to_current_state()
arm.set_pose_target(ready)
plan = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()
print("Ready to begin.")
rospy.sleep(2)	


blockLoc = [0.5, 0.2, -0.245]
targetLoc = [0.4, -0.2, -0.245] 

#Position 1
ready = Pose(Point(blockLoc[0], blockLoc[1], -0.05), Quaternion(0, 1, 0, 0))
arm.set_start_state_to_current_state()
arm.set_pose_target(ready)
plan = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()
rospy.sleep(2)	



#Position 2
ready = Pose(Point(targetLoc[0], targetLoc[1], -0.05), Quaternion(0, 1, 0, 0))
arm.set_start_state_to_current_state()
arm.set_pose_target(ready)
plan = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()
rospy.sleep(2)	



