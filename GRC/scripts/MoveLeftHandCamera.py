#!/usr/bin/env python

# node move_left_hand_camera
# moves Baxter's left arm up into "cameraman" position using moveIt package

import sys, rospy, tf, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
#from math import pi #This is only necessary if we wind up giving nonzero euler angles for orientation

#Information on Baxter's coordinate frame found at https://www.researchgate.net/publication/326268262_Optimal_randomized_path_planning_for_redundant_manipulators_based_on_Memory-Goal-Biasing
#Define desired orientation and pose - need to figure out where these numbers come from, what units they're in
#I set it to zero for now so that we can see what Baxter's default orientation is (I think it points straight down already by default?)

#orient = Quaternion(0.0454193467806, 0.885452336413, -0.451420745691, 0.100650649472)
orient = Quaternion(0.0, 1.0, 0.0, 0.0)

#Point and Pose are both geometry_msgs message types: 
#Point() defines a point in space as three floats xyz; Pose() defines a point position and quaternion orientation
#Based on that document, I think these measurements are given in meters, with the origin in Baxter's midsection
#By rough estimation, it looks like the table will be around 0 in z; extend from 0.5 to 0.8 in x; and from -0.3 to 0.3 in y
#So "above the table" might be something like the values I've inserted here

pose = Pose(Point(0.64, 0.0, 0.54), orient)

sys.argv.append("joint_states:=/robot/joint_states")
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
rospy.init_node('move_left_hand_camera')

#MoveGroupCommander is a ROS class, so we're defining arm as a MoveGroupCommander object(?)
#Arm names ("planning groups") are given here https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial

arm = moveit_commander.MoveGroupCommander("left_arm")
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.01)

#Move left arm to cameraman position
arm.set_start_state_to_current_state()
arm.set_pose_target(pose)
plan1 = arm.plan()
arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()



