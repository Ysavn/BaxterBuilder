Baxter the Builder
Claire Yang, Ashwad Pandit, Emily Golberg, Avneet Saluja

Necessary Package Dependencies:
Baxter Moveit Configuration
Baxter IKFast Left Arm Plugin, Baxter IKFast Right Arm Plugin
Baxter Description
OpenCV 2.4
Numpy
baxter_builder service

Steps to run our Baxter the Builder code:
Reboot laptop
Turn on Baxter
Verify that laptop is connected via ethernet, and on BaxterRouter wifi

Terminal 1: basic setup and system checks, joint trajectory action server
cd ros_ws
. baxter.sh
rosrun baxter_tools enable_robot.py -e
rostopic list
rosrun baxter_tools camera_control.py -l
 #If left hand camera is not enabled, do the below.
 #rosrun baxter_tools camera_control.py -c right_hand_camera OR head_camera
 #rosrun baxter_tools camera_control.py -o left_hand_camera
rosrun baxter_interface joint_trajectory_action_server.py

Terminal 2: run moveit thru RViz
. baxter.sh
cd src/GRC
roslaunch baxter_moveit_config baxter_grippers.launch

Terminal 3: (optional for debugging) view left hand camera
. baxter.sh
cd src/GRC
rosrun image_view image_view image:=/cameras/left_hand_camera/image

Terminal 4: run the MoveLeftHandCamera node to set up the left arm in the correct position
. baxter.sh
cd src/GRC
rosrun GRC MoveLeftHandCamera.py
NOTE: Left-hand camera postion: [x, y, z] = [0.64, 0, 0.54]

Terminal 5: run the RightArmReady script to move the right arm into the correct position
. baxter.sh
cd src/GRC
rosrun GRC RightArmReady.py

Terminal 6: run the TargetIdentifier script to start publishing target_color 
. baxter.sh
cd src/GRC
rosrun GRC TargetIdentifier.py

Terminal 7: run the BlockIdentifier node to identify centroid locations of blocks
. baxter.sh
cd src/GRC
rosrun GRC BlockIdentifier.py

Terminal 8: run the PickPlace script to start looping through of movement
. baxter.sh
cd src/GRC
rosrun GRC PickPlace.py

Terminal x: (optional) Echo current position and orientation of the end-effector
. baxter.sh
rostopic echo /robot/limb/right/endpoint_state

ALTERNATIVELY:
Launch the project using BaxterTheBuilderSetup.launch and BaxterTheBuilderDemo.launch, found in prototype_scripts folder

Troubleshooting Notes:
If getting 'package GRC not found' error upon rosrun, enter source /home/csci5551/ros_ws/devel/setup.bash into command line and try again.
