#!/usr/bin/env python

import sys, rospy, tf, actionlib
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import baxter_interface
from baxter_builder.srv import *
# imports all, ObjectLocation, ObjectLocationRequest, ObjectLocationResponse

xb = 0.0
yb = 0.0
zb = 0.0
objfound = False
objcolor = 0

#Initiate all_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  
rospy.init_node('pick_and_place')

print("Initiating service clients...")
rospy.wait_for_service('obj_location_service')
blockLocationService = rospy.ServiceProxy("obj_location_service", ObjectLocation) # creates a local proxy for simple use
print("created service proxy")

# could change this to a for loop
while not rospy.is_shutdown():
        try:
		print("I am in the try")
		request = ObjectLocationRequest("red")
		print("request created")
		rospy.wait_for_service('obj_location_service')
		print("finished waiting for service")
		blockResponse = blockLocationService.call(request)
		print("called the service successfully")
		if blockResponse.objfound == True:
			print("Block selected.")
			ros.sleep(5)
	except:
		print("Service call failed")

#blockResponse = blockLocationService()
#print(type(blockResponse))
#request = WordCountRequest('one two three', 3)
#count,ignored = word_counter(request)

print("Done.")

rospy.spin()

#Loop through pick-and-place_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  
#while not rospy.is_shutdown():
	#print("Waiting for target_location_service...")
	#rospy.wait_for_service("target_location_service")
	#targetLoc = targetLocationService()

#	print("Waiting for block_location_service...")
#	rospy.wait_for_service("block_location_service_2")

#	blockResponse = blockLocationService.call(ObjLocationRequest())
        #blockResponse = blockLocationService() # call this like a local function
#	if blockResponse.objfound == True:
#		blockLoc = [blockResponse.xb, blockResponse.yb, -0.25]
#		blockCol = blockResponse.objcolor
#		print("Block selected.")
#        rospy.spin()
