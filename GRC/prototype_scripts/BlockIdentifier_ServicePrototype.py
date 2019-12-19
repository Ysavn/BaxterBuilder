#!/usr/bin/env python

from std_msgs.msg import Float32MultiArray, String, Float32, MultiArrayDimension, Int32
from sensor_msgs.msg import Image
import rospy
import numpy as np
from baxter_builder.srv import *
from rospy.service import ServiceManager

obj_found = False
obj_color = 0 # 0 is red, 1 is green, 2 is blue
xb = 0
yb = 0
zb = 1.0

def update_location(msg):
    xb = msg
    obj_color = 0
    obj_found = True

def get_obj_location(request):
    global xb
    global yb
    global zb
    global obj_found
    global obj_color
    zb = 0
    obj_found = True
    if request.targetcolor == "red":
        xb = 1
        yb = 1
    while xb == 0 and yb == 0:
        rospy.sleep(1)
    return ObjectLocationResponse(xb, yb, zb, obj_found, obj_color)

def main():
    rospy.init_node('block_identifier')
    print("Node initialized!")
    image_sub = rospy.Subscriber('counter', Int32, update_location) # this is getting continuously called?
    rospy.sleep(1)
    rospy.Service("obj_location_service", ObjectLocation, get_obj_location)
    print("I have created the service")
    if rospy.is_shutdown():
    	block_location_srv.shutdown()
        ("I have shutdown the service")
    rospy.spin()

if __name__ == '__main__':
    main()

