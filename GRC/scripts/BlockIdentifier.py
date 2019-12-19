#!/usr/bin/env python

# Updated Dec 13, 2019

# node block_identifier
# subscribes to '/cameras/left_hand_camera/image' topic
# subscribes to 'target_color' topic
# returns a ObjLocation for the PickPlace to read
# Colors possible to detect: red, green, blue

# import cv2, cv_bridge
from std_msgs.msg import Float32MultiArray, String, Float32, MultiArrayDimension
from sensor_msgs.msg import Image
import rospy, cv2, cv_bridge, numpy as np
from shapedetector import ShapeDetector
import imutils
from baxter_builder.srv import *

# calculated the values for colors using opencv_color_selector
lower_red_1 = np.array([0, 136, 26])
upper_red_1 = np.array([13, 255, 77])
lower_blue = np.array([101,23,15])
upper_blue = np.array([179,136,42])
lower_green = np.array([50,23,9])
upper_green = np.array([90,116,56])

# service inspired by Northwestern's ObjLocation service
obj_found = False
obj_color = 0 # 0 is red, 1 is green, 2 is blue
xb = 0 #0.5
yb = 0 #0.2
zb = -0.25

ccx = 0.127*(1.35)/(495-430) # Camera calibration factor X axis
ccy = 0.127*(1.35*0.8)/(197-139) # Camera calibration factor Y axis
pose_x = 0.64
pose_y = 0

# image is 640x400 pixels
row = 62
height = 264
column = 512
width = 64

kernel = np.ones((3, 3), np.uint8)

target_color_msg = ""

def set_color(msg):
    global target_color_msg
    if msg.data == "RED" or msg.data == "GREEN" or msg.data == "BLUE":
        #print("I am in set color for block identifier")
        target_color_msg = msg.data
        #print(target_color_msg)

def image_callback(msg):
    global xb, yb, obj_color, obj_found

    bridge = cv_bridge.CvBridge()
    #Pulls image messages, converts them to OpenCV messages in HSV color space
    image = bridge.imgmsg_to_cv2(msg)
    image = cv2.GaussianBlur(image, (5, 5), 0) # blur to reduce noise
    kernel = np.ones((3,3),np.uint8)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)  #erode then dilate
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel) #dilate then erode
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.imwrite("hsv_BlockIdentifier.jpg", hsv) # saves the image

    lower = np.array([0, 136, 26])
    upper = np.array([13, 255, 77])
    # set the color based off of target_color
    if target_color_msg == "RED":
        print("I have been changed to red in BlockIdentifier")
        obj_color = 0
        lower = lower_red_1
        upper = upper_red_1
    elif target_color_msg == "GREEN":
        print("I have been changed to red in BlockIdentifier") 
        obj_color = 1
        lower = lower_green
        upper = upper_green
    elif target_color_msg == "BLUE":
        print("I have been changed to blue in BlockIdentifier")
        obj_color = 2
        lower = lower_blue
        upper = upper_blue
    mask = 0
    # search for objects in designated ROI
    mask = cv2.inRange(hsv, lower , upper)
    mask = mask[row:row+height,column:column+width] # only consider the ROI
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  #erode then dilate
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  #dilate then erode
    cv2.imwrite("mask_BlockIdentifier.jpg", mask) # saves the image
    # sd = ShapeDetector() #TODO: can comment this out if slowing it down
    cnts_temp = cv2.findContours(mask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts_temp)
    # cnts = [c for c in cnts_temp if sd.detect(c) == "square"] # filter out the contours so that they only detect squares
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]  # get largest contour
    num_obj = len(cnts)
    if num_obj > 0:
        for c in cnts:
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx_int = int(M['m10']/M['m00'])
                cy_int = int(M['m01']/M['m00'])
            else:
                cx_int = 0
                cy_int = 0
            cx_int += column
            cy_int += row
            print("This is cx: ")
            print(cx_int)
            print("This is cy: ")
            print(cy_int)
            xb = ((-cy_int + 200) * ccy) + pose_x
            yb = ((-cx_int + 320) * ccx) + pose_y
            print("This is xb: ")
            print(xb)
            print("This is yb: ")
            print(yb)
            cv2.circle(image, (cx_int, cy_int), 5, (255, 255, 255), -1)
            cv2.putText(image, "centroid", (cx_int - 25, cy_int - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.imwrite("centroid_of_BlockIdentifier.jpg", image)
            obj_found = True

def get_obj_location(request):
    global xb, yb, obj_color, obj_found
    # TODO: might have to change this condition so that it eventually returns a response
    while xb == 0 and yb == 0:
        rospy.sleep(1)
    print("I am about to respond to the service in block identifier")
    return ObjLocationResponse(xb, yb, zb, obj_found, obj_color)

# ROS has generated three classes: ObjLocation, ObjLocationRequest, ObjLocationResponse
def main():
    print("Let's go!")
    rospy.init_node('block_identifier')
    print("initialized node")
    target_color_sub = rospy.Subscriber('target_color', String, set_color)
    image_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, image_callback)
    block_location_srv = rospy.Service("block_location_service", ObjLocation, get_obj_location)
    print("I have started the service")
    if rospy.is_shutdown():
        block_location_srv.shutdown()
        ("I have shutdown the service")
    rospy.spin()

if __name__ == '__main__':
    main()
