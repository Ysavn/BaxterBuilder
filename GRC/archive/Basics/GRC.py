#!/usr/bin/env python 

import rospy

from std_msgs.msg import String



rospy.init_node('GRC')

pub = rospy.Publisher('targetText', String, queue_size = 10)

rate = rospy.Rate(2)

count = 0
while not rospy.is_shutdown():
	pub.publish("Avneet")
	count += 1
	rate.sleep()


