#!/usr/bin/env python 

import rospy

from std_msgs.msg import Int32

def callback(msg):
	
	Intg = Int32()
	Intg.data = msg.data * 2
	pub.publish(Intg)



rospy.init_node('Listener_Publisher')


sub = rospy.Subscriber('counter', Int32, callback)
pub = rospy.Publisher('multiply_counter', Int32)



rospy.spin()





