#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
#from aruco_msgs.msg import MarkerArray
		
def whycon_data(msg):
	mark_0 = msg.poses[0].position
	mark_1 = msg.poses[1].position
	mark_2 = msg.poses[2].position
	#print type(mark_2.x), "natasha"
	point_0 = [round(mark_0.x,3), round(mark_0.y,3), round(mark_0.z,3)]
	point_1 = [round(mark_1.x,3), round(mark_1.y,3), round(mark_1.z,3)]
	point_2 = [round(mark_2.x,3), round(mark_2.y,3), round(mark_2.z,3)]
	#whyconpoint_0_marker = {0: point_0, 1: point_1, 2: point_2}
	print '\n'
	print point_0
	print point_1
	print point_2
	print '\n'

if __name__=="__main__":
	
	rospy.init_node('arena_setting',anonymous=False) # initializing a ros node with name marker_detection
	
	#whycon_marker = {}	# Declaring dictionaries
	
	#rospy.Subscriber('/whycon/poses',PoseArray,whycon_data)	# Subscribing to topic
	
	while not rospy.is_shutdown():
		rospy.spin()
