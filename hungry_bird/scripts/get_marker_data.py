#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}

		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		mark_0 = msg.poses[0].position
		mark_1 = msg.poses[1].position
		mark_2 = msg.poses[2].position
		#print type(mark_2.x), "natasha"
		point_0 = [round(mark_0.x,3), round(mark_0.y,3), round(mark_0.z,3)]
		point_1 = [round(mark_1.x,3), round(mark_1.y,3), round(mark_1.z,3)]
		point_2 = [round(mark_2.x,3), round(mark_2.y,3), round(mark_2.z,3)]
		self.whycon_marker = {0: point_0, 1: point_1, 2: point_2}
	
	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		mark_0 = msg.markers[0].pose.pose.orientation		
		mark_1 = msg.markers[1].pose.pose.orientation
		mark_2 = msg.markers[2].pose.pose.orientation
		orient_0 = [round(mark_0.x,3), round(mark_0.y,3), round(mark_0.z,3), round(mark_0.w,3)]
		orient_1 = [round(mark_1.x,3), round(mark_1.y,3), round(mark_1.z,3), round(mark_1.w,3)]
		orient_2 = [round(mark_2.x,3), round(mark_2.y,3), round(mark_2.z,3), round(mark_2.w,3)]
		self.aruco_marker = {0: orient_0, 1: orient_1, 2: orient_2}#msg.markers[0].pose.pose.orientation.x

		# Printing the detected markers on terminal
		print "\n"
		print "WhyCon_marker",self.whycon_marker
		print "ArUco_marker",self.aruco_marker




if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()