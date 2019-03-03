#!/usr/bin/env python

from geometry_msgs.msg import Quaternion
from aruco_msgs.msg import MarkerArray
import rospy

class Dalal():
	def __init__(self):
		rospy.init_node('middleMan',anonymous=False)
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.converter)
		self.pub = rospy.Publisher('/aruco_marker_publisher_markers', Quaternion, queue_size=10)

	def converter(self,msg):
		mark = msg.markers[0].pose.pose.orientation

		send = Quaternion()
		#print(mark)
		send = mark
		#print send
		#print "\n"
		self.pub.publish(send)

if __name__ == "__main__":
	D = Dalal()	
	while not rospy.is_shutdown():
		pass	
		
