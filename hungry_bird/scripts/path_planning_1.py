#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import math
	
class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [5.61, -1.91, 54.95 ,0.0]	

		self.current_goal = [5.68, -1.91, 33.40, 0.0]	## initial_way_point [x, y, z, yaw]

		self.current_buffer = []		## list of current_path_points
		self.current_buffer_index = 0 
		self.threshold = 0.5

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd 			= PlutoMsg()
		self.cmd.rcRoll 	= 1500
		self.cmd.rcPitch 	= 1500
		self.cmd.rcYaw	 	= 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 	= 1500
		self.cmd.rcAUX2 	= 1500
		self.cmd.rcAUX3 	= 1500
		self.cmd.rcAUX4 	= 1500
		# self.cmd.plutoIndex = 0
		self.error_data = self.get_error_data([self.current_goal[i]-self.drone_position[i] for i in range(3)])

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [ -5, -5,  -3.5    ,0]
		self.Ki = [  0,  0,  -0.00015,0]
		self.Kd = [-30,-13,-190      ,0]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.error 		= [Float64(0.0), Float64(0.0), Float64(0.0), Float64(0.0)]
		self.last_error = [0.0,0.0,0.0,0.0]
		self.d_error 	= [0.0,0.0,0.0,0.0]
		self.sum_error  = [0.0,0.0,0.0,0.0]
		self.out_value 	= [0.0,0.0,0.0,0.0]
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]

		#some flags
		self.init_reached = False
		self.goal1_reached = False
		self.goal2_reached = False
		self.returned = False

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.080 # in seconds

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub 		= rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		self.alt_err_pub 		= rospy.Publisher('/alt_error', Float64, queue_size=10)
		self.pitch_err_pub 		= rospy.Publisher('/pitch_error', Float64, queue_size=10)
		self.roll_err_pub 		= rospy.Publisher('/roll_error', Float64, queue_size=10)
		self.yaw_err_pub 		= rospy.Publisher('/yaw_error', Float64, queue_size=10)
		self.pub_path_request 	= rospy.Publisher('/path_topic', Int16, queue_size=1)

		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuninltitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('whycon/poses', PoseArray,self.whycon_callback)
		#rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		rospy.Subscriber('/drone_yaw',Float64,self.yaw_callback)
		rospy.Subscriber('/vrep/waypoints',PoseArray,self.get_path) ## for getting path points to goal1
		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll 	= 1500
		self.cmd.rcYaw 		= 1500
		self.cmd.rcPitch 	= 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 	= 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z   			# data_type : float		 
		#---------------------------------------------------------------------------------------------------------------

	def yaw_callback(self,yaw):
		self.drone_position[3] = yaw.data	

	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,pitch):	
		self.Kp[0] = pitch.Kp*0.06
		self.Ki[0] = pitch.Ki*0.008
		self.Kd[0] = pitch.Kd*0.3

	def roll_set_pid(self,roll):	
		self.Kp[1] = roll.Kp*0.06
		self.Ki[1] = roll.Ki*0.008
		self.Kd[1] = roll.Kd*0.3

	## function to set self.K()[3]     (i.e for yaw)
	#----------------------------------------------------------------------------------------------------------------------
	#some extra call_backs
	def get_path(self,msg):
		# parse msg to create a list of whycon positions
		# update self.current_buffer and self.current_buffer_index
		self.current_buffer_index = 1
		self.current_buffer = []
		print "get_path called"
		for i in range(len(msg.poses)):
			mark = msg.poses[i].position
			point = [mark.x, mark.y, mark.z, 0.0]	## last one for yaw
			self.current_buffer.append(point)
			#print i,"----------  ",point		
		return


	#----------------------------------------------------------------------------------------------------------------------

	def get_error_data(self, inPut=False):		
		if not inPut:
			return [self.error[i].data for i in range(3)]
		else:
			return [inPut[i]-self.drone_position[i] for i in range(3)]


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sample_time defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum 
		#print type(self.drone_position[0]), type(self.setpoint[0])		
		#print "pid :",self.current_goal,"\n",self.error_data,self.threshold
		while (not rospy.is_shutdown()) and self.error_data[0]>self.threshold and \
											self.error_data[1]>self.threshold and \
											self.error_data[2]>self.threshold:
			self.error = [Float64(self.current_goal[i]-self.drone_position[i]) for i in range(4)]
			self.alt_err_pub.publish(self.error[2])
			self.pitch_err_pub.publish(self.error[0])
			self.roll_err_pub.publish(self.error[1])
			self.yaw_err_pub.publish(self.error[3])
			
			self.error_data = self.get_error_data()
			#print self.error_data
			#print self.error_data, self.threshold

			self.sum_error = [self.sum_error[i]+self.error[i].data for i in range(4)]
			self.d_error = [(self.error[i].data-self.last_error[i]) for i in range(4)]
			for i in range(4):
				self.out_value[i]  = self.Kp[i]*self.error[i].data
				self.out_value[i] += self.Ki[i]*self.sum_error[i]
				self.out_value[i] += self.Kd[i]*self.d_error[i]

			self.cmd.rcPitch = 1500+self.out_value[0]
			self.cmd.rcRoll = 1500+self.out_value[1]
			self.cmd.rcThrottle = 1500+self.out_value[2]	
			self.cmd.rcYaw = 1500+self.out_value[3]

			time.sleep(self.sample_time)
			
			if self.cmd.rcPitch <= self.min_values[0]:
				self.cmd.rcPitch = self.min_values[0]
			
			if self.cmd.rcRoll <= self.min_values[1]:
				self.cmd.rcRoll = self.min_values[1]
			
			if self.cmd.rcThrottle <= self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]
			
			if self.cmd.rcYaw <= self.min_values[3]:
				self.cmd.rcYaw = self.min_values[3]	

			if self.cmd.rcPitch >= self.max_values[0]:
				self.cmd.rcPitch = self.max_values[0]
			
			if self.cmd.rcRoll >= self.max_values[1]:
				self.cmd.rcRoll = self.max_values[1]
			
			if self.cmd.rcThrottle >= self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]
			
			if self.cmd.rcYaw >= self.max_values[3]:
				self.cmd.rcYaw = self.max_values[3]		

			self.last_error = [self.error[i].data for i in range(4)]			

			self.command_pub.publish(self.cmd)
		#print "pid exited"
		return


if __name__ == "__main__":
	
	e_drone = Edrone()
	# hellipad ----------> initial_waypoint
	e_drone.pid()

	# initial_waypoint ----------> goal1
	e_drone.init_error  = [5.68-e_drone.drone_position[0], -1.91-e_drone.drone_position[1], 33.40 - e_drone.drone_position[2]]	#[5.68, -1.91, 33.40, 0.0]	

	#if e_drone.get_error_data(e_drone.init_error)<=e_drone.threshold:
	e_drone.init_reached = True
		#publish this achievment inorder to get current_buffer and current_buffer_index updated
	msg = Int16()
	msg.data = 1
	e_drone.pub_path_request.publish(msg)
	time.sleep(1)
		#assuming current_buffer and current_buffer_index updated
	#e_drone.threshold = 1.0	#path-friendly threshold	
	#print "waypoints :",len(e_drone.current_buffer)
	#print "third point :", e_drone.current_buffer[0]
	while e_drone.current_buffer_index < len(e_drone.current_buffer):
		e_drone.current_goal = e_drone.current_buffer[e_drone.current_buffer_index]+[0.0]	## last one -- yaw value.
		e_drone.error_data = e_drone.get_error_data(e_drone.current_goal)
		e_drone.pid()
		time.sleep(0.5)
		e_drone.current_buffer_index += 3	#1 maybe changed if required.
	

	# goal1 ----------> goal2
	#print "goal1 ----------> goal2"
	#e_drone.threshold = 1.0 	#goal-friendly threshold	

	e_drone.goal1_error = [13.12-e_drone.drone_position[0], -.567-e_drone.drone_position[1], 41.89-e_drone.drone_position[2]]
	
	#if e_drone.get_error_data(e_drone.goal1_error)<=e_drone.threshold:
	e_drone.goal1_reached = True
	#publish this achievment inorder to get current_buffer and current_buffer_index updated
	msg = Int16()
	msg.data = 2
	e_drone.pub_path_request.publish(msg)
	time.sleep(1)
	#print "slept"
		#assuming current_buffer and current_buffer_index updated

	#e_drone.threshold = 1.0	#path-friendly threshold
	
	while e_drone.current_buffer_index < len(e_drone.current_buffer):
		e_drone.current_goal = e_drone.current_buffer[e_drone.current_buffer_index]+[0.0]	## last one -- yaw value.
		e_drone.error_data = e_drone.get_error_data(e_drone.current_goal)
		e_drone.pid()
		time.sleep(0.5)
		e_drone.current_buffer_index += 3	#1 maybe changed if required.


	# goal2 ----------> initial_waypoint
	#e_drone.threshold = 1.0 	#goal-friendly threshold	

	e_drone.goal2_error = [5.71-e_drone.drone_position[0], 8.38-e_drone.drone_position[1], 43.76-e_drone.drone_position[2]]
	
	#if e_drone.get_error_data(e_drone.goal2_error)<=e_drone.threshold:
	e_drone.goal2_reached = True
	#publish this achievment inorder to get current_buffer and current_buffer_index updated
	msg = Int16()
	msg.data = 3
	e_drone.pub_path_request.publish(msg)
	time.sleep(1)
		#assuming current_buffer and current_buffer_index updated

	#e_drone.threshold = 1.0	#path-friendly threshold
	
	while e_drone.current_buffer_index < len(e_drone.current_buffer):
		e_drone.current_goal = e_drone.current_buffer[e_drone.current_buffer_index]+[0.0]	## last one -- yaw value.
		e_drone.error_data = e_drone.get_error_data(e_drone.current_goal)
		e_drone.pid()
		time.sleep(0.5)
		e_drone.current_buffer_index += 3	#1 maybe changed if required.
	

	#while not e_drone.goal2_reached:
	#	continue
	e_drone.disarm()	
