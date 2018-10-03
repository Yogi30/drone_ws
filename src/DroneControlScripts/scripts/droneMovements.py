#!/usr/bin/env python

import rospy
import mavros
from mavros_msgs.msg import *
from geometry_msgs.msg import Point,PoseStamped
from mavros_msgs.srv import *
import time

current_state = State()

class FlightControl:
	def __init__(self):		
		pass

	def takeoff(self):
		rospy.wait_for_service('mavros/cmd/takeoff')
		try:
			takeoff_handler = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
			takeoff_handler(altitude = 3)
		except rospy.ServiceException, e:
			print "Service takeoff call failed: %s"%e

	def arm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			arming_handler = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
			arming_handler(True)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def disarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
		    arming_handler = rospy.ServiceProxy("mavros/cmd/arming",CommandBool)
		    arming_handler(False) 
		except rospy.ServiceException, e:
			print "Service disarming call failed: %s"%e

class FlightModes:
	def __init__(self):
		self.rate = rospy.Rate(20)
		self.pose = PoseStamped()
		self.previous_state = State()

	def change_to_OffBoard(self):
		while not current_state.connected:
			self.rate.sleep()

		self.mode_handler = rospy.ServiceProxy("mavros/set_mode",SetMode)
		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 3
		self.setpoint_pub = rospy.Publisher("mavros/setpoint_position/local",PoseStamped,queue_size = 10)

		for i in range(100):
			self.setpoint_pub.publish(self.pose)
			self.rate.sleep()

		self.last_request = rospy.Time.now()
		while not current_state.mode == "OFFBOARD":

			if (current_state.mode != "OFFBOARD") and (rospy.Time.now() - self.last_request > rospy.Duration(5)):
				self.mode_handler(base_mode=0, custom_mode="OFFBOARD") 
				self.last_request = rospy.Time.now()

			if self.previous_state.mode != current_state.mode:
				rospy.loginfo("%r"%current_state.mode)

			self.previous_state = current_state
			self.pose.header.stamp = rospy.Time.now()
			self.setpoint_pub.publish(self.pose)
			self.rate.sleep()

	def change_to_Stabilized(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			mode_handler = rospy.ServiceProxy("mavros/set_mode",SetMode)
			mode_handler(custom_mode='STABILIZED')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

	def change_to_Altitude(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			mode_handler = rospy.ServiceProxy("mavros/set_mode",SetMode)
			mode_handler(custom_mode='ALTCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Altitude Mode could not be set."%e

	def change_to_Position(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			mode_handler = rospy.ServiceProxy("mavros/set_mode",SetMode)
			mode_handler(custom_mode='POSCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Position Mode could not be set."%e

	def change_to_AutoLand(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			mode_handler = rospy.ServiceProxy("mavros/set_mode",SetMode)
			mode_handler(custom_mode='AUTO.LAND')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Autoland Mode could not be set."%e

	def state_cb(self,msg):
		global current_state
		current_state = msg

class FlightMovements:
	def __init__(self):
		self.state = State()
		self.drone_pos_target = PositionTarget()
		self.drone_pos_target.type_mask = int('010111111000', 2)
		self.drone_pos_target.coordinate_frame = 1
		self.local_pos = Point()
		self.drone_pos_target.position.x = 0.0
		self.drone_pos_target.position.y = 0.0
		self.drone_pos_target.position.z = 0.0

	def position_cb(self, msg):
		self.local_pos.x = msg.pose.position.x
		self.local_pos.y = msg.pose.position.y
		self.local_pos.z = msg.pose.position.z

	def update_drone_pos(self):
		self.drone_pos_target.position.x = self.local_pos.x
		self.drone_pos_target.position.y = self.local_pos.y
		self.drone_pos_target.position.z = self.local_pos.z

	def x_dir(self):
		self.update_drone_pos()
		self.drone_pos_target.position.x = self.local_pos.x + 5
		self.drone_pos_target.position.y = self.local_pos.y
		rospy.loginfo("x_dir")

	def neg_x_dir(self):
		self.update_drone_pos()
		self.drone_pos_target.position.x = self.local_pos.x - 5
		self.drone_pos_target.position.y = self.local_pos.y

	def y_dir(self):
		self.update_drone_pos()
		self.drone_pos_target.position.x = self.local_pos.x
		self.drone_pos_target.position.y = self.local_pos.y + 5

	def neg_y_dir(self):
		self.update_drone_pos()
		self.drone_pos_target.position.x = self.local_pos.x
		self.drone_pos_target.position.y = self.local_pos.y - 5

if __name__ == '__main__':
	rospy.init_node('MovementControl',anonymous=True)	
	rate = rospy.Rate(20)

	droneMode = FlightModes()
	droneMovement = FlightMovements()
	droneControl = FlightControl()

	state_sub = rospy.Subscriber("mavros/state",State,droneMode.state_cb)
	pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, droneMovement.position_cb)
	
	while not current_state.connected:
		rate.sleep()
	rospy.loginfo("Connected: %r"% current_state.connected)
	while not current_state.armed:
			droneControl.arm()
			rate.sleep()
	rospy.loginfo("Armed: %r"% current_state.armed)
	droneMode.change_to_OffBoard()
	droneControl.takeoff()
