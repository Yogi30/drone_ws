#!/usr/bin/env python

import rospy
import mavros
from mavros_msgs.msg import *
from geometry_msgs.msg import Point,PoseStamped,Twist
from mavros_msgs.srv import *
import time
import math

current_state = State()
previous_state = State()

def takeoff():
	rospy.wait_for_service('mavros/cmd/takeoff')
	try:
		takeoff_handler = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
		takeoff_handler(altitude = 2)
	except rospy.ServiceException, e:
		print "Service takeoff call failed: %s"%e

def state_cb(msg):
	global current_state
	current_state = msg

if __name__ == '__main__':
	rospy.init_node('offboard_node',anonymous=True)
	rate = rospy.Rate(20)
	state_sub = rospy.Subscriber("mavros/state",State,state_cb)
	pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=10)
	arming_handler = rospy.ServiceProxy("mavros/cmd/arming",CommandBool)
	set_mode_handler = rospy.ServiceProxy("mavros/set_mode",SetMode)	

	circle = Twist()
	count = 0

	while not current_state.connected:
		rate.sleep()

	arming_handler(True)
	takeoff()

	circle.linear.y = 1 * math.cos(0.50 * count)
	circle.linear.x = 1 * math.sin(0.50 * count)
	circle.angular.z = -0.50

	for i in range(100):
		pub.publish(circle)
		rate.sleep()

	last_request = rospy.Time.now()

	try:
		while not rospy.is_shutdown():
			if (current_state.mode != "OFFBOARD") and (rospy.Time.now() - last_request > rospy.Duration(0.05)):
				set_mode_handler(base_mode=0, custom_mode="OFFBOARD") 
				last_request = rospy.Time.now()

			else:
				if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(0.05)):
					arming_handler(True) 
					last_request = rospy.Time.now()

			if previous_state.armed != current_state.armed:
				rospy.loginfo("%r" % current_state.armed) 

			if previous_state.mode != current_state.mode:
				rospy.loginfo("%r"%current_state.mode)

			previous_state = current_state

			circle.linear.y = 1 * math.cos(0.5*count)
			circle.linear.x = 1 * math.sin(0.5*count)
			circle.angular.z = -0.50
			
			count = count + 0.05

			pub.publish(circle)
			rate.sleep()

	except KeyboardInterrupt:
		print "Changing to Stabilized"
		set_mode_handler(base_mode=0, custom_mode="STABILIZED")
		
