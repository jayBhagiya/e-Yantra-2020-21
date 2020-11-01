#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time, math

x=0
y=0
theta=0

def pose_callback(pose):

	global x, y, theta

	# Testing values
	# rospy.loginfo(pose)
	# print 'Pose data : {}'.format(pose)

	# Storing the value of pose.x and pose.y in global variable x and y 
	x = pose.x
	y = pose.y
	theta = pose.theta

def move_in_circle():

	data = Twist()

	# Initialize the node and publish handle
	rospy.init_node('node_turtle_revolve', anonymous=True)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
	
	rospy.Subscriber('turtle1/pose', Pose, pose_callback)

	# Setting the linear and angular values for turtle
	data.linear.x = 2.0
	data.angular.z = 1.0

	# As per current position (5.55, 5.55) current distance is calculated
	current_distance = abs(math.sqrt(((5.55-0) ** 2) + ((5.55-0) ** 2)))
	rate = rospy.Rate(10)

	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

	# Current distance is removed so we can get distance_moved = 0
	distance_moved = -current_distance
	
	while not rospy.is_shutdown():

		# Taking the last position of turtle in (x0, y0)
		global x, y
		x0 = x
		y0 = y	
		
		rospy.loginfo("Moving in circle..")
		pub.publish(data)
		
		rate.sleep()

		# Calcutaling distance between current position (x, y) and last position (x0, y0)
		distance_moved = distance_moved + abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
		print distance_moved

		# Testing values
		# print 'X = {} , Y = {}'.format(x, y)
		# print 'X0 = {} , Y0 = {}'.format(x0, y0)

		# Comparing total distance with perimeter of circle which is 2*pi*radius 
		if not distance_moved < (2*math.pi*2): 
			rospy.loginfo('reached')
			break

	data.linear.x = 0
	data.angular.z = 1.0
	pub.publish(data)


		
if __name__ == '__main__':
	try:

		time.sleep(2) # Wait for 2 seconds
		move_in_circle()

	except rospy.ROSInterruptException:
		pass
