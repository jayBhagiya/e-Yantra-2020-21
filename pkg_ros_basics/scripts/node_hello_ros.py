#!/usr/bin/env python

import rospy

def main():
	rospy.init_node('node_hello_ros', anonymous=True)
	rospy.loginfo('Hello World!!')
	rospy.spin()
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
