#!/usr/bin/env python

import rospy
from pkg_ros_basics.msg import myMessage

def func_callback_topic_my_topic(myMsg):
	
	rospy.loginfo("Data Received: (%d, %s, %.2f, %.2f)", myMsg.id, myMsg.name, myMsg.temperature, myMsg.humidity)
	
def main():
	
	rospy.init_node('node_myMsg_listner', anonymous=True)
	rospy.Subscriber('my_topic', myMessage, func_callback_topic_my_topic)
	rospy.spin()
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSIntrruptException:
		pass
