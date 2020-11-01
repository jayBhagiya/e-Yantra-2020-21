#!/usr/bin/env python

# ROS Node - Action Server - Ros - IoT Bridege

import rospy
import actionlib
import threading
import requests

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

from pkg_ros_iot_bridge.msg import msgMqttSub

from pyiot import iot

class ActionServerRosIotBridge:

	"""docstring for ActionServerRosIotBridge"""
	def __init__(self):

		self._asri = actionlib.ActionServer('action_ros_iot', msgRosIotAction, self.on_goal, self.on_cancel, auto_start=False)

		param_config_iot = rospy.get_param('config_pyiot')
		self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
		self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
		self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
		self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
		self._config_mqtt_qos = param_config_iot['mqtt']['qos']
		self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

		self._config_google_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']

		self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

		ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback, self._config_mqtt_server_url, self._config_mqtt_server_port, 
												self._config_mqtt_sub_topic, self._config_mqtt_qos)
		
		if ret == 0:
			rospy.loginfo("Mqtt Subscribe Thred Started")
		else:
			rospy.loginfo("Failed to start Mqtt Subscribe Thread")

		self._asri.start()

		rospy.loginfo("Started ROS-IoT Bridge Action Server")

	def mqtt_sub_callback(self, client, userdata, message):
		payload = str(message.payload.decode("utf-8"))

		print("[Mqtt SUB CB] Message : ", payload)
		print("[Mqtt SUB CB] Topic : ", message.topic)

		msg_mqtt_sub = msgMqttSub()
		msg_mqtt_sub.timestamp = rospy.Time.now()
		msg_mqtt_sub.topic = message.topic
		msg_mqtt_sub.message = payload

		self._handle_ros_pub.publish(msg_mqtt_sub)

	def on_goal(self, goal_handle):
		
		goal = goal_handle.get_goal()

		rospy.loginfo("Received new goal form Client")
		rospy.loginfo(goal)

		if(goal.protocol == "mqtt"):

			if ((goal.mode == "pub") or (goal.mode == "sub")):
				goal_handle.set_accepted()

				thread = threading.Thread(name="worker", target=self.on_process, args=(goal_handle,))

				thread.start()

			else:
				goal_handle.set_rejected()
				return

		else:
			goal_handle.set_rejected()
			return

	def on_process(self, goal_handle):
		
		flag_success = False
		result = msgRosIotResult()

		goal_id = goal_handle.get_goal_id()
		rospy.loginfo("Processing Goal : " + str(goal_id.id))

		goal = goal_handle.get_goal()

		if (goal.protocol == "mqtt"):
			rospy.logwarn("MQTT")

			if goal.mode == "pub":
				rospy.logwarn("MQTT pub Goal ID : " + str(goal_id.id))

				rospy.logwarn(goal.topic + " > " + goal.message)

				sh_res = tuple(map(int, goal.message.split(', ')))
				ret = iot.mqtt_publish(self._config_mqtt_server_url, self._config_mqtt_server_port, goal.topic, str(sh_res), self._config_mqtt_qos)

				iot.push_data(self._config_google_spread_sheet_id, sh_res[0], sh_res[1], sh_res[2])

				if ret == 0:
					rospy.loginfo("Mqtt Publish Successful")
					result.flag_success = True
				else:
					rospy.loginfo("Mqtt Failed to Publish")
					result.flag_success = False

		rospy.loginfo("Send goal result to client")
		if result.flag_success == True:
			rospy.loginfo("Succeeded")
			goal_handle.set_succeeded(result)
		else:
			rospy.loginfo("Goal Failed, Aborting")
			goal_handle.set_aborted(result)

		rospy.loginfo("Goal ID : " + str(goal_id.id) + " Goal processing done.")

	def on_cancel(self, goal_handle):
		rospy.loginfo("Received cancel request.")
		goal_id = goal_handle.get_goal_id()


def main():
	rospy.init_node('node_action_server_ros_iot_bridge')

	action_server = ActionServerRosIotBridge()

	rospy.spin()

if __name__ == '__main__':
	main()
		