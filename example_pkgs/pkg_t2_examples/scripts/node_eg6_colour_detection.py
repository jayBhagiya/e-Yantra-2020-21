#!/usr/bin/env python

import rospy
import cv2
import sys
from imutils import contours
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Camera1:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
		self._shelf_data = [[0 for x in range(3)] for x in range(4)] 

	# def get_dominant_colour(self, arg_img):
	# 	# setting values for base colors 
	# 	g = arg_img[:, :, 1:2] 
	# 	r = arg_img[:, :, 2:] 
	# 	y = arg_img[:, :, :1:] 

	# 	# computing the mean 
	# 	g_mean = np.mean(g) 
	# 	r_mean = np.mean(r) 
	# 	y_mean = np.mean(y)

	# 	# displaying the most prominent color 
	# 	if (g_mean > r_mean and g_mean > y_mean): 
	# 		return 'green'
	# 	elif (r_mean > g_mean and r_mean > y_mean):
	# 		return 'red'
	# 	elif (g_mean > y_mean and r_mean > y_mean): 
	# 		return 'yellow'
	# 	else: 
	# 		return 'none'

	def get_dominant_colour(self, arg_img):

		colors = {'yellow': ([15, 25, 0], [50, 255, 255]), 'green': ([40, 15, 0], [179, 255, 255]), 'red': ([0, 30, 0], [25, 255, 255])}

		flag = 0

		for color, (lower, upper) in colors.items():
			lower = np.array(lower, dtype=np.uint8)
			upper = np.array(upper, dtype=np.uint8)

			hsv = cv2.cvtColor(arg_img, cv2.COLOR_BGR2HSV)
			color_mask = cv2.inRange(hsv, lower, upper)
			color_mask = cv2.merge([color_mask, color_mask, color_mask])

			mask = np.zeros(arg_img.shape, dtype=np.uint8)
			mask = cv2.bitwise_or(mask, color_mask)

			gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
			_, cnts, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			if len(cnts) > 0:
				flag = 1
				rospy.loginfo("color : {}".format(color))
				cv2.imshow('arg_img', arg_img)
				cv2.imshow('mask', mask)
				cv2.waitKey()
				return color

		if flag == 0:
			rospy.loginfo("color : None")
			return 'None'

	def update_datasheet(self, image):
		start = [50, 150]
		width = 90
		height = 75

		width_addition = 0
		height_addition = 0

		for row in range(4):
			y1 = start[1] + height_addition

			for col in range(3):
				x1 = start[0] + width_addition
				rospy.loginfo("[{}, {}]".format(row, col))
				rospy.loginfo("x = {}:{}, y = {}:{}".format(x1, x1 + width, y1, y1 + height))
				new_img = image[y1:y1 + height, x1:x1 + width]
				self._shelf_data[row][col] = self.get_dominant_colour(new_img)
				width_addition += width

			height_addition += height
			width_addition = 0

		rospy.loginfo(self._shelf_data)

	def callback(self,data):
		try:
		  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
		  rospy.logerr(e)

		(rows,cols,channels) = cv_image.shape

		image = cv_image

		# Resize a 720x1280 image to 360x640 to fit it on the screen
		resized_image = cv2.resize(image, (720/2, 1280/2))

		# cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
		self.update_datasheet(resized_image)

		cv2.waitKey(3)


def main(args):

	rospy.init_node('node_eg1_read_camera', anonymous=True)

	ic = Camera1()

	rospy.loginfo("Form Main : ")
	rospy.loginfo(ic._shelf_data)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)