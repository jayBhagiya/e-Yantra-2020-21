#! /usr/bin/env python

import rospy
import sys
import copy
import cv2
import numpy as np
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_task4.msg import msgUR5_1Action, msgUR5_1Goal, msgUR5_1Feedback, msgUR5_1Result
from pkg_task4.msg import msgUR5_2Action, msgUR5_2Goal, msgUR5_2Feedback, msgUR5_2Result

class UR5_1_client:
    """docstring for UR5_1_client"""
    def __init__(self):

        self._ur5_1_action_client = actionlib.ActionClient('/action_ur5_1_pick_place', msgUR5_1Action)

        self._goal_handles_1 = {}

        self._service_conveyot_belt = '/eyrc/vb/conveyor/set_power'
        rospy.wait_for_service(self._service_conveyot_belt)
        self._conveyot_belt = rospy.ServiceProxy(self._service_conveyot_belt, conveyorBeltPowerMsg)

        self._topic_ur5_1_feedback = '/action_ur5_1_pick_place/feedback'
        rospy.Subscriber(self._topic_ur5_1_feedback, msgUR5_1Feedback, self.feedback_callback)

        self._feedback = True
        self._home_result = True
        self._counter = 1
        self._color_map = {'red' : 1, 'yellow' : 2, 'green' : 3}

        self._ur5_1_action_client.wait_for_server()
        rospy.loginfo("UR5 1 Pick and Place - Action server is up, we can send new goals!")

    def conveyor_belt_activator(self, set_power):
        result = self._conveyot_belt(set_power)
        if result.result == True:
            rospy.loginfo('\033[94m' + "Conveyor Belt started with power {}.".format(set_power) + '\033[0m')
        else:
            rospy.logerr("Conveyor Belt having some error, not started.")

    def on_transition(self, goal_handle):
        
        result = msgUR5_1Result()

        index = 0
        for i in self._goal_handles_1:
            if self._goal_handles_1[i] == goal_handle:
                index = i 
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )

        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo("[UR5 1 Result] Home State : {}".format(result.home_state))

            if (result.home_state == True):
                self._home_result = True
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                self._home_result = False
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    def send_goal(self, row, col):

        goal = msgUR5_1Goal(row=row, column=col)

        goal_handle = self._ur5_1_action_client.send_goal(goal, self.on_transition, None)

        rospy.loginfo('\033[94m' + "Goal has been sent." + '\033[0m')

        return goal_handle

    def result_callback(self, result):
        rospy.loginfo("[Result] : {}".format(result))
        self._home_result = result.home_state

    def feedback_callback(self, msg_feedback):
        rospy.loginfo("[UR5 1 Feedback] vg_state : {}".format(msg_feedback.feedback.vg_state))
        self._feedback = msg_feedback.feedback.vg_state

class UR5_2_client:

    def __init__(self):

        self._ur5_2_action_client = actionlib.ActionClient('/action_ur5_2_pick_place', msgUR5_2Action)

        self._goal_handles_2 = {}

        self._service_conveyot_belt = '/eyrc/vb/conveyor/set_power'
        rospy.wait_for_service(self._service_conveyot_belt)
        self._conveyot_belt = rospy.ServiceProxy(self._service_conveyot_belt, conveyorBeltPowerMsg)

        self._topic_ur5_2_feedback = '/action_ur5_2_pick_place/feedback'
        rospy.Subscriber(self._topic_ur5_2_feedback, msgUR5_2Feedback, self.feedback_callback)

        self._topic_logical_camera = '/eyrc/vb/logical_camera_2'
        rospy.Subscriber(self._topic_logical_camera, LogicalCameraImage, self.func_callback_logical_camera_topic)

        self._tl = [1, 1, 1]
        self._feedback = True
        self._home_result = True
        self._counter = 1
        self._color_map = {'red' : 1, 'yellow' : 2, 'green' : 3}

        self._ur5_2_action_client.wait_for_server()
        rospy.loginfo("UR5 2 Pick and Place - Action server is up, we can send new goals!")

    def func_callback_logical_camera_topic(self, model_data):
        num_model = len(model_data.models)
        
        if num_model >= 1:
            for i in range(num_model):
                model_obj = model_data.models[i]
                self._cur_model = model_obj.type
                if self._cur_model.startswith('packagen'):
                    self._tl[0] = model_obj.pose.position.x
                    self._tl[1] = model_obj.pose.position.y
                    self._tl[2] = model_obj.pose.position.z
                    # rospy.loginfo(self._cur_model)
                    # rospy.loginfo(self._tl)
                else:
                    self._tl = [1, 1, 1]
        else:
            self._tl = [1, 1, 1]

    def conveyor_belt_activator(self, set_power):
        result = self._conveyot_belt(set_power)
        if result.result == True:
            rospy.loginfo('\033[94m' + "Conveyor Belt started with power {}.".format(set_power) + '\033[0m')
        else:
            rospy.logerr("Conveyor Belt having some error, not started.")

    def start_cb_and_proccess_goal(self, color):

        color_map = self._color_map

        tl = self._tl
        self.conveyor_belt_activator(70)

        while not tl[1] < 0 :
            tl = self._tl

        self.conveyor_belt_activator(0)

        tl_pos = self._tl
        goal_handle_2 = self.send_goal(tl_pos[0], tl_pos[1], tl_pos[2], color)
        rospy.loginfo('\033[94m' + "Goal No. {} Sending.".format(self._counter) + '\033[0m')
        self._goal_handles_2[self._counter] = goal_handle_2
        self._counter += 1


    def send_goal(self, tl_x, tl_y, tl_z, pack_color):

        goal = msgUR5_2Goal(tl_x=tl_x, tl_y=tl_y, tl_z=tl_z, pack_color=pack_color)

        goal_handle = self._ur5_2_action_client.send_goal(goal, self.on_transition, None)

        rospy.loginfo('\033[94m' + "Goal has been sent." + '\033[0m')

        return goal_handle

    def on_transition(self, goal_handle):
        
        result = msgUR5_2Result()

        index = 0
        for i in self._goal_handles_2:
            if self._goal_handles_2[i] == goal_handle:
                index = i 
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )

        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo("[UR5 2 Result] Home State".format(result.home_state))

            if (result.home_state == True):
                self._home_result = True
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                self._home_result = False
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    def result_callback(self, result):
        rospy.loginfo("[Result] : {}".format(result.home_state))
        self._home_result = result.home_state

    def feedback_callback(self, msg_feedback):
        rospy.loginfo("[UR 2 Feedback] vg_state : {}".format(msg_feedback.feedback.vg_state))
        self._feedback = msg_feedback.feedback.vg_state

class Camera1:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)

        self._shelf_data = [[0 for x in range(3)] for x in range(4)] 

    def get_dominant_colour(self, arg_img):
        # setting values for base colors 
        b = arg_img[:, :, 0] 
        g = arg_img[:, :, 1] 
        r = arg_img[:, :, 2] 

        y = (b+g)/2

        # computing the mean 
        g_mean = int(np.mean(g))
        r_mean = int(np.mean(r))
        y_mean = int(np.mean(y))

        # displaying the most prominent color 
        if (g_mean > r_mean and g_mean > y_mean): 
            return 'green'
        elif (r_mean > g_mean and r_mean > y_mean):
            return 'red'
        elif (r_mean > y_mean and g_mean > y_mean): 
            return 'yellow'
        else: 
            return 'none'

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
                # print("[{}, {}]".format(row, col))
                # print("x = {}:{}, y = {}:{}".format(x1, x1 + width, y1, y1 + height))
                new_img = image[y1:y1 + height, x1:x1 + width]
                self._shelf_data[row][col] = self.get_dominant_colour(new_img)
                width_addition += width
                # cv2.imshow('image', new_img)
                # cv2.waitKey()
            height_addition += height
            width_addition = 0

        # rospy.loginfo(self._shelf_data)
        # return self._shelf_data

    def callback(self, data):
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

def main():

    rospy.init_node('node_t4_action_client')

    ur5_1 = UR5_1_client()
    ur5_2 = UR5_2_client()

    ic = Camera1()
    shelf_data = ic._shelf_data

    rospy.loginfo('\033[96m' + "Starting in 5 seconds." + '\033[0m')
    rospy.sleep(5)

    new_color_data = [[0 for x in range(3)] for x in range(4)]
    num0 = 0
    for i in range(12):
        new_color_data[num0][i%3] = shelf_data[num0][i%3]
        if i%3 == 2:
            num0 += 1

    rospy.loginfo("Color Data : {}".format(new_color_data))

    rospy.sleep(10)

    num = 0
    for i in range(9):

        while not ur5_2._feedback == True and ur5_1._home_result == True:
            rospy.sleep(0.001)

        goal_handle_1 = ur5_1.send_goal(num, i%3)
        ur5_1._goal_handles_1[i] = goal_handle_1
        rospy.loginfo("Goal #{} Sent..".format(i))

        rospy.sleep(0.2)

        while ur5_1._feedback == True:
            rospy.sleep(0.001)

        rospy.loginfo("Color : {}".format(new_color_data[num][i%3]))
        ur5_2.start_cb_and_proccess_goal(new_color_data[num][i%3])

        if i%3 == 2:
            num = num + 1

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()