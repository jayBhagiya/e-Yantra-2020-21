#! /usr/bin/env python

import rospy
import sys
import copy

import actionlib

import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_task3.msg import msgUR5Action, msgUR5Goal, msgUR5Feedback, msgUR5Result

class UR5_print_tf:

    def __init__(self):
        rospy.init_node('node_ur5_action_client')

        self._ur5_action_client = actionlib.ActionClient('/action_ur5_pick_place', msgUR5Action)
        self._goal_handles = {}

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        self._service_conveyot_belt = '/eyrc/vb/conveyor/set_power'
        rospy.wait_for_service(self._service_conveyot_belt)
        self._conveyot_belt = rospy.ServiceProxy(self._service_conveyot_belt, conveyorBeltPowerMsg)

        self._feedback = True
        self._result = True
        self._counter = 1
        self._color_map = {'red' : 1, 'green' : 2, 'blue' : 3}

        self._ur5_action_client.wait_for_server()
        rospy.loginfo("Pick and Place - Action server is up, we can send new goals!")


    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            tl = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rt = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            # rospy.loginfo(  "\n" +
            #                 "Translation: \n" +
            #                 "x: {} \n".format(trans.transform.translation.x) +
            #                 "y: {} \n".format(trans.transform.translation.y) +
            #                 "z: {} \n".format(trans.transform.translation.z) +
            #                 "\n" +
            #                 "Orientation: \n" +
            #                 "x: {} \n".format(trans.transform.rotation.x) +
            #                 "y: {} \n".format(trans.transform.rotation.y) +
            #                 "z: {} \n".format(trans.transform.rotation.z) +
            #                 "w: {} \n".format(trans.transform.rotation.w) )
            return tl, rt

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rospy.logerr("TF error")
            return [1, 1, 1], [1, 1, 1, 1]

    def conveyor_belt_activator(self, set_power):
        result = self._conveyot_belt(set_power)
        if result.result == True:
            rospy.loginfo('\033[94m' + "Conveyor Belt started with power {}.".format(set_power) + '\033[0m')
        else:
            rospy.logerr("Conveyor Belt having some error, not started.")

    def start_cb_and_proccess_goal(self, color):

        color_map = self._color_map

        tl, rt = self.func_tf_print('logical_camera_2_frame', 'logical_camera_2_packagen{}_frame'.format(color_map[color]))
        self.conveyor_belt_activator(30)

        while not tl[1] < 0 :
            tl, rt = self.func_tf_print('logical_camera_2_frame', 'logical_camera_2_packagen{}_frame'.format(color_map[color]))

        self.conveyor_belt_activator(0)

        while not self._result:
            rospy.sleep(0.5)

        if self._result == True:
            tl_pos, rt_pos = self.func_tf_print('ur5_wrist_3_link', 'logical_camera_2_packagen{}_frame'.format(color_map[color]))
            goal_handle = self.send_goal(tl_pos[0], tl_pos[1], tl_pos[2], color)
            rospy.loginfo('\033[94m' + "Goal No. {} Sending.".format(self._counter) + '\033[0m')
            self._goal_handles[self._counter] = goal_handle
            self._counter += 1
            self._result = False


    def send_goal(self, tl_x, tl_y, tl_z, pack_color):

        goal = msgUR5Goal(tl_x=tl_x, tl_y=tl_y, tl_z=tl_z, pack_color=pack_color)

        goal_handle = self._ur5_action_client.send_goal(goal, self.on_transition, None)

        rospy.loginfo('\033[94m' + "Goal has been sent." + '\033[0m')

        return goal_handle

    def on_transition(self, goal_handle):
        
        result = msgUR5Result()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
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
            rospy.loginfo(result.final_state)

            if (result.final_state == True):
                self._result = True
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                self._result = False
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    def result_callback(self, result):
        rospy.loginfo("[ Result ] : {}".format(result.final_state))
        self._result = result.final_state

    def feedback_callback(self, feedback):
        rospy.loginfo("[ Feedback ] : {}".format(feedback))
        self._feedback = feedback.cur_state

def main():
    ur5_tf = UR5_print_tf()

    rospy.loginfo('\033[96m' + "Starting in 5 seconds." + '\033[0m')
    rospy.sleep(5)

    ur5_tf.start_cb_and_proccess_goal('red')
    rospy.sleep(2)

    ur5_tf.start_cb_and_proccess_goal('green')
    rospy.sleep(2)

    ur5_tf.start_cb_and_proccess_goal('blue')
    rospy.sleep(2)

    rospy.spin()


if __name__ == '__main__':
    main()