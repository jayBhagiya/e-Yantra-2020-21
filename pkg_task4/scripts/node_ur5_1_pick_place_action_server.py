#! /usr/bin/env python

import rospy
import sys
import copy
import threading
import os
import math
import time
import rospkg
import yaml

import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pkg_vb_sim.srv import vacuumGripper
from pkg_task4.msg import msgUR5_1Action, msgUR5_1Goal, msgUR5_1Feedback, msgUR5_1Result

class UR5_1_pick_place:

    """docstring for UR5_2_pick_place"""
    def __init__(self):

        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, 
            robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._group.set_planning_time(20)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Attribute to store computed trajectory by the planner 
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        self._ur5_1_action_server = actionlib.ActionServer('/action_ur5_1_pick_place', msgUR5_1Action,
                                                 self.func_on_goal,
                                                 self.func_on_cancel,
                                                 auto_start=False)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        self._service_vacuum_gripper = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1'
        rospy.wait_for_service(self._service_vacuum_gripper)
        self._vacuum_gripper = rospy.ServiceProxy(self._service_vacuum_gripper, vacuumGripper)

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        self._ur5_1_action_server.start()
        rospy.loginfo('\033[94m' + "UR5_1 Action server started" + '\033[0m')

        rospy.loginfo('\033[94m' + ">>> UR5_1_pick_place init done." + '\033[0m')

    def vacuum_activator(self, state):
        result = self._vacuum_gripper(state)
        # rospy.loginfo('\033[94m' + "Vacuum Griper Result. {}".format(result) + '\033[0m')
        if result.result == True:
            rospy.loginfo('\033[94m' + "Vacuum Griper of UR5_1 Activated." + '\033[0m')
            return True
        else:
            rospy.loginfo("Vacuum Griper of UR5_1 DeActivated.")
            return False

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

    def func_on_goal(self, obj_msg_goal):

        rospy.loginfo('\033[94m' + "Received a Goal from Client." + '\033[0m')
        rospy.loginfo(obj_msg_goal)

        obj_msg_goal.set_accepted()

        thread = threading.Thread( name='Worker', target=self.func_process_goal, args=(obj_msg_goal,) )
        thread.start()

    def func_process_goal(self, goal_handle):

        obj_msg_goal = goal_handle.get_goal()
        goal_id = goal_handle.get_goal_id()
        goal_status = goal_handle.get_goal_status()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        ur5_1_feedback = msgUR5_1Feedback()

        row = obj_msg_goal.row
        col = obj_msg_goal.column

        vg_state = self.vacuum_activator(True)
        ur5_1_feedback.vg_state = vg_state
        self._ur5_1_action_server.publish_feedback(goal_status, ur5_1_feedback)

        rospy.loginfo('\033[94m' + "Translating Package from {},{} to on Conveyer belt.".format(row, col) + '\033[0m')
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'pose{}{}_to_place.yaml'.format(row, col), 5)

        vg_state = self.vacuum_activator(False)
        ur5_1_feedback.vg_state = vg_state
        self._ur5_1_action_server.publish_feedback(goal_status, ur5_1_feedback)

        ur5_1_result = msgUR5_1Result()

        rospy.loginfo('\033[94m' + "Translating from straightUp position to {},{} Package.".format(row, col) + '\033[0m')
        belt_result = self.moveit_hard_play_planned_path_from_file(self._file_path, 'place_to_home.yaml', 5)
        if row == 2 and col == 2:
            pass
        else:
            if col%3 == 2:
                col = -1
                row = row + 1
            belt_result = self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_pose{}{}.yaml'.format(row, col + 1), 5)

        ur5_1_result.home_state = belt_result
        if belt_result:
            rospy.loginfo("Succeeded...")
            goal_handle.set_succeeded(ur5_1_result)
        else:
            rospy.loginfo("Goal Failed. Aborting...")
            goal_handle.set_aborted(ur5_1_result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def func_on_cancel(self, obj_msg_goal):
        rospy.loginfo('\033[94m' + "Received Cancel Request." + '\033[0m')
        goal_id = obj_msg_goal.get_goal_id()

def main():

    rospy.init_node('node_ur5_1_pick_place_action_server', anonymous=True)

    ur5_1 = UR5_1_pick_place()

    rospy.loginfo("Going to 0,0 Position..")
    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'init_to_home.yaml', 5)
    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'home_to_pose00.yaml', 5)

    rospy.spin()


if __name__ == '__main__':
    main()