#! /usr/bin/env python

import rospy
import sys
import copy
import threading

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pkg_vb_sim.srv import vacuumGripper
from pkg_task4.msg import msgUR5_2Action, msgUR5_2Goal, msgUR5_2Feedback, msgUR5_2Result

class UR5_2_pick_place:

    # Constructor
    def __init__(self):

        self._robot_ns = '/ur5_2'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, 
            robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._ur5_2_action_server = actionlib.ActionServer('/action_ur5_2_pick_place', msgUR5_2Action,
                                                 self.func_on_goal,
                                                 self.func_on_cancel,
                                                 auto_start=False)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self._service_vacuum_gripper = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2'
        rospy.wait_for_service(self._service_vacuum_gripper)
        self._vacuum_gripper = rospy.ServiceProxy(self._service_vacuum_gripper, vacuumGripper)

        self._box_length = 0.15               # Length of the Package
        self._vacuum_gripper_width = 0.115    # Vacuum Gripper Width

        self._color_map = {'red' : 1, 'yellow' : 2, 'green' : 3}

        self._ur5_2_action_server.start()
        rospy.loginfo('\033[94m' + " >>> Init done. UR5_2 Action server started." + '\033[0m')

    def vacuum_activator(self, state):
        result = self._vacuum_gripper(state)
        # rospy.loginfo('\033[94m' + "Vacuum Griper Result. {}".format(result) + '\033[0m')
        if result.result == True:
            rospy.loginfo('\033[94m' + "Vacuum Griper of UR5_2 is Activated." + '\033[0m')
            return True
        else:
            rospy.loginfo("Vacuum Griper of UR5_2 is DeActivated.")
            return False

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts) )

        return flag_success

    def go_to_home_pose(self):
        
        box_length = self._box_length               # Length of the Package
        vacuum_gripper_width = self._vacuum_gripper_width    # Vacuum Gripper Width
        delta = vacuum_gripper_width + (box_length/2)  # 0.19

        ur5_2_home_pose = geometry_msgs.msg.Pose()
        ur5_2_home_pose.position.x = -0.8
        ur5_2_home_pose.position.y = 0
        ur5_2_home_pose.position.z = 1 + delta
        # This to keep EE parallel to Ground Plane
        ur5_2_home_pose.orientation.x = -0.5
        ur5_2_home_pose.orientation.y = -0.5
        ur5_2_home_pose.orientation.z = 0.5
        ur5_2_home_pose.orientation.w = 0.5

        rospy.loginfo('\033[94m' + "Going to Home pose." + '\033[0m')
        state = self.hard_go_to_pose(ur5_2_home_pose, 5)

        return state

    def go_to_pack_pose(self, pose_x, pose_y, pose_z):

        ur5_2_pack_pose = geometry_msgs.msg.Pose()
        ur5_2_pack_pose.position.x = pose_x 
        ur5_2_pack_pose.position.y = pose_y
        ur5_2_pack_pose.position.z = pose_z
        # This to keep EE parallel to Ground Plane
        ur5_2_pack_pose.orientation.x = -0.5
        ur5_2_pack_pose.orientation.y = -0.5
        ur5_2_pack_pose.orientation.z = 0.5
        ur5_2_pack_pose.orientation.w = 0.5

        rospy.loginfo('\033[94m' + "Going from home pose to pack pose." + '\033[0m')
        state = self.go_to_pose(ur5_2_pack_pose)

        return state

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

        color = obj_msg_goal.pack_color
        color_map = self._color_map
        pose_map = {1 : {'x' : 0.11, 'y' : 0.65, 'z' : 1.3}, 2 : {'x' : 0.75, 'y' : 0.03, 'z' : 1.3}, 3 : {'x' : 0.04, 'y' : -0.75, 'z' : 1.3}}

        ur5_2_feedback = msgUR5_2Feedback()

        # Bin Pose
        ur5_2_bin_pose = geometry_msgs.msg.Pose()
        ur5_2_bin_pose.position.x = pose_map[color_map[color]]['x']
        ur5_2_bin_pose.position.y = pose_map[color_map[color]]['y']
        ur5_2_bin_pose.position.z = pose_map[color_map[color]]['z']
        # This to keep EE parallel to Ground Plane
        ur5_2_bin_pose.orientation.x = -0.5
        ur5_2_bin_pose.orientation.y = -0.5
        ur5_2_bin_pose.orientation.z = 0.5
        ur5_2_bin_pose.orientation.w = 0.5

        x = -0.8 + obj_msg_goal.tl_z               # Camrera famre wrt world is (-0.8) in x direction, so pack farme wrt to world will be (-0.8) + z
        y = obj_msg_goal.tl_y                # Camera frame and world both are in same in y direction
        z = 2 + 0.195 - obj_msg_goal.tl_x   # Camrera frame wrt world is 2 in z direction, so pcak frame wrt to world will be (2) + (-x) [z(world) = -x(camrea frame)]

        rospy.loginfo('\033[94m' + "UR5_2 Going from current position to {} Package.".format(obj_msg_goal.pack_color) + '\033[0m')
        self.go_to_pack_pose(x, y, z)

        vg_state = self.vacuum_activator(True)
        ur5_2_feedback.vg_state = vg_state
        self._ur5_2_action_server.publish_feedback(goal_status, ur5_2_feedback)

        rospy.loginfo('\033[94m' + "UR5_2 Going from {} package to {} bin.".format(color, color) + '\033[0m')
        self.hard_go_to_pose(ur5_2_bin_pose, 5)

        vg_state = self.vacuum_activator(False)
        ur5_2_feedback.vg_state = vg_state
        self._ur5_2_action_server.publish_feedback(goal_status, ur5_2_feedback)

        bin_result = self.go_to_home_pose()

        ur5_2_result = msgUR5_2Result()
        ur5_2_result.home_state = bin_result
        if bin_result:
            rospy.loginfo("Succeeded..")
            goal_handle.set_succeeded(ur5_2_result)
        else:
            rospy.loginfo("Goal Failed. Aborting...")
            goal_handle.set_aborted(ur5_2_result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def func_on_cancel(self, obj_msg_goal):
        rospy.loginfo('\033[94m' + "Received Cancel Request." + '\033[0m')
        goal_id = obj_msg_goal.get_goal_id()

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class UR5_2_pick_place Deleted." + '\033[0m')


def main():

    rospy.init_node('node_ur5_2_pick_place_action_server', anonymous=True)

    ur5_2 = UR5_2_pick_place()

    # Going to Home Position
    ur5_2.go_to_home_pose()
    rospy.sleep(3)

    rospy.spin()


if __name__ == '__main__':
    main()