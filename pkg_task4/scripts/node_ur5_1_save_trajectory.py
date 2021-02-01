#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from pkg_vb_sim.srv import vacuumGripper
from std_srvs.srv import Empty

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_ur5_1_save_trajectory', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._group.set_planning_time(22)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._service_vacuum_gripper = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1'
        rospy.wait_for_service(self._service_vacuum_gripper)
        self._vacuum_gripper = rospy.ServiceProxy(self._service_vacuum_gripper, vacuumGripper)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def vacuum_activator(self, state):
        result = self._vacuum_gripper(state)
        # rospy.loginfo('\033[94m' + "Vacuum Griper Result. {}".format(result) + '\033[0m')
        if result.result == True:
            rospy.loginfo('\033[94m' + "Vacuum Griper of UR5_1 Activated." + '\033[0m')
            return True
        else:
            rospy.loginfo("Vacuum Griper of UR5_1 DeActivated.")
            return False

    def wait_for_state_update(self, arg_box_name, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = arg_box_name
        scene = self._scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_box(self, arg_box_name, position, timeout=4):
        box_name = arg_box_name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()

        box_pose.header.frame_id = self._robot.get_planning_frame()
        box_pose.header.stamp = rospy.Time.now()

        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]
        box_pose.pose.orientation.w = 1.0

        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

        return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

    def attach_box(self, arg_box_name, timeout=4):
        box_name = arg_box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        grasping_group = group_names[0]
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_name, box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, arg_box_name, timeout=4):
        box_name = arg_box_name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, arg_box_name, timeout=4):
        box_name = arg_box_name
        scene = self._scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_name, box_is_attached=False, box_is_known=False, timeout=timeout)

    def add_all_boxes(self):

        pack_pose = [[0.28, -0.41, 1.91], [0.00, -0.41, 1.91], [-0.28, -0.41, 1.91],
                     [0.28, -0.41, 1.65], [0.00, -0.41, 1.65], [-0.28, -0.41, 1.65],
                     [0.28, -0.41, 1.42], [0.00, -0.41, 1.42], [-0.28, -0.41, 1.42]]
        num = 0
        for i in range(0, 9):
            rospy.sleep(0.5)
            pack_name = 'packagen{}{}'.format(num, i%3)
            rospy.loginfo("Adding packagen{}{}".format(num, i%3))
            if i%3 == 2:
                num = num + 1
            self.add_box(pack_name, pack_pose[i])

    def go_to_home_pose(self):

        lst_home_joint_angles = [math.radians(-180),
                          math.radians(-90),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

        rospy.loginfo('\033[94m' + "Going to Home pose(Straight UP)." + '\033[0m')
        state = self.hard_set_joint_angles(lst_home_joint_angles, 5)

        return state

    def go_to_place_pose(self):

        lst_place_joint_angles = [math.radians(0),
                          math.radians(-133),
                          math.radians(-58),
                          math.radians(-79),
                          math.radians(90),
                          math.radians(0)]

        rospy.loginfo('\033[94m' + "Going to Place pose(to place pack on belt)." + '\033[0m')
        state = self.hard_set_joint_angles(lst_place_joint_angles, 10)

        return state

    def get_set_of_angles(self, row, col):

        pack_angles = [[161, -105, -14, -61, 20, 90], [119, -91, -25, -63, 62, 90], [58, -122, 19, -77, 122, 90],
                       [160, -79, -86, -15, 20, 90], [122, -63, -97, -20, 60, 90], [55, -84, -83, -13, 126, 90],
                       [-62, -102, 96, 6, 118, -90], [120, -61, -131, 12, 62, 90], [56, -85, -115, 20, 126, 90]]

        mapping = {'[0, 0]' : 0, '[0, 1]' : 1, '[0, 2]' : 2, '[1, 0]' : 3, '[1, 1]' : 4, '[1, 2]' : 5, '[2, 0]' : 6, '[2, 1]' : 7, '[2, 2]' : 8}
        co_ordinates = '[{}, {}]'.format(row, col)

        lst_pack_joints = [math.radians(pack_angles[mapping[co_ordinates]][0]),
                           math.radians(pack_angles[mapping[co_ordinates]][1]),
                           math.radians(pack_angles[mapping[co_ordinates]][2]),
                           math.radians(pack_angles[mapping[co_ordinates]][3]),
                           math.radians(pack_angles[mapping[co_ordinates]][4]),
                           math.radians(pack_angles[mapping[co_ordinates]][5])]

        return lst_pack_joints

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
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit('ur5_1')

    row = 2
    col = 0

    ur5.add_all_boxes()
    rospy.sleep(2)

    # **1** For saving init to home position.

    rospy.loginfo('\033[94m' + "Going to Home position." + '\033[0m')
    ur5.go_to_home_pose()

    file_name = 'init_to_home.yaml'
    file_path = ur5._file_path + file_name

    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)

    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(1)

    # **2** For saving home to pack position.

    rospy.loginfo('\033[94m' + "Translating from straightUp position to {},{} Package.".format(row, col) + '\033[0m')
    ur5.hard_set_joint_angles(ur5.get_set_of_angles(row, col), 5)

    file_name = 'home_to_pose{}{}.yaml'.format(row, col)
    file_path = ur5._file_path + file_name

    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)

    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(1)

    # **3** For saving pack to place position.

    vg_state = ur5.vacuum_activator(True)
    ur5.attach_box('packagen{}{}'.format(row, col))
    rospy.sleep(0.5)

    rospy.loginfo('\033[94m' + "Translating Package from {},{} to on Conveyer belt.".format(row, col) + '\033[0m')
    ur5.go_to_place_pose()

    file_name = 'pose{}{}_to_place.yaml'.format(row, col)
    file_path = ur5._file_path + file_name

    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)

    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(1)

    vg_state = ur5.vacuum_activator(False)
    ur5.detach_box('packagen{}{}'.format(row, col))
    ur5.remove_box('packagen{}{}'.format(row, col))

    # **4** For saving place to home position.

    rospy.loginfo('\033[94m' + "Going to Home position." + '\033[0m')
    ur5.go_to_home_pose()
    file_name = 'place_to_home.yaml'
    file_path = ur5._file_path + file_name

    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)

    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(1)

    del ur5

if __name__ == '__main__':
    main()