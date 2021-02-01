#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.srv import *

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_goto_tf_waypoints', anonymous=True)

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')

        self._conveyot_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        self._vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        self._box_length = 0.15               # Length of the Package
        self._vacuum_gripper_width = 0.115    # Vacuum Gripper Width

        self._color_map = {'red' : 1, 'green' : 2, 'blue' : 3}

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            tl = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rt = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +
                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )
            return tl, rt

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
            return [0, 0, 0], [0, 0, 0, 0]

    def conveyor_belt_activator(self, set_power):
        result = self._conveyot_belt(set_power)
        if result:
            rospy.loginfo('\033[94m' + " Conveyor Belt started with power {}.".format(set_power) + '\033[0m')
        else:
            ospy.logerr(" Conveyor Belt having some error, not started.")

    def vacuum_activator(self, state):
        result = self._vacuum_gripper(state)
        if result:
            rospy.loginfo('\033[94m' + "Vacuum Griper Activated." + '\033[0m')
        else:
            rospy.logerr("Vacuum Griper Not Activated.")

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def placing_pack_in_bin(self, color):

        color_map = self._color_map
        pose_map = {1 : {'x' : 0.11, 'y' : 0.65, 'z' : 1.3}, 2 : {'x' : 0.75, 'y' : 0.03, 'z' : 1.3}, 3 : {'x' : 0.04, 'y' : -0.75, 'z' : 1.3}}
        
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

        self.vacuum_activator(True)
        self.go_to_pose(ur5_2_bin_pose)
        self.vacuum_activator(False)

    def starting_conveyor_belt_for_next_pack(self, next_pack):

        color_map = self._color_map

        self.conveyor_belt_activator(40)
        tl = [1, 1, 1]
        
        while not tl[0] < 0 :
            tl, rt = self.func_tf_print('ur5_wrist_3_link', 'logical_camera_2_packagen{}_frame'.format(color_map[next_pack]))
        
        self.conveyor_belt_activator(0)
        x = -tl[2]
        y = tl[0]
        z = -( tl[1] - 0.195 )
        
        rospy.loginfo('\033[94m' + "Translating from current position to Red Package." + '\033[0m')
        self.ee_cartesian_translation(x, y, z)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():
    ur5 = CartesianPath()

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    # Teams may use this info in Tasks

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    # Waiting till all the packages have been loaded
    rospy.sleep(5)

    # 1. Go to Home Position
    ur5.go_to_pose(ur5_2_home_pose)
    rospy.sleep(3)

    # Going for red package
    ur5.starting_conveyor_belt_for_next_pack('red')
    ur5.placing_pack_in_bin('red')

    # Going for green package
    ur5.go_to_pose(ur5_2_home_pose)
    ur5.starting_conveyor_belt_for_next_pack('green')
    ur5.placing_pack_in_bin('green')

    # Going for blue package
    ur5.go_to_pose(ur5_2_home_pose)
    ur5.starting_conveyor_belt_for_next_pack('blue')
    ur5.placing_pack_in_bin('blue')

    ur5.go_to_pose(ur5_2_home_pose)

    del ur5


if __name__ == '__main__':
    main()
