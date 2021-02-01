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
from pkg_task3.msg import msgUR5Action, msgUR5Goal, msgUR5Feedback, msgUR5Result

class UR5_CartesianPath:

    # Constructor
    def __init__(self):

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

        self._ur5_action_server = actionlib.ActionServer('/action_ur5_pick_place', msgUR5Action,
                                                 self.func_on_goal,
                                                 self.func_on_cancel,
                                                 auto_start=False)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self._service_vacuum_gripper = '/eyrc/vb/ur5_1/activate_vacuum_gripper'
        rospy.wait_for_service(self._service_vacuum_gripper)
        self._vacuum_gripper = rospy.ServiceProxy(self._service_vacuum_gripper, vacuumGripper)

        self._box_length = 0.15               # Length of the Package
        self._vacuum_gripper_width = 0.115    # Vacuum Gripper Width

        self._color_map = {'red' : 1, 'green' : 2, 'blue' : 3}

        self._ur5_action_server.start()
        rospy.loginfo('\033[94m' + " >>> Init done. Action server started." + '\033[0m')

    def vacuum_activator(self, state):
        result = self._vacuum_gripper(state)
        # rospy.loginfo('\033[94m' + "Vacuum Griper Result. {}".format(result) + '\033[0m')
        if result.result == True:
            rospy.loginfo('\033[94m' + "Vacuum Griper Activated." + '\033[0m')
            return True
        else:
            rospy.loginfo("Vacuum Griper DeActivated.")
            return False

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

    def go_to_home_pose(self):
        
        box_length = self._box_length               # Length of the Package
        vacuum_gripper_width = self._vacuum_gripper_width    # Vacuum Gripper Width
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

        rospy.loginfo('\033[94m' + "Going to Home pose." + '\033[0m')
        state = self.go_to_pose(ur5_2_home_pose)

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
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        color = obj_msg_goal.pack_color
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

        x = -obj_msg_goal.tl_z               # Converting reference from EE to World x(World) = -z(EE)
        y = obj_msg_goal.tl_x                # Converting reference from EE to World y(World) = x(EE)
        z = -( obj_msg_goal.tl_y - 0.195 )   # Converting reference from EE to World z(World) = -y(EE)  and subtracting width of gripper and half of the package

        rospy.loginfo('\033[94m' + "Translating from current position to {} Package.".format(obj_msg_goal.pack_color) + '\033[0m')
        self.ee_cartesian_translation(x, y, z)

        vg_state = self.vacuum_activator(True)

        rospy.loginfo('\033[94m' + "Translating from {} package to {} bin.".format(color, color) + '\033[0m')
        self.go_to_pose(ur5_2_bin_pose)

        vg_state = self.vacuum_activator(False)

        bin_result = self.go_to_home_pose()

        ur5_result = msgUR5Result()
        if bin_result:
            ur5_result.final_state = bin_result
            goal_handle.set_succeeded(ur5_result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def func_on_cancel(self, obj_msg_goal):
        rospy.loginfo('\033[94m' + "Received Cancel Request." + '\033[0m')
        goal_id = obj_msg_goal.get_goal_id()

    # Destructor
    # def __del__(self):
    #     moveit_commander.roscpp_shutdown()
    #     rospy.loginfo(
    #         '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():

    rospy.init_node('node_ur5_pick_place_action_server', anonymous=True)

    # Waiting till all the packages are loaded
    rospy.sleep(5)

    ur5 = UR5_CartesianPath()

    # Going to Home Position
    ur5.go_to_home_pose()
    rospy.sleep(3)

    rospy.spin()


if __name__ == '__main__':
    main()