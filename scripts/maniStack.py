#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ KB-1492 ]
# Author List:		[ vidya vepoori, ch pranathi]
# Filename:			maniStack.py
# Functions:		print_pose_ee( ), print_joint_angles( ), set_joint_angles( ),
#                   go_to_predefined_pose( )
# Nodes:		    pub:'/move_group/display_planned_path'

# importing ros stuff


import rospy
import sys
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
import message_filters
import numpy as np
import warnings

##############################################################
pose_values = None
xyz = []
quat_list = []
y_pose = []
r_pose = []
ye = None
re = None
###############################################################

# class for controling ur5


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('manipulation arm', anonymous=True)

        self._planning_group = "arm"
        self._eef_planning_group = "gripper"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._eef_group = moveit_commander.MoveGroupCommander(
            self._eef_planning_group)

        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._planning_frame = self._eef_group.get_planning_frame()

        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # printing pose of end-effector

    def print_pose_ee(self):

        global quat_list, pose_values, xyz

        pose_values = self._group.get_current_pose().pose

        x = pose_values.position.x
        y = pose_values.position.y
        z = pose_values.position.z

        xyz = [x, y, z]

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "q_x: {}\n".format(q_x) +
                      "q_y: {}\n".format(q_y) +
                      "q_z: {}\n".format(q_z) +
                      "q_w: {}\n".format(q_w) +
                      '\033[0m')

    # printing pose of arm

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')

    # giving joint angle for arm manipulation

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # for going to predefined pose for ee

    def go_to_predefined_ee_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._eef_group.set_named_target(arg_pose_name)
        plan = self._eef_group.plan()
        self._eef_group.go(wait=True)

    # for going to predefined pose for arm group
    def go_to_predefined_arm_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        self._group.go(wait=True)

    # ee to vaild pose

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
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def callback(y_data, r_data):

    global y_pose, r_pose, ye, re, quat_list, xyz

    ur5 = Ur5Moveit()

#______________________ yellow fruit pose wrt ebot base_________________________#

    ye = y_data
    y_x = ye.position.x
    y_y = ye.position.y
    y_z = ye.position.z
    y_roll = ye.orientation.x
    y_pitch = ye.orientation.y
    y_yaw = ye.orientation.z
    y_w = ye.orientation.w
    y_pose = [y_x, y_y, y_z, y_roll, y_pitch, y_yaw, y_w]
    print("y_pose", y_pose)

#______________________ red fruit pose wrt ebot base____________________________#

    re = r_data
    r_x = re.position.x
    r_y = re.position.y
    r_z = re.position.z
    r_roll = re.orientation.x
    r_pitch = re.orientation.y
    r_yaw = re.orientation.z
    r_w = re.orientation.w
    r_pose = [r_x, r_y, r_z, r_roll, r_pitch, r_yaw, r_w]
    print("r_pose", r_pose)

#______________giving pose to got to yellow fruit pose___________________#

    y_1 = geometry_msgs.msg.Pose()
    y_1.position.x = y_x
    y_1.position.y = y_y/2.5
    y_1.position.z = y_z
    y_1.orientation.x = 3.31854795803658e-05
    y_1.orientation.y = -0.999962112333367
    y_1.orientation.z = -0.0072225703885678404
    y_1.orientation.w = 0.004858731675766901

    y_2 = geometry_msgs.msg.Pose()
    y_2.position.x = y_x
    y_2.position.y = y_y / 1.7
    y_2.position.z = y_z - 0.03
    y_2.orientation.x = 3.31854795803658e-05
    y_2.orientation.y = -0.999962112333367
    y_2.orientation.z = -0.0072225703885678404
    y_2.orientation.w = 0.004858731675766901

  #______________giving pose to got to red fruit pose____________________#

    r_3 = [math.radians(71),
           math.radians(31),
           math.radians(-67),
           math.radians(214),
           math.radians(-158),
           math.radians(180)]

    r_1 = geometry_msgs.msg.Pose()
    r_1.position.x = 0.121794
    r_1.position.y = 0.235638915564
    r_1.position.z = 1.352879
    r_1.orientation.x = -0.026171504812217326
    r_1.orientation.y = 0.9996959245270841
    r_1.orientation.z = 0.01632619337790168
    r_1.orientation.w = 0.0061103959126186625

    r_2 = geometry_msgs.msg.Pose()
    r_2.position.x = r_x
    r_2.position.y = r_y / 1.7
    r_2.position.z = r_z
    r_2.orientation.x = 0.017434692531414327
    r_2.orientation.y = 0.9996959245270841
    r_2.orientation.z = 0.01722619337790168
    r_2.orientation.w = 0.0027107640953955014


#__________________________execution____________________________#

    while not rospy.is_shutdown():

        if not sys.warnoptions:
            warnings.simplefilter("ignore")

        ur5.print_pose_ee()
        ur5.print_joint_angles()
        rospy.sleep(1)


        ur5.go_to_pose(y_1)
        ur5.go_to_pose(y_2)
        ur5.go_to_predefined_ee_pose("close")
        ur5.go_to_pose(y_1)
        ur5.go_to_predefined_arm_pose("yellow_basket")
        ur5.go_to_predefined_ee_pose("open")
        ur5.go_to_predefined_arm_pose("left_side_90")
        rospy.sleep(1)
        ur5.set_joint_angles(r_3)
        rospy.sleep(1)
        ur5.go_to_pose(r_2)
        ur5.go_to_predefined_ee_pose("close")
        ur5.go_to_predefined_arm_pose("red_basket")
        ur5.go_to_predefined_ee_pose("open")
        

        rospy.spin()

    # del ur5


def main():

    ur5 = Ur5Moveit()

    now = rospy.get_time()

    ur5.go_to_predefined_arm_pose("left_side_90")

    sub_y_pose = message_filters.Subscriber("/yellow_pose", Pose)
    sub_r_pose = message_filters.Subscriber("/red_pose", Pose)

    clbk_pose = message_filters.ApproximateTimeSynchronizer(
        [sub_y_pose, sub_r_pose], queue_size=10, slop=0.5, allow_headerless=True)
    clbk_pose.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    main()
