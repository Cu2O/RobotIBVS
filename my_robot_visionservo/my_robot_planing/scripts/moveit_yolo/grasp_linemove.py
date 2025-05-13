#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import moveit_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

import rospy
import cv2
import sys
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ctypes import *
import math
from math import pi
import numpy as np
from threading import Thread
from std_srvs.srv import Empty

from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
import random

class graspDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
            
        self.reference_frame = "base"
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.end_link = self.arm.get_end_effector_link()
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        pose_target = geometry_msgs.msg.Pose()
        robot=moveit_commander.RobotCommander()
        
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.02)

        # moveit初始位置
        #self.arm.set_named_target("try")
        #self.arm.go(wait=True)

        # 关节控制
        joints = [0,  -1.04, +1.04, 0, 0, 0]
        self.movej(joints)
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)

    def movep(self,pose,qua):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()

        quaa = quaternion_from_euler(qua[0], qua[1], qua[2])

        target_pose.pose.position.x = pose[0]
        target_pose.pose.position.y = pose[1]
        target_pose.pose.position.z = pose[2]
        target_pose.pose.orientation.x = quaa[0]
        target_pose.pose.orientation.y = quaa[1]
        target_pose.pose.orientation.z = quaa[2]
        target_pose.pose.orientation.w = quaa[3]
            
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_link)
        traj = self.arm.plan()
        self.arm.execute(traj,wait=True)
    
    def movel(self, pose, qua):
        """
        实现机械臂的直线运动
        :param pose: 目标位置 [x, y, z]
        :param qua: 目标姿态 [roll, pitch, yaw]
        """
        waypoints = []
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()

        # 将欧拉角转换为四元数
        quaa = quaternion_from_euler(qua[0], qua[1], qua[2])

        # 设置目标位姿
        target_pose.pose.position.x = pose[0]
        target_pose.pose.position.y = pose[1]
        target_pose.pose.position.z = pose[2]
        target_pose.pose.orientation.x = quaa[0]
        target_pose.pose.orientation.y = quaa[1]
        target_pose.pose.orientation.z = quaa[2]
        target_pose.pose.orientation.w = quaa[3]

        # 添加目标位姿到路径点
        waypoints.append(target_pose.pose)

        # 规划直线路径
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,   # 路径点列表
            0.01,        # 步长，控制路径的平滑程度
            0.0          # 跳跃阈值，0表示不允许跳跃
        )

        # 执行路径
        if fraction == 1.0:  # 确保路径规划成功
            self.arm.execute(plan, wait=True)
        else:
            rospy.logwarn("movel: 直线路径规划失败，完成率为 {:.2f}".format(fraction))
    
    def movej(self, joint):
        target = self.arm.get_current_joint_values()
        # 设置目标关节角度
        for i in range(6):
            target[i] = joint[i]
        self.arm.set_joint_value_target(target)
        state = self.arm.go(wait=True)
        # plan = self.arm.plan()
        # self.arm.execute(plan, wait=True)

    def shutDowm(self):
        # moveit关闭
        moveit_commander.roscpp_initializer.roscpp_shutdown()
        moveit_commander.os._exit(0)
        rospy.loginfo('grasp complete!!!')
    
    def go_arm_named(self, place = 'ready'):
        # 移动至命名的pose
        self.arm.set_named_target(place)
        self.arm.go(wait=True)

    def go_gripper_named(self, place = 'open'):
        self.gripper.set_named_target(place)
        self.gripper.go(wait=True)

endDown=[0,3.14,0]

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('grasp_yolo')

    # 实例化抓取控制类
    grasp_demo = graspDemo()

    # 定义两个目标点的位姿
    point1 = [0.3, 0.0, 0.4]  # 第一个点的位置 [x, y, z]
    orientation1 = [0, 3.14, 0]  # 第一个点的姿态 [roll, pitch, yaw]

    point2 = [0.3, 0.2, 0.4]  # 第二个点的位置 [x, y, z]
    orientation2 = [0, 3.14, 0]  # 第二个点的姿态 [roll, pitch, yaw]

    rospy.loginfo("Starting loop between two points...")

    try:
        while not rospy.is_shutdown():
            # 移动到第一个点
            rospy.loginfo("Moving to point 1...")
            grasp_demo.movel(point1, orientation1)

            # 在第一个点停留一段时间
            rospy.sleep(2)

            # 移动到第二个点
            rospy.loginfo("Moving to point 2...")
            grasp_demo.movel(point2, orientation2)

            # 在第二个点停留一段时间
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down...")

    # 关闭机械臂控制
    grasp_demo.shutDowm()
