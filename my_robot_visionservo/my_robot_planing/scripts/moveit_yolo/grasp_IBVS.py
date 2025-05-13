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
        self.arm.set_named_target("try")
        self.arm.go(wait=True)

        # 关节控制
        # joints = [0,  -1.04, +1.04, 0, 0, 0]
        # self.movej(joints)
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
        """
        实现机械臂的关节运动
        :param joint: 目标关节角度 [joint1, joint2, joint3, joint4, joint5, joint6]
        """
        target = self.arm.get_current_joint_values()
        # 设置目标关节角度
        for i in range(6):
            target[i] = joint[i]
        self.arm.set_joint_value_target(target)
        state = self.arm.go(wait=True)
        # plan = self.arm.plan()
        # self.arm.execute(plan, wait=True)

    def moveC(self, target_pose, qua, steps=50):
        """
        实现机械臂的半圆运动
        :param target_pose: 目标点 [x, y, z]
        :param qua: 姿态 [roll, pitch, yaw]
        :param steps: 半圆分段数，默认为50
        """
        waypoints = []
        current_pose = self.arm.get_current_pose().pose  # 获取当前机械臂末端位姿

        # 计算起点和终点的中点作为圆心
        center = [
            (current_pose.position.x + target_pose[0]) / 2,
            (current_pose.position.y + target_pose[1]) / 2,
            (current_pose.position.z + target_pose[2]) / 2
        ]

        # 计算半径
        radius = math.sqrt(
            (current_pose.position.x - center[0]) ** 2 +
            (current_pose.position.y - center[1]) ** 2 +
            (current_pose.position.z - center[2]) ** 2
        )

        radiusx = current_pose.position.x - center[0]
        # print(radiusx)
        # 将欧拉角转换为四元数
        quaa = quaternion_from_euler(qua[0], qua[1], qua[2])

        # 生成半圆路径点
        for i in range(steps + 1):
            angle = math.pi * i / steps  # 从 0 到 π 分段生成角度
            x = center[0] + radiusx * math.cos(angle)
            y = center[1] + radiusx * math.sin(angle)
            z = center[2]  # 保持 z 坐标不变
            # print("x",x,"y",y)
            # 设置路径点的位姿
            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = quaa[0]
            pose.pose.orientation.y = quaa[1]
            pose.pose.orientation.z = quaa[2]
            pose.pose.orientation.w = quaa[3]
            # 添加路径点
            waypoints.append(pose.pose)

        # 规划笛卡尔路径
        
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,   # 路径点列表
            0.01,        # 步长，控制路径的平滑程度
            0.0          # 跳跃阈值，0表示不允许跳跃
        )

        # 执行路径
        if fraction == 1.0:  # 确保路径规划成功
            self.arm.execute(plan, wait=True)
        else:
            rospy.logwarn("moveC: 半圆路径规划失败，完成率为 {:.2f}".format(fraction))

    def movedp(self, dpose, qua):
        """
        在当前末端姿态的基础上，移动dpose
        :param dpose: 位移 [dx, dy, dz]
        :param qua: 姿态 [roll, pitch, yaw]
        """
        current_pose = self.arm.get_current_pose().pose  # 获取当前末端位姿

        # 计算目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()

        target_pose.pose.position.x = current_pose.position.x + dpose[0]
        target_pose.pose.position.y = current_pose.position.y + dpose[1]
        target_pose.pose.position.z = current_pose.position.z # - 0.2 # dpose[2]

        # 保持当前姿态
        target_pose.pose.orientation = current_pose.orientation

        # 设置目标位姿并执行
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_link)
        traj = self.arm.plan()
        self.arm.execute(traj, wait=True)

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


def get_rotation_matrix(q):
    # in TF, it is xyzw
    # xyzw方向转旋转矩阵
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    rot = [[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], 
            [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x],
            [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]]
    return rot

def get_CameraFrame_Pos(u, v, depthValue):
    # 图像系转相机系（u、v图像坐标，depthValue对应坐标的深度值）
    # fx fy cx cy为相机内参
    K = 0.05
    fx = 831.3844
    fy = 831.3844
    cx = 480
    cy = 270

    x = float(depthValue) + 0.025
    y = -K * float(u - cx) / fx
    z = -K * float(v - cy)  / fy
    # print('xyz',[x, y, z, 1])

    return [z, y, x]

findObject = False
depthImg = np.array(0)
depthOK = False

def BoundingBoxCallBack(data):
    # yolo检测的回调函数
    global findObject, u, v, graspObject

    if not findObject:
        # 待抓取目标为空，则请求输入抓取目标
        if graspObject == '':
            rospy.loginfo('object detected, please input the object you want to grasp:(cube, trangle, star)\n')
        objectGrasp = []
        for dat in data.bounding_boxes:
            # 遍历所有目标，种类与待抓取目标相同则保存目标中心位置
            if graspObject == dat.Class:
                objectGrasp.append([dat.Class, (dat.xmin + dat.xmax)/2, (dat.ymin + dat.ymax)/2])
                # print(dat.xmin, dat.xmax, dat.ymin ,dat.ymax)
                # objectGrasp.append([dat.Class, (dat.xmin + dat.xmax)/2, dat.ymax])
        if objectGrasp != []:
            # 如果待抓取目标存在，则在目标列表随机选择一个返回
            rospy.loginfo('{} found, begin grasp!!!'.format(graspObject))
            _, u, v = random.choice(objectGrasp)
            findObject = True
        else:
            rospy.loginfo('The object you want to grasp is absent!!!')

def depthCallback(data):
    # 深度图像回调函数
    global depthImg,depthOK
    depthOK = False
    depthImg = CvBridge().imgmsg_to_cv2(data, data.encoding)
    depthOK = True

endDown=[0,3.14,0]

if __name__ == "__main__":
    # 深度图像和检测框对应topic名称
    depth_topic = 'camera/depth/image_raw'
    BoundingBox_topic = '/yolov5/BoundingBoxes'
    # 初始化ros节点
    rospy.init_node('grasp_yolo')
    # 实例化抓取控制类
    grasp_demo = graspDemo()
    grasp_demo.go_arm_named('try')
    u = 960
    v = 540
    graspObject = 'trangle'
    # 订阅深度图像和检测框对应topic
    rospy.Subscriber(BoundingBox_topic, BoundingBoxes, BoundingBoxCallBack, queue_size=1) 
    rospy.Subscriber(depth_topic, Image, depthCallback, queue_size=1) 
    rospy.loginfo('get subscriber!!!')

    
    while not rospy.is_shutdown():
        if findObject and depthOK:
            rospy.loginfo('grasp is going!!!')

            u, v = int(u), int(v) # 防止检测的坐标越界
            # 图像坐标转相机系坐标
            cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u])
            grasp_demo.movedp(cameraFrame_pos, endDown)

            rospy.sleep(0.3)
            findObject = False
    grasp_demo.shutDowm()  
