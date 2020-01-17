#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# from test_msgs.msg import value
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        L = moveit_commander.MoveGroupCommander('L')
        R = moveit_commander.MoveGroupCommander('R')
                
        # 获取终端link的名称
        L_end_effector_link = L.get_end_effector_link()
        R_end_effector_link = R.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        L.set_pose_reference_frame(reference_frame)
        R.set_pose_reference_frame(reference_frame)
        
        # 当运动规划失败后，允许重新规划
        L.allow_replanning(True)
        R.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        L.set_goal_position_tolerance(0.001)
        L.set_goal_orientation_tolerance(0.001)
        L.set_max_acceleration_scaling_factor(1)
        L.set_max_velocity_scaling_factor(1) 
        R.set_goal_position_tolerance(0.001)
        R.set_goal_orientation_tolerance(0.001)
        R.set_max_acceleration_scaling_factor(1)
        R.set_max_velocity_scaling_factor(1)	
             
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        # RobotModel 中的 link 可设置相对坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     


        target_pose.pose.position.x = -0.168799
        target_pose.pose.position.y = -0.405586
        target_pose.pose.position.z = 1.24636
        target_pose.pose.orientation.x = 0.624507
        target_pose.pose.orientation.y = -0.331149
        target_pose.pose.orientation.z = 0.625156
        target_pose.pose.orientation.w = 0.330926
        L.set_start_state_to_current_state()
        L.set_pose_target(target_pose, L_end_effector_link)
        
        target_pose.pose.position.x = -0.670965
        target_pose.pose.position.y = -0.54653
        target_pose.pose.position.z = 1.75392
        target_pose.pose.orientation.x = -0.612427
        target_pose.pose.orientation.y = -0.353143
        target_pose.pose.orientation.z = 0.612832
        target_pose.pose.orientation.w = -0.353072
        R.set_start_state_to_current_state()
        R.set_pose_target(target_pose, R_end_effector_link)
        # 规划运动路径
        traj = L.plan()
        traj = R.plan()

        # # 按照规划的运动路径控制机械臂运动
        L.go()
        R.go()
        L.get_joint_value_target()
        R.get_joint_value_target()	
        rospy.sleep(2)
        

if __name__ == "__main__":
    MoveItIkDemo()

    
    
