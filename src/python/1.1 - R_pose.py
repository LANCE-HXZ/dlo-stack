#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        KUKA = moveit_commander.MoveGroupCommander('KUKA')
        L = moveit_commander.MoveGroupCommander('L')
        R = moveit_commander.MoveGroupCommander('R')
                
        # 获取终端link的名称
        L_end_effector_link = L.get_end_effector_link()
        R_end_effector_link = R.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'frame'
        KUKA.set_pose_reference_frame(reference_frame)
        
        # 当运动规划失败后，允许重新规划
        KUKA.allow_replanning(False)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        KUKA.set_goal_position_tolerance(0.001)
        KUKA.set_goal_orientation_tolerance(0.001)
        KUKA.set_max_acceleration_scaling_factor(1)
        KUKA.set_max_velocity_scaling_factor(1) 	
             
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        # RobotModel 中的 link 可设置相对坐标系
        L_target_pose = PoseStamped()
        L_target_pose.header.frame_id = reference_frame
        L_target_pose.header.stamp = rospy.Time.now()     
        R_target_pose = PoseStamped()
        R_target_pose.header.frame_id = reference_frame
        R_target_pose.header.stamp = rospy.Time.now() 

        L_target_pose.pose.position.x = 0.345136
        L_target_pose.pose.position.y = -0.433041
        L_target_pose.pose.position.z = -0.571319
        L_target_pose.pose.orientation.x = -0.515366
        L_target_pose.pose.orientation.y = 0.856776
        L_target_pose.pose.orientation.z = 0.0030054
        L_target_pose.pose.orientation.w = -0.0179874

        R_target_pose.pose.position.x = -0.2
        R_target_pose.pose.position.y = -0.1
        R_target_pose.pose.position.z = 0.73
        R_target_pose.pose.orientation.x = 0.000475125
        R_target_pose.pose.orientation.y = 0.605187
        R_target_pose.pose.orientation.z = 0.000511703
        R_target_pose.pose.orientation.w = 0.796083
          
        KUKA.set_start_state_to_current_state()
        KUKA.set_pose_target(L_target_pose, L_end_effector_link)
        KUKA.set_pose_target(R_target_pose, R_end_effector_link)
        # 规划运动路径
        # traj = KUKA.plan()

        # # 按照规划的运动路径控制机械臂运动
        KUKA.go()
        # KUKA.get_joint_value_target()	
        

if __name__ == "__main__":
    MoveItIkDemo()

    
    
