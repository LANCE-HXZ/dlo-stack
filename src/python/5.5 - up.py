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
        arm = moveit_commander.MoveGroupCommander('R')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'frame'
        arm.set_pose_reference_frame(reference_frame)
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.0000001)
        arm.set_goal_orientation_tolerance(0.0000001)
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1) 	
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        
        arm.set_start_state_to_current_state()
        
        # 向上0.1
        target_pose = arm.get_current_pose()
        target_pose.pose.position.z += 0.1
        
        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        traj = arm.plan()
        arm.go()
        arm.get_joint_value_target()		

if __name__ == "__main__":
    MoveItIkDemo()

    
    
