#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import termios

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
        reference_frame = 'camera_frame'
        arm.set_pose_reference_frame(reference_frame)
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(False)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.0001)
        arm.set_goal_orientation_tolerance(0.0001)
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1) 	
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        
        arm.set_start_state_to_current_state()
        movex,movey = 0,0
        move = 0.1
        move_ = 0.05
        print("%.2f" %target_pose.pose.position.x,"%.2f" %target_pose.pose.position.y)
        while True:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
            try :
                ch = sys.stdin.read(1)
                if ch=='D' or ch=='C' or ch=='A' or ch=='B' or ch=='a' or ch=='d' or ch=='w' or ch=='s':
                    if ch=='D':
                        movex=-move
                    elif ch=='C': 
                        movex=+move 
                    elif ch=='A': 
                        movey=+move 
                    elif ch=='B': 
                        movey=-move
                    elif ch=='a':
                        movex=-move_
                    elif ch=='d': 
                        movex=+move_ 
                    elif ch=='w': 
                        movey=+move_        
                    elif ch=='s': 
                        movey=-move_

                    target_pose = arm.get_current_pose()
                    target_pose.pose.position.x+=movex
                    target_pose.pose.position.y+=movey 
                    movex=0
                    movey=0
                    arm.set_start_state_to_current_state()
                    arm.set_pose_target(target_pose, end_effector_link)
                    arm.go()
                    print("%.2f" %(target_pose.pose.position.x+0.05),"%.2f" %(-target_pose.pose.position.y-0.37))
                    print("Waiting input:")
                elif ch=='q':
                    break
                else:
                    pass
            finally :
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    MoveItIkDemo()


    
    
