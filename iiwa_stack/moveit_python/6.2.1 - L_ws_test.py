#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, numpy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
numpy.set_printoptions(threshold=numpy.inf)

class MoveItIkDemo:    
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')   

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('L') 

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'frame'
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
        current = PoseStamped()
        current.header.frame_id = reference_frame
        current.header.stamp = rospy.Time.now()
        current = arm.get_current_pose()

        target_pose.pose.position.z = 0.73
        target_pose.pose.orientation.x = -0.572507
        target_pose.pose.orientation.y = -0.0213573
        target_pose.pose.orientation.z = 0.819444
        target_pose.pose.orientation.w = -0.0170479

        N = [[2]*43 for i in range(34)]
        x_start = -1.05
        # x_start = -0.5
        y_start = -1.15
        # y_start = -0.8

        for i in range(34):
            for j in range(43):
                last_x = "%.2f" %current.pose.position.x
                last_y = "%.2f" %current.pose.position.y
                xx = x_start + 0.05 * j
                yy = y_start + 0.05 * i
                target_pose.pose.position.x = xx
                target_pose.pose.position.y = yy
                arm.set_start_state_to_current_state()
                arm.set_pose_target(target_pose, end_effector_link)
                arm.go()
                print " "
                current = arm.get_current_pose()
                now_x = "%.2f" %current.pose.position.x
                now_y = "%.2f" %current.pose.position.y
                if last_x == now_x and last_y == now_y:
                    N[i][j] = 0
                else:
                    N[i][j] = 1
                for k in range(34):
                    print "%.2f" %(y_start + 0.05 * k),'\t',
                    for l in range(43):
                        print N[k][l],
                    print " "
                print '\n',"current",'\t',now_x,'\t',now_y
                print "lasttime",'\t',last_x,'\t',last_y,'\n'

if __name__ == "__main__":
    MoveItIkDemo()


    
    
