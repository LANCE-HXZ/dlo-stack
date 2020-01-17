#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from test_msgs.msg import value
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
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
        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(2)
             
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
       
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.3
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.3
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
          
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        
        # # 按照规划的运动路径控制机械臂运动
        arm.go()
        arm.get_joint_value_target()	
        rospy.sleep(2)
        
        # # 控制机械臂终端向xia移动5cm
        # arm.shift_pose_target(2, 0.01, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
        # arm.get_current_pose()
        # # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
           
        # # 控制机械臂回到初始化位置
        # arm.set_named_target('goal')
        # arm.go() 
        # rospy.sleep(5)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(2)
        

        
        
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
   
#         pub_l_2 = rospy.Publisher('/left_joint2_controller/command', value , queue_size=10)

#     #pub_l_3 = rospy.Publisher('/left_joint3_controller/command', value , queue_size=10)
    
#         pub_l_4 = rospy.Publisher('/left_joint4_controller/command', value , queue_size=10)

#         pub_l_5 = rospy.Publisher('/left_joint5_controller/command', value , queue_size=10)

#         pub_l_6 = rospy.Publisher('/left_joint6_controller/command', value , queue_size=10)

#         pub_l_7 = rospy.Publisher('/left_joint7_controller/command', value , queue_size=10)

#         pub_l_8 = rospy.Publisher('/left_joint8_controller/command', value , queue_size=10)

#         pub_l_9 = rospy.Publisher('/left_joint9_controller/command', value , queue_size=10)
        
#     #更新频率是1hz
#         rate = rospy.Rate(0.01)
#    # float64 data = 3
   
    
#     #while not rospy.is_shutdown():
#         #计算距离
#         #rospy.loginfo('Talker: GPS: x=%f ,y= %f',x,y)
#   #zhuaqu
        
#         pub_l_7.publish(3)
#         pub_l_8.publish(3)
#         pub_l_4.publish(4)
#         rospy.sleep(1)
#         pub_l_9.publish(4)
#         pub_l_7.publish(3.5) 
#         pub_l_8.publish(3.5)
#         #rospy.sleep(0.5)      
#         pub_l_6.publish(2)
#         rospy.sleep(1)
#         pub_l_5.publish(3)
#         rospy.sleep(3)
#         #pub_l_2.publish(0)
#         joint_positions = [-1.721767, 0.8288, 0.905476, 1.774476, -0.913854, -0.754855,-0.265988]
#         arm.set_joint_value_target(joint_positions)
#         arm.go()
#         rospy.sleep(1)
        
#         #rospy.sleep(1)
#  #songkai
    
#         rospy.sleep(2)
#         pub_l_4.publish(0)
#         pub_l_5.publish(0)
#         pub_l_6.publish(0)
#         #rospy.sleep(1)
#         pub_l_9.publish(0)
#         pub_l_7.publish(0) 
#         pub_l_8.publish(0)
#        #arm.set_named_target('goal')
#         #arm.go() 
#         #rospy.sleep(2)
# 	arm.set_named_target('houtui')
#         arm.go() 
#         rospy.sleep(2)
        
        
#         #arm.set_start_state_to_current_state()
#         #arm.shift_pose_target(2, 0.1, end_effector_link)
#         #arm.go()
#        # pub_l_2.publish(0)
#         arm.set_named_target('home')
#         arm.go()
#         pub_l_4.publish(4)
#         rospy.sleep(2)
	#pub_e.publish(0)	
        #pub_4.publish(3.5)
	#pub_3.publish(3.5)
        
       
        #rate.sleep()
      # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
