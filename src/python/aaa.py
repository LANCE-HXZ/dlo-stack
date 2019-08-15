#!/usr/bin/env python
# -*- coding: utf-8 -*-
#####
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
           
        # 初始化需要使用move group控制的机械臂中的right_arm group
        arm = moveit_commander.MoveGroupCommander('KUKA')
        endeffector = moveit_commander.MoveGroupCommander('KUKA')
   
           
        # 获取终端link的名称
        end_effector_link = endeffector.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
      
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('KUKA_HOME')
        arm.go()
        rospy.sleep(2)

if __name__ == "__main__":
    MoveItIkDemo()
