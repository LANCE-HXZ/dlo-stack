#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 要用python2运行
import rospy, sys
import roslib
import std_msgs
import sensor_msgs
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# from test_msgs.msg import value
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class image_converter:
 
  def __init__(self):
    self.class_list_pub = rospy.Publisher("crop_class_topic", std_msgs.msg.Int64, queue_size=100)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("rgb_topic", sensor_msgs.msg.Image, self.rgb_callback)
    self.crop_sub = rospy.Subscriber("crop_topic", sensor_msgs.msg.Image, self.callback)
    normalize = T.Normalize(mean=[0.485, 0.456, 0.406],  # 数据转换 , 每个维度的值转换到规定区间内
                                    std=[0.229, 0.224, 0.225])
    self.transforms = T.Compose([
                    T.Resize(224),
                    T.CenterCrop(224),
                    T.ToTensor(),
                    normalize
                ])
    self.results = []
  def rgb_callback(self,msg):
    self.results = []
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
 
    # (rows,cols,channels) = cv_image.shape
    pil_img = PIL.Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    data = self.transforms(pil_img)
    # print(pil_img)
    class_result = predict(data, self.results)
    print(class_result)
    print('Crop_predict_net waiting for crop_img')    
    # cv2.imshow("1", bw_img)
    # cv2.waitKey(1)
 
    try:
      self.class_list_pub.publish(class_result[-1])
    except CvBridgeError as e:
      print(e)

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
        KUKA.allow_replanning(True)
        # rospy.sleep(5)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        KUKA.set_goal_position_tolerance(0.001)
        KUKA.set_goal_orientation_tolerance(0.001)
        KUKA.set_max_acceleration_scaling_factor(0.03)
        KUKA.set_max_velocity_scaling_factor(0.03) 	

        # KUKA.set_named_target('KUKA_HOME')
        # KUKA.go()
             
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        # RobotModel 中的 link 可设置相对坐标系
        L_target_pose = PoseStamped()
        L_target_pose.header.frame_id = reference_frame
        L_target_pose.header.stamp = rospy.Time.now()     
        R_target_pose = PoseStamped()
        R_target_pose.header.frame_id = reference_frame
        R_target_pose.header.stamp = rospy.Time.now() 

        L_target_pose.pose.position.x -= 0.01
        R_target_pose.pose.position.x -= 0.01
        L_target_pose.pose.position.y -= 0.01
        R_target_pose.pose.position.y -= 0.01
        L_target_pose.pose.position.z -= 0.01
        R_target_pose.pose.position.z -= 0.01
        KUKA.set_start_state_to_current_state()
        KUKA.set_pose_target(L_target_pose, L_end_effector_link)
        KUKA.set_pose_target(R_target_pose, R_end_effector_link)
        traj = KUKA.plan()
        KUKA.go()
        KUKA.get_joint_value_target()

if __name__ == "__main__":
    ic = image_converter()
    rospy.init_node('crop_predict_node')
    print('\n', 'Crop_predict_net waiting for crop_img')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    MoveItIkDemo()

    
    
