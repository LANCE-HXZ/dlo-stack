#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import time
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class image_converter:
 
  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
    self.bridge = CvBridge()
  def read_img(self):
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/455.png") #D 单根5交叉
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/D1235.png") #D 双根5交叉
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/D1119.png") #D 双根奇数端点
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T054.png") #D 三根9交叉
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/69.png") #T
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/T0726.png") #I (已解决)遍历终点找不到端点, 轮廓提取的问题
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/467.png") #IX 单根6交叉  // 浮点数例外???
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0533.png") #I
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0673.png") #I
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0782.png") #D
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/1930.png") #D
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/D0684.png") #Q

    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/89.bmp")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/125.bmp")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/700.bmp")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/1238.bmp")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/1589.bmp")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/D3_image_235.png")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/T3_image_466.png")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/T7_image_936.png")
    # cv_image = cv2.imread("/home/hxz/Workspaces/dlo_stack/src/kuka_ws/pic_buffer/T9_image_1175.png")


    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/D0736.png") #Q 遍历路径错误, 图像细化问题
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0920.png") #D 交叉点距离过近
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0715.png") #I yolo端点识别缺失
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0828.png") #I 二分类预测错误
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0877.png") # 二分类预测错误
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0879.png") # 浮点数例外, 可能是交叉点距离过近
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0870.png") #Q 二分类预测错误
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0825.png") #D 提取轮廓出现断点
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0840.png") # 二分类预测错误
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/24.png") # 图像细化问题, 跳跃时找不到对面点
    # cv_image = cv2.imread("/home/lance/Data/0-RGBimg/T0859.png") # 提取轮廓出现断点

    filename = "20201123233331"
    # 实验合并图,先切割出原图
    cv_image = cv2.imread("/home/lance/Data/1124all/" + filename + ".png")
    cv_image = cv_image[0:480, 0:640]
    # 不带边框的原图
    # cv_image = cv2.imread("/home/lance/Workspaces/hxz_ws/src/pic_buffer/1.png")
    # edge = 60
    # cv_image = cv2.resize(cv_image, (600,450))
    # cv_image = cv2.copyMakeBorder(cv_image, edge, edge, edge, edge, cv2.BORDER_CONSTANT, value=[85, 120, 68])
    # cv_image = cv2.resize(cv_image, (640,400))
    # cv2.imwrite("pic_buffer/R.png", cv_image)
    # sp = cv_image.shape
    # cv_image = cv_image[15:sp[0]-25, 0:sp[1]]
    # cv_image = cv2.resize(cv_image, (640,320))
    # cv_image = cv_image[0:sp[0]/2, 0:sp[1]]
    # cv_image = cv_image[sp[0]/4:sp[0]*3/4, 0:sp[1]]
    # cv_image = cv_image[sp[0]/2:sp[0], 0:sp[1]]
    
    rospy.loginfo("rgb_img has been published")
    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # pub_msg = "1"
    # self.image_pub.publish(pub_msg)
      # rate = rospy.Rate(0.1) # hz
      # while not rospy.is_shutdown():
      #   cv_image = cv2.imread("/home/lance/Data/0-RGBimg/1.png")
      #   rospy.loginfo("Image get")
      #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      #   rate.sleep()
 
def main(args):
  ic = image_converter()
  rospy.init_node('virtual_camera_node', anonymous=True) # 加入一个随机数是节点名唯一
  time.sleep(1)
  try:
    ic.read_img()
    # rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  # cv2.AllWindows()
 
if __name__ == '__main__':
    main(sys.argv)