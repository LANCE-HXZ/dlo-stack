#!/usr/bin/env python
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# import rospy
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
 
# def talker():
#     bridge = CvBridge()
#     pub = rospy.Publisher('image_topic', Image, queue_size=1)
#     rospy.init_node('py_node', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         input_img = cv2.imread("/home/lance/Data/0-RGBimg/1.png")
#         rospy.loginfo("img opened")
#         pub.publish(bridge.cv2_to_imgmsg(input_img, "bgr8"))
#         rate.sleep()
 
# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass


# from __future__ import print_function
 
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
 
class image_converter:
 
  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
 
    self.bridge = CvBridge()
    self.sub = rospy.Subscriber("boxes_topic", BoundingBoxes, self.callback)
#   def read_img(self):
#       rate = rospy.Rate(1) # 10hz
#       while not rospy.is_shutdown():
#         cv_image = cv2.imread("/home/lance/Data/0-RGBimg/1.png")
#         rospy.loginfo("img opened")
#         self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#         rate.sleep()
  def callback(self,data):
    print(data)
    # try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #   print(e)
 
    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)
 
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)
 
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
 
def main(args):
  ic = image_converter()
  rospy.init_node('listener_node', anonymous=True)
  try:
    # ic.read_img()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)