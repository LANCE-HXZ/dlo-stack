import cv2
from cv_bridge import CvBridge, CvBridgeError
import darknet as dn
from ctypes import *
import math
import random
import roslib
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
import sensor_msgs
import sys
import time
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

net = dn.load_net("darknet_ros/darknet_ros/yolo_network_config/cfg/yolov3-voc.cfg".encode('utf-8'), 
    "model_weights/darknet_weights/yolov3-voc_final.weights".encode('utf-8'), 0)
meta = dn.load_meta("darknet_ros/darknet_ros/yolo_network_config/cfg/voc.data".encode('utf-8'))  
print('\n', 'Darknet model loaded')
# def array_to_image(arr):
#     arr = arr.transpose(2,0,1)
#     c = arr.shape[0]
#     h = arr.shape[1]
#     w = arr.shape[2]
#     arr = (arr/255.0).flatten()
#     data = dn.c_array(dn.c_float, arr)
#     im = dn.IMAGE(w,h,c,data)
#     return im

class image_converter:
 
  def __init__(self):
    self.boxes_pub = rospy.Publisher("boxes_topic", BoundingBoxes, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("rgb_topic", String, self.callback)
#   
  def callback(self,data):
    print('\n', 'Predicting')
    # try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #   print(e)
    # cv_image = cv2.copyMakeBorder(cv_image, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=[85, 120, 68])
    # cv_image = cv2.imread("pic_buffer/R.png")
    # cv2.imwrite("pic_buffer/R.png", cv_image)
    
    # im = array_to_image(cv_image)
    # dn.rgbgr_image(im)
    # # print(dn.detect(net, meta, im))
    r = dn.detect(net, meta, b"pic_buffer/1_R.png")
    # print("===")
    img = cv2.imread("pic_buffer/1_R.png")
    c_index = 0
    # e_index = 65  # 'A'
    for item in r:   #画方框和文字
        ileft = int(item[2][0]-item[2][2]*0.5)##矩形点坐标##item[2][0]交点x轴坐标
        iup = int(item[2][1]-item[2][3]*0.5)
        iright = int(item[2][0]+item[2][2]*0.5)
        ibtm = int(item[2][1]+ item[2][3]*0.5)
        persent = str(round(item[1],2))
        # name = item[0].decode('utf-8')
        text = persent
        loca = str(int(item[2][0]))+" "+str(int(item[2][1]))+"\n"+str(int(item[2][2]))+" "+str(int(item[2][3]))+"\n"
        if 'endpoint' == item[0].decode('utf-8'):
            # text = chr(e_index)
            # e_index+=1
            # text = "E-" + persent
            cv2.rectangle (img, (ileft, iup), (iright, ibtm), (0, 0, 255), 2)
            cv2.circle(img, (int(item[2][0]), int(item[2][1])), 3, (0, 0, 255), -1)
            cv2.putText(img, text, (ileft, iup-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            text = str(c_index)
            c_index+=1
            # text = "C-" + persent
            cv2.rectangle (img, (ileft, iup), (iright, ibtm), (255, 0, 0), 2)
            cv2.circle(img, (int(item[2][0]), int(item[2][1])), 3, (255, 0, 0), -1)
            cv2.putText(img, text, (ileft, iup-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            # print (item)
    
    cv2.imwrite("pic_buffer/2_D.png", img)
    
    boxes = BoundingBoxes()
    print('\n', 'Predict result:')
    for i in range(len(r)):
        box = BoundingBox()
        print('    ', r[i][0], r[i][1]*100, '%')
        box.Class = r[i][0].decode('utf-8')
        box.probability = r[i][1]
        box.id = i
        box.xmin = int(r[i][2][0])
        box.ymin = int(r[i][2][1])
        box.xmax = int(r[i][2][2])
        box.ymax = int(r[i][2][3])
        boxes.bounding_boxes.append(box)
        # if b'endpoint' == r[i][0]:
        #     print('\t', r[i][0], r[i][1]*100, '%')
        print('    ', int(r[i][2][0]), int(r[i][2][1]), int(r[i][2][2]), int(r[i][2][3]))
        # if b'cross' == r[i][0]:
        #     print('\t', r[i][0], r[i][1]*100, '%')
        #     # print('\t', r[i][2])
    print('\n', 'Darknet waiting for rgb_img')
    time.sleep(0.5)
    try:
      self.boxes_pub.publish(boxes)
    except CvBridgeError as e:
      print(e)

if __name__ == "__main__":
    #net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    #im = load_image("data/wolf.jpg", 0, 0)
    #meta = load_meta("cfg/imagenet1k.data")
    #r = classify(net, meta, im)
    #print r[:10]
    
    ic = image_converter()
    rospy.init_node('yolo_node')
    print('\n', 'Darknet waiting for rgb_img')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    
    # r = detect(net, meta, "darknet_ros/darknet/data/1.bmp".encode('utf-8'))
    # print('\n', 'Predict result:')
    # for i in range(len(r)):
    #     if b'endpoint' == r[i][0]:
    #         print(r[i][0], r[i][1]*100, '%')
    #         print('\t', r[i][2])
    

