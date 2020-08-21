#!/usr/bin/env python
# -*- coding: utf-8 -*-
from model import *
import torch
import roslib
import rospy
from std_msgs.msg import String
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import torch.backends.cudnn as cudnn
import cv2
from torchvision import transforms
from torch.autograd import Variable
import numpy as np
import PIL
from PIL import Image
import os
# 自定义灰度界限，大于这个值为黑色，小于这个值为白色
threshold = 128
table = []
for i in range(256):
    if i < threshold:
        table.append(1)  # 白底黑线
    else:
        table.append(0)
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
modelname = 'bwnet600'
# modelname = 'dangenmynet600'
# modelname = 'mynet380'
# modelname = 'mynet600'
cudnn.benchmark = True
model = torch.nn.DataParallel(UNet16(num_filters=32, pretrained='vgg')).cuda()
# model = torch.nn.DataParallel(UNet16(num_filters=32, pretrained='vgg')).cpu()

# if flag == 't':
#     path = './data2m1100/train/Image/'
# else:
#     path = './data2m1100/vali/Image/'

path = '/home/lance/Workspaces/hxz_ws/pic_buffer/'
state_path = '/home/lance/Workspaces/hxz_ws/src/dlo/python/ros_mynet/model/' + modelname + '.ckpt'
# state = torch.load(state_path, map_location=torch.device('cpu'))
state = torch.load(state_path)
# print(len(state))
# model.load_state_dict(torch.load(state_path, map_location=torch.device('cpu')))
model.load_state_dict(torch.load(state_path))
print('\n', 'Bw_predict_net model loaded')
# img_num = ['153']
# for i in range(len(img_num)):
def predict(img):
    # img_name = path + img_num[i] + '.png'
    # img = Image.open(img_name)
    img = transforms.ToTensor()(img)
    img = img.unsqueeze(0)
    # img_arr = Variable(img, requires_grad=True).cuda(async=True)
    # img_arr = img.cuda()
    img_arr = img
    out_overlap, out_gradient, Y = model(img_arr)
    imgo = transforms.ToPILImage()(out_overlap.data[0].cpu())
    imgg = transforms.ToPILImage()(out_gradient.data[0].cpu())
    imgY = transforms.ToPILImage()(Y.data[0].cpu())
    # imgo = transforms.ToPILImage()(out_overlap.data[0].cuda())
    # imgg = transforms.ToPILImage()(out_gradient.data[0].cuda())
    # imgY = transforms.ToPILImage()(Y.data[0].cuda())
    imgo = cv2.cvtColor(np.asarray(np.uint8(imgo)), cv2.COLOR_RGB2BGR)
    imgg = cv2.cvtColor(np.asarray(np.uint8(imgg)), cv2.COLOR_RGB2BGR)
    imgY = cv2.cvtColor(np.asarray(np.uint8(imgY)), cv2.COLOR_RGB2BGR)
    # imgo = cv2.copyMakeBorder(imgo, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=0)
    # imgg = cv2.copyMakeBorder(imgg, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=0)
    # imgY = cv2.copyMakeBorder(imgY, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=0)
    img_name = path + 'O.png'
    # imgo.save(img_name)
    cv2.imwrite(img_name, imgo)
    print('out overlap save done')
    img_name = path + 'G.png'
    # imgg.save(img_name)
    cv2.imwrite(img_name, imgg)
    print('out gradient save done')
    img_name = path + 'Y.png'
    # imgY.save(img_name)
    cv2.imwrite(img_name, imgY)
    print('out Y save done')
    return imgg, imgo, imgY
    # img_ = np.array(imgY)
    # rows, cols, h = img_.shape
    # img = transforms.ToTensor()(img_)
    # # print(img)
    # # 红底
    # for r in range(rows):
    #     for c in range(cols):
    #         if img[1][r][c] > 0 or img[2][r][c] > 0:
    #             img[1][r][c] = 0
    #         else:
    #             img[1][r][c] = 1

    # img = transforms.ToPILImage()(img)
    # # img.save('predict1.bmp')
    # # 模式L”为灰色图像，它的每个像素用8个bit表示，0表示黑，255表示白，其他数字表示不同的灰度。
    # Img = img.convert('L')
    # # Img.save("label/test1.jpg")
    # # 图片二值化
    # photo = Img.point(table, '1')
    # photo.save(path + 'B.png')
    # # photo = photo.astype(np.uint8)
    # # print(photo)
    # bw_img = cv2.cvtColor(np.asarray(np.uint8(photo)), cv2.COLOR_GRAY2BGR)
    # # photo.show()
    # return bw_img


class image_converter:
 
  def __init__(self):
    self.image_pub = rospy.Publisher("bw_topic", String, queue_size=1000)
 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("rgb_topic", String, self.callback)
#   
  def callback(self,data):
    # try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #   print(e)
    # print('\n', 'Predicting') 
    cv_image = cv2.imread(path + "R.png")
    # (rows,cols,channels) = cv_image.shape
    pil_img = PIL.Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

    # print(pil_img)
    bw_imgg, bw_imgo, bw_imgy = predict(pil_img)
    print('\n', 'Senting bw_img')    
    # cv2.imshow("1", bw_img)
    # cv2.waitKey(1)

    print('\n', 'Bw_predict_net waiting for rgb_img')
    pub_msg = "1"
    try:
      self.image_pub.publish(pub_msg)
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(bw_imgo, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('bw_img_node')
  print('\n', 'Bw_predict_net waiting for rgb_img')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  # cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)