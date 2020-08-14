# -*- coding: utf-8 -*-
# 黑白互换
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import os
from torchvision import transforms
from PIL import Image
from datetime import datetime
import torch

# # 打印gpu是否可用
# os.environ["CUDA_VISIBLE_DEVICES"] = ""
# print('cuda' if torch.cuda.is_available() else 'cpu')

# threshold = 128
# table = []
# for i in range(256):
#     if i < threshold:
#         table.append(0)
#     else:
#         table.append(1)
#
#
# transform = transforms.Compose([
#     transforms.ToTensor(),
#     transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
#
# img_name = 'test/89.bmp'
# img = cv2.imread(img_name)
# # print(img)
# img = Image.open(img_name)
# img = transforms.ToTensor()(img)
# for i in range(320):
#     for j in range(640):
#         img[0][i][j] = 0 if img[0][i][j] else 1
#
# print(img[0][110])
# img = transforms.ToPILImage()(img)
# img = img.convert('L')
# photo = img.point(table, '1')
# photo.save('test/test.bmp')


# 矩阵取反
# array = [0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, \
#          1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, \
#          0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, \
#          1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, \
#          1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
#          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
#          1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, \
#          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
#          0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, \
#          1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, \
#          0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, \
#          1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, \
#          1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
#          1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, \
#          1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, \
#          1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0]
#
# for i in range(len(array)):
#     if array[i] == 0:
#         array[i] = 1
#     else:
#         array[i] = 0
#
# print(array)


# 图像腐蚀
# !/usr/bin/env python
# encoding: utf-8

# import cv2
# import numpy as np
#
# img = cv2.imread('test/test.bmp', 0)
#
# # OpenCV定义的结构元素
# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
#
# # 显示原始二值图像
# cv2.imshow("original", img)
#
# # # 腐蚀图像
# # eroded = cv2.erode(img, kernel)
# # # 显示腐蚀后的图像
# # cv2.imshow("Eroded Image", eroded);
#
# # 膨胀图像
# dilated = cv2.dilate(img, kernel)
# # 显示膨胀后的图像
# cv2.imshow("Dilated Image", dilated);
# # # 原图像
# # cv2.imshow("Origin", img)
# #
# # # NumPy定义的结构元素
# # NpKernel = np.uint8(np.ones((3, 3)))
# # Nperoded = cv2.erode(img, NpKernel)
# # # 显示腐蚀后的图像
# # cv2.imshow("Eroded by NumPy kernel", Nperoded);
#
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# # 读黑白图, 背景为 0, 前景为 255
# prev_time = datetime.now()
# img = cv2.imread('test/test.png', cv2.IMREAD_GRAYSCALE)
# # ret, img = cv2.threshold(img, 127, 255, 1)
# cur_time = datetime.now()
# print(cur_time - prev_time)
# print(640*255)
# print(sum(img[0]))
# print(sum(img[108]))


# # 黑线转白线
# import cv2
# from PIL import Image
#
# gt_name = './test/test.png'
# img = cv2.imread(gt_name, -1)
# print(img)
# row, col = img.shape
# for i in range(row):
#     for j in range(col):
#         if img[i][j] == 255:
#             img[i][j] = 0
#         else:
#             img[i][j] = 255
# print(img)
# image = Image.fromarray(img)
# image.save('./test/test0.png')

# import torch
# # GPU
# gt_name = './test/test.png'
# img = cv2.imread(gt_name, -1)
# num_devices = cv2.cuda.getCudaEnabledDeviceCount()
# print(num_devices)

img = cv2.imread('/home/lance/Data/WhiteLine/1.png')
print(img[90][320])