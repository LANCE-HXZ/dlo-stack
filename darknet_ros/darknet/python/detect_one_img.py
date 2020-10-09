#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ctypes import *
import darknet as dn
import time
import colorsys
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from PIL import Image, ImageDraw, ImageFont
import cv2
# import pyrealsense2 as rs
import numpy as np
import math
import random
import os


net = dn.load_net(b"darknet_ros/darknet_ros/yolo_network_config/cfg/yolov3-voc.cfg", b"model_weights/darknet_weights/yolov3-voc_final.weights", 0)
meta = dn.load_meta(b"darknet_ros/darknet_ros/yolo_network_config/cfg/voc.data")

font = cv2.FONT_HERSHEY_SIMPLEX
fps = 0

f = open('detect_result.txt','w')

while True:

         fps = fps+1

         f.write(str(fps)+"\n")
         
         img = cv2.imread('pic_buffer/1_R.png')
         
         r = dn.detect(net, meta, "pic_buffer/1_R.png".encode('utf-8'))
         for item in r:   #画方框和文字
            ileft = int(item[2][0]-item[2][2]*0.5)##矩形点坐标##item[2][0]交点x轴坐标
            iup = int(item[2][1]-item[2][3]*0.5)
            iright = int(item[2][0]+item[2][2]*0.5)
            ibtm = int(item[2][1]+ item[2][3]*0.5)
            persent = str(round(item[1],2))
            name = item[0].decode('utf-8')+"\n"
            f.write(name)
            text = item[0].decode('utf-8')+persent
            loca = str(int(item[2][0]))+" "+str(int(item[2][1]))+"\n"+str(int(item[2][2]))+" "+str(int(item[2][3]))+"\n"
            f.write(loca)
            cv2.rectangle (img, (ileft,iup), (iright,ibtm), (0,255,0), 2)
            cv2.putText(img, text, (ileft, iup-8), font, 0.5, (0,0,255), 1)
                # print (item)
                 
         cv2.imshow('result', img)
         key = cv2.waitKey(1)
	 
         if key & 0xFF == ord('q') or key == 27:
             f.close() 
             cv2.destoryAllWindows()
             break


  





