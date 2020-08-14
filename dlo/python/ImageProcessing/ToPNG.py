#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 图片格式同一为(640, 360)的png
from PIL import Image
import numpy as np
import os

path = './'
image_list = os.listdir(path)
length = len(image_list)
for k in range(length):
    img = Image.open(os.path.join(path, image_list[k]))
    img = img.resize((640, 320))
    img.save(os.path.join(path, image_list[k][:-4] + '.png'))
    print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )   

# path = '/home/lance/Data/5-Background/white'
# img = Image.open(path + '.png')
# # img = img.resize((640, 320))
# img = img.resize((640+160, 320+160))
# img.save(os.path.join(path + '.png'))
   

