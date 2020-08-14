from PIL import Image
import numpy as np
import os
import cv2

# 分出一百个测试集
# read_path = '/home/lance/Data/7-CrossSquare/class/4-train'
# read_path = '/home/lance/Data/7-CrossSquare/class/t'
read_path = '/home/lance/Data/7-CrossSquare/class/4-test/0'
image_list = os.listdir(read_path)
length = len(image_list)

# for k in range(100):  # train
#     os.remove(os.path.join(read_path, image_list[k]))
for k in range(400):  # test
    os.remove(os.path.join(read_path, image_list[k+100]))
