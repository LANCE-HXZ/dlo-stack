# 图片格式同一为(640, 360)的png
from PIL import Image
import numpy as np
from torchvision import transforms
import os

rgb_path = '/home/lance/Data/RGBimg/'
depth_path = '/home/lance/Data/DepthMap/'
save_path = '/home/lance/Data/'

image_list = os.listdir(rgb_path)
length = len(image_list)
# for k in range(length):
k = 1
if k:
    rgb_img = Image.open(os.path.join(rgb_path, image_list[k]))
    rgb_img = transforms.ToTensor()(rgb_img)
    depth_img = Image.open(os.path.join(depth_path, image_list[k]))
    depth_img = transforms.ToTensor()(depth_img)
    print(depth_img[0][0])
    rgb_img[2] = 120
    rgb_img = transforms.ToPILImage()(rgb_img)
    rgb_img.save(os.path.join(save_path, image_list[k]))
    print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, ', ', image_list[k])   