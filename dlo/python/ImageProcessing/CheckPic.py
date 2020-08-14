# 检查数据集中不完整的部分图

import os
from PIL import Image
import numpy as np


read_path = '/home/lance/Data/WhiteLine/T'
check_path = '/home/lance/Data/RGBimg/T'
save_path = '/home/lance/Data/RGBimg/T2'

image_list = os.listdir(read_path)
length = len(image_list)
for k in range(length):
    img = Image.open(os.path.join(check_path, image_list[k]))
    img.save(os.path.join(save_path, image_list[k]))
    print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, ', ', image_list[k])   