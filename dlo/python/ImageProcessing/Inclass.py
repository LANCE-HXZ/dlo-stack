from PIL import Image
import numpy as np
import os
import cv2

# 用键盘分类放置图片
read_path = '/home/lance/Data/test/5-图片分类/left'
up_path = '/home/lance/Data/test/5-图片分类/up'
down_path = '/home/lance/Data/test/5-图片分类/down'

image_list = os.listdir(read_path)
length = len(image_list)
for k in range(length):
    img = cv2.imread(os.path.join(read_path, image_list[k]), cv2.IMREAD_COLOR)
    cv2.namedWindow("crop",0)
    cv2.imshow("crop", img)
    key = cv2.waitKey(0)
    print(key)
    if key == 1048695: # w键 up类
        cv2.imwrite(os.path.join(up_path, image_list[k]),img)
    elif key == 1048691: # s键 down类
        cv2.imwrite(os.path.join(down_path, image_list[k]),img)
    elif key == 1048689:
        break
    else:
        pass
    up_list = os.listdir(up_path)
    down_list = os.listdir(down_path)
    print('\t', '\t', "up: ", len(up_list), "    down: ", len(down_list))
    os.remove(os.path.join(read_path, image_list[k]))
cv2.destroyAllWindows()
