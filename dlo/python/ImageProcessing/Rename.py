# 图片格式同一为(640, 360)的png
from PIL import Image
import numpy as np
import os

# # D0
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'D0' + image_list[k][1:2]+image_list[k][4:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )   

# # D1
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'D' + image_list[k][1:3]+image_list[k][5:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )  

# # T0
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T0' + image_list[k][1:2]+image_list[k][4:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )   

# # T1
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T' + image_list[k][1:3]+image_list[k][5:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )  

# # 1-100
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T0' + image_list[k][1:2]+image_list[k][9:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )   

# # x01-x09
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T0' + image_list[k][1:2]+image_list[k][-5:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )

# # x00
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T0' + image_list[k][1:2] + '1' + image_list[k][-6:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )

# # x10-x99
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T0' + image_list[k][1:2] + image_list[k][-6:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )

# # xx01-xx09
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T' + image_list[k][1:3]+image_list[k][-5:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )

# # xx00
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T' + image_list[k][1:3]+'1'+image_list[k][-6:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )

# # xx10-xx99
# read_path = '/home/lance/Data/test/重命名/read'
# save_path = '/home/lance/Data/test/重命名/save'
# image_list = os.listdir(read_path)
# length = len(image_list)
# for k in range(length):
#     img = Image.open(os.path.join(read_path, image_list[k]))
#     img = img.resize((640, 360))
#     img.save(os.path.join(save_path,'T' + image_list[k][1:3]+image_list[k][-6:]))
#     print('Transforming: ', (k+1)/length, ', ', k+1, ' / ', length, )