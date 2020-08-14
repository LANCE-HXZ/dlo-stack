from model import *
import torch
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import torch.backends.cudnn as cudnn
import cv2
from torchvision import transforms
from torch.autograd import Variable
import numpy as np
from PIL import Image
import os

# os.environ["CUDA_VISIBLE_DEVICES"] = "0"
modelname = 'bwnet600'
# modelname = 'dangenmynet600'
# modelname = 'mynet380'
# modelname = 'mynet600'
cudnn.benchmark = True
# model = torch.nn.DataParallel(UNet16(num_filters=32, pretrained='vgg')).cuda()
model = torch.nn.DataParallel(UNet16(num_filters=32, pretrained='vgg'))
# if flag == 't':
#     path = './data2m1100/train/Image/'
# else:
#     path = './data2m1100/vali/Image/'

path = '/home/lance/Workspaces/hxz_ws/pic_buffer/'
state_path = '/home/lance/Workspaces/hxz_ws/src/dlo/python/ros_mynet/model/' + modelname + '.ckpt'
# path = path + imgname + '.bmp'
# path = './checkpoints/'
state = torch.load(state_path, map_location=torch.device('cpu'))
# print(len(state))
model.load_state_dict(torch.load(state_path, map_location=torch.device('cpu')))
print('Model loaded')
# img_num = ['153', 'D057', 'T0348']
img_num = ['R']
for i in range(len(img_num)):
    img_name = path + img_num[i] + '.png'
    img = Image.open(img_name)
    img = transforms.ToTensor()(img)
    img = img.unsqueeze(0)
    # img_arr = Variable(img, requires_grad=True).cuda(async=True)
    # img_arr = img.cuda()
    img_arr = img
    out_overlap, out_gradient, Y = model(img_arr)

    imgo = transforms.ToPILImage()(out_overlap.data[0].cpu())
    imgg = transforms.ToPILImage()(out_gradient.data[0].cpu())
    imgY = transforms.ToPILImage()(Y.data[0].cpu())
    img_name = path + img_num[i] + 'O_' + modelname + '.png'
    imgo.save(img_name)
    print('out overlap save done')
    img_name = path + img_num[i] + 'G_' + modelname + '.png'
    imgg.save(img_name)
    print('out gradient save done')
    img_name = path + img_num[i] + 'Y_' + modelname + '.png'
    imgY.save(img_name)
    print('out Y save done')
