#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import torch
from torchvision import transforms
from PIL import Image
import os
import cv2
import matplotlib.pyplot as plt
# from MyData import test_dataloader, train_dataloader
from FCN import FCNs, VGGNet

# os.environ["CUDA_VISIBLE_DEVICES"] = "0"
img_num = '223'
model_num = '45'
model_prename = "fcn_model_"
model_name = './Model/' + model_prename + model_num + ".pt"
# img_name_list = os.listdir('./Checkpoint/')

# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')

model = torch.load(model_name, map_location=torch.device('cpu'))  # 加载模型
# model = torch.load(model_name, map_location='cuda:0')
model = model.to(device)
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
threshold = 128
table = []
for i in range(256):
    if i < threshold:
        table.append(0)
    else:
        table.append(1)

if __name__ == '__main__':
    # 单张
    # img_name = './Checkpoint/' + img_num + '.bmp'  # 预测的图片
    img_name = '/home/lance/Workspaces/hxz_ws/src/dlo/python/BW_NET/Checkpoint/7.jpg'  # 预测的图片
    imgA = cv2.imread(img_name)
 
    imgA = transform(imgA)
    imgA = imgA.to(device)
    imgA = imgA.unsqueeze(0)
    output = model(imgA)
    # print(output)
    output = torch.sigmoid(output)
    # print(output.shape)
    output_np = output.cpu().detach().numpy().copy()  # output_np.shape = (4, 2, 160, 160)
    # print(output_np.shape)  # (1, 2, 160, 160)
    output_np = np.argmin(output_np, axis=1)
    # print(output_np[0][101])  # (1,160, 160)
    img = Image.open(img_name)
 
    img_ = np.array(img)
    rows, cols, h = img_.shape
    img = transforms.ToTensor()(img)
    img[0] = img[1] = img[2] = torch.from_numpy(output_np[0])
    # for i in range(rows):
    #     for j in range(cols):
    #         img[0][i][j] = (0 if output_np[0][i][j] == 0 else 1)
    # img[1] = img[0]
    # img[2] = img[0]

    img = transforms.ToPILImage()(img)
    img = img.convert('L')
    img.save('./Checkpoint/%s_%s.png' %(img_num, model_num))
    photo = img.point(table, '1')
    photo.save('./Checkpoint/%s_%s.png' %(img_num, model_num))
    print('\n', '  ==== MISSION COMPLETE ====', '\n')


    # # 多张
    # for i in img_name_list:
    #     if os.path.splitext(i)[1] == ".bmp":
    #         img_name = '/home/lance/Workspaces/hxz_ws/src/dlo/python/BW_NET/Checkpoint/' + i
    #         print(img_name)
    #         imgA = cv2.imread(img_name)
    #         # imgA = cv2.resize(imgA, (160, 160))

    #         imgA = transform(imgA)
    #         imgA = imgA.to(device)
    #         imgA = imgA.unsqueeze(0)
    #         output = model(imgA)
    #         # print(output)
    #         output = torch.sigmoid(output)
    #         # print(output.shape)
    #         output_np = output.cpu().detach().numpy().copy()  # output_np.shape = (4, 2, 160, 160)
    #         # print(output_np.shape)  # (1, 2, 160, 160)
    #         output_np = np.argmin(output_np, axis=1)
    #         # print(output_np[0][101])  # (1,160, 160)
    #         img = Image.open(img_name)
    #         img_ = np.array(img)
    #         rows, cols, h = img_.shape
    #         img = transforms.ToTensor()(img)
    #         img[0] = img[1] = img[2] = torch.from_numpy(output_np[0])
    #         # for i in range(rows):
    #         #     for j in range(cols):
    #         #         img[0][i][j] = (0 if output_np[0][i][j] == 0 else 1)
    #         # img[1] = img[0]
    #         # img[2] = img[0]

    #         img = transforms.ToPILImage()(img)
    #         img = img.convert('L')
    #         # print(img)
    #         photo = img.point(table, '1')
    #         photo.save('/home/lance/Workspaces/hxz_ws/src/dlo/python/BW_NET/Checkpoint/%s_%s.png' %(i[:-4], model_num))
    # print('\n', '  ==== MISSION COMPLETE ====', '\n')

    # plt.subplot(1, 2, 1)
    # # plt.imshow(np.squeeze(bag_msk_np[0, ...]), 'gray')
    # # plt.subplot(1, 2, 2)
    # plt.imshow(np.squeeze(output_np[0, ...]), 'gray')
    # plt.pause(3000)
