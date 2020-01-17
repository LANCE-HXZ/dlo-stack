# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import torch
from torchvision import transforms
from PIL import Image
import os
import cv2
import matplotlib.pyplot as plt
from MyData import test_dataloader, train_dataloader
from FCN import FCNs, VGGNet
img_num = 'test_001'
model_num = '0'
img_name = 'checkpoints/predict/' + img_num + '.bmp'  # 预测的图片
model_name = 'checkpoints/fcn_model_' + model_num + '.pt'

# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')

model = torch.load(model_name)  # 加载模型
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
    imgA = cv2.imread(img_name)
    # imgA = cv2.resize(imgA, (160, 160))

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
    # print(img)
    photo = img.point(table, '1')
    photo.save('checkpoints/predict/%s_%s.bmp' %(img_num, model_num))
    print('\n', '  ==== MISSION COMPLETE ====')

    # plt.subplot(1, 2, 1)
    # # plt.imshow(np.squeeze(bag_msk_np[0, ...]), 'gray')
    # # plt.subplot(1, 2, 2)
    # plt.imshow(np.squeeze(output_np[0, ...]), 'gray')
    # plt.pause(3000)