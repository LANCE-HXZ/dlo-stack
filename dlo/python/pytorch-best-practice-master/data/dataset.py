# coding:utf8
import os
from PIL import Image
from torch.utils import data
import numpy as np
from torchvision import transforms as T


class DogCat(data.Dataset):

    def __init__(self, root, transforms=None, train=True, test=False):
        '''
        主要目标： 获取所有图片的地址，并根据训练，验证，测试划分数据
        '''
        self.test = test
        self.train = train
        imgs = [os.path.join(root, img) for img in os.listdir(root)]  # 图片的文件名列表

        # test1: data/test1/8973.jpg
        # train: data/train/cat.10004.jpg 
        if self.test:
            # imgs = sorted(imgs, key=lambda x: int(x.split('.')[-2].split('/')[-1]))
            imgs = sorted(imgs, key=lambda x: int(x.split('.')[-2].split('/')[-1].split('_')[-2]+x.split('.')[-2].split('/')[-1].split('_')[-1]))  # 切分出测试集图片编号
        else:
            imgs = sorted(imgs, key=lambda x: int(x.split('.')[-2]))  # 切分出训练集图片编号

        imgs_num = len(imgs)  # 图片数量

        # shuffle imgs  打乱顺序
        np.random.seed(100)
        imgs = np.random.permutation(imgs)

        if self.test:
            self.imgs = imgs  # 测试集不划分
        elif train:
            self.imgs = imgs[:int(0.7 * imgs_num)]  # 划分 训练集 : 验证集 = 7 : 3 , 按顺序取
        else:
            self.imgs = imgs[int(0.7 * imgs_num):]
        # input[channel] = (input[channel] - mean[channel]) / std[channel]
        if transforms is None:  # 默认为 None
            normalize = T.Normalize(mean=[0.485, 0.456, 0.406],  # 数据转换 , 每个维度的值转换到规定区间内
                                    std=[0.229, 0.224, 0.225])
            # 测试集和验证集不用数据增强
            if self.test or not train:
                self.transforms = T.Compose([
                    T.Resize(224),
                    T.CenterCrop(224),
                    T.ToTensor(),
                    normalize
                ])
            # 训练集需要数据增强
            else:  # train
                self.transforms = T.Compose([
                    T.Resize(256),
                    T.RandomResizedCrop(224),
                    T.RandomHorizontalFlip(),
                    T.ToTensor(),
                    normalize
                ])

    def __getitem__(self, index):
        '''
        一次返回一张图片的数据
        '''
        img_path = self.imgs[index]
        # if self.test and not self.train:
        #     label = int(self.imgs[index].split('.')[-2].split('/')[-1])
        # else:  # 训练和验证集
        label = 1 if 'up' in img_path.split('/')[-1] else 0
        data = Image.open(img_path)
        data = self.transforms(data)
        return data, label

    def __len__(self):
        return len(self.imgs)
