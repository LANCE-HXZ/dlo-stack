import os
import numpy as np
from torch.utils.data.dataset import Dataset
import torch
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
#import cv2
from torchvision import transforms
from PIL import Image

class MyData_tr(Dataset):
	def __init__(self,root):

		self.root =root
		self.image_path = self.root + '/RGB'
		self.overlap_path = self.root + '/BW'
		self.gradient_path = self.root + '/BW'
		self.image_list = os.listdir(self.image_path)
	
		self.len = len(self.image_list)

	def __getitem__(self,index):
		img = transforms.ToTensor()(Image.open(os.path.join(self.image_path,self.image_list[index])))
		overlap = transforms.ToTensor()(Image.open(os.path.join(self.overlap_path,self.image_list[index])))
		gradient  = transforms.ToTensor()(Image.open(os.path.join(self.gradient_path,self.image_list[index])))
		return img, overlap, gradient

	def __len__(self):
		return self.len

class MyData_vl(Dataset):
	def __init__(self,root):
		self.root = root
		self.image_path = self.root + '/RGB'
		self.overlap_path = self.root + '/WB'
		self.gradient_path = self.root + '/WB'
		self.image_list = os.listdir(self.image_path)
	
		self.len = len(self.image_list)

	def __getitem__(self,index):
		img = transforms.ToTensor()(Image.open(os.path.join(self.image_path,self.image_list[index])))
		overlap = transforms.ToTensor()(Image.open(os.path.join(self.overlap_path,self.image_list[index])))
		gradient  = transforms.ToTensor()(Image.open(os.path.join(self.gradient_path,self.image_list[index])))
		return img, overlap, gradient




	def __len__(self):
		return self.len

# dataset_t = MyData_vl('data_/vali')
# print(dataset_t.__len__())

