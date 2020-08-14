from torch.utils.data import DataLoader
import torch
from dataset import *
from model import *
import time
from torch.autograd import Variable
import numpy as np

class Trainer(object):
    def __init__(self,model, train_loader,val_loader,criterion,epoch,path):#, lr):
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.criterion = criterion
        self.epoch = epoch
        self.savepath = path

    def train(self,optimizer,epoch,lr):
        start_time = time.time()
        loss_arr = [[],[],[],[]]
        for i, (img, overlap,gradient) in enumerate(self.train_loader): 
            
            # torch.nn.Dropout(0.5)## 
            # print("train pic %d"%i) 
            # dropout= nn.Dropout(p=0.5)
            # output = dropout(i)
            img_arr = Variable(img, requires_grad=True)
            overlap_arr = Variable(overlap).float()
            gradient_arr =Variable(gradient).float()

            img_arr = img_arr.cuda()
            overlap_arr = overlap_arr.cuda()
            gradient_arr = gradient_arr.cuda()
            out_overlap, out_garadient,Y = self.model(img_arr)
            #out_overlap = out_overlap.float()

            loss1,loss2,loss3 = self.criterion(gradient_arr,overlap_arr,out_garadient,out_overlap,Y)

            loss = loss1 + loss2 + loss3
            loss_arr[0].append(loss.data.cpu().numpy())
            loss_arr[1].append(loss1.data.cpu().numpy())
            loss_arr[2].append(loss2.data.cpu().numpy())
            loss_arr[3].append(loss3.data.cpu().numpy())
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        end_time = time.time()
        loss_arr = np.array(loss_arr)
        print('Epoch %d (lr %.4f)' % (epoch, lr))
        print('train ave: loss %.5f loss1:%.5f  loss2:%.5f loss3:%.5f ,time %3.2f' \
               % (np.mean(loss_arr[0]), np.mean(loss_arr[1]),np.mean(loss_arr[2]),np.mean(loss_arr[3]),end_time-start_time))

    def val(self):
        start_time = time.time()
        loss_arr = [[],[],[],[]]
        for i, (img,overlap,gradient) in enumerate(self.val_loader):
            # print("val pic %d"%i)
            img_arr = Variable(img, requires_grad=True)
            overlap_arr = Variable(overlap).float()
            gradient_arr =Variable(gradient).float()
            img_arr = img_arr.cuda()
            overlap_arr = overlap_arr.cuda()
            gradient_arr = gradient_arr.cuda()
            out_overlap, out_garadient,Y = self.model(img_arr)
            #out_overlap = out_overlap.float()
            loss1,loss2,loss3 = self.criterion(gradient_arr,overlap_arr,out_garadient,out_overlap,Y)
            loss = loss1 + loss2 + loss3
            loss_arr[0].append(loss.data.cpu().numpy())
            loss_arr[1].append(loss1.data.cpu().numpy())
            loss_arr[2].append(loss2.data.cpu().numpy())
            loss_arr[3].append(loss3.data.cpu().numpy())
        end_time = time.time()
        loss_arr = np.array(loss_arr)
        print('val ave: loss %.5f loss1:%.5f  loss2:%.5f loss3:%.5f ,time %3.2f' \
           % (np.mean(loss_arr[0]), np.mean(loss_arr[1]),np.mean(loss_arr[2]),np.mean(loss_arr[3]),end_time - start_time))

    def save_model(self,savename):
        # torch.save(self.model,'mynet.pkl',self.savepath)
        state_dict = self.model.state_dict()
        torch.save(state_dict,savename)




