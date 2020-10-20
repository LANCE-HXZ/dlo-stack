from model import *
import  dataset as data
from torch.utils.data import DataLoader
from train import Trainer
import torch
import torch.backends.cudnn as cudnn
from torch import nn
from torch.autograd import Variable
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

EPOCH  = 601
LR = 0.0001
#class_weight  = torch.Tensor([0.0001,1,100]).cuda(async = True)
model_save_path = '/home/hxz/bw_net/model'
save_frequence = 20

def get_lr(epoch,num):
    lr = LR
    if num*0.5 <= epoch <num*0.8:
        lr  = 0.5*lr
    elif epoch >= num*0.8:
        lr = 0.1*lr
    return lr


def loss(tar_G,tar_O,out_G,out_O,Y):
    #weight = class_weight
    # loss_O = nn.CrossEntropyLoss()
    loss_O = nn.MSELoss()#, size_average=True)#, reduce=True)#,overlap)BCEWithLogitsLoss
    #loss_O = nn.L1Loss()
    # loss_O = nn.MultiLabelSoftMarginLoss()
    # loss_O = nn.PoissonNLLLoss()
    # loss_O = nn.SoftMarginLoss()#, size_average=True)#, reduce=True)#,overlap)
    # loss_O = nn.SoftMarginLoss()#, size_average=True)#, reduce=True)#,overlap)BCEWithLogitsLoss
    loss_G = nn.MSELoss()#size_average=True, reduce=True)#,gradient)
    loss_G_ = nn.MSELoss()#size_average=True, reduce=True)#, Y)
    # loss_G = nn.L1Loss()#size_average=True, reduce=True)#,gradient)
    # loss_G_ = nn.L1Loss()#size_average=True, reduce=True)#, Y)
    # tar_O = tar_O.type(torch.cuda.LongTensor)


    return loss_O(out_O,tar_O),loss_G(out_G,tar_G),loss_G_(Y,tar_G)
    # return loss_G(out_G,tar_G),loss_G_(Y,tar_G)

def main():
    model = torch.nn.DataParallel(UNet16(num_filters = 32, pretrained = 'vgg')).cuda()
    print('model get')
    cudnn.benchmark = True    
    criterion = loss    

    dataset_t = data.MyData_tr('/home/hxz/data/train')
    dataset_v = data.MyData_vl('/home/hxz/data/vali')
    train_loader = DataLoader(
        dataset_t,
        batch_size = 2,
        shuffle = True,
        num_workers = 1,
        pin_memory=True)

    val_loader = DataLoader(
        dataset_v,
        batch_size = 2,
        shuffle = True,
        num_workers = 1,
        pin_memory=True)

    trainer = Trainer(    #self,model, train_loader,criterion,optimizer, epoch, lr
    model = model,
    train_loader = train_loader,
    val_loader = val_loader,
    criterion  = criterion,
    # optimizer = optimizer,
    epoch = EPOCH,
    path = model_save_path)#, lr = )

    for epoch in range(EPOCH):
        lr = get_lr(epoch,EPOCH)
        optimizer = torch.optim.Adam(model.parameters(),lr)
        trainer.train(optimizer,epoch,lr)         
        trainer.val()  
        if epoch == 0:
            trainer.save_model(os.path.join(model_save_path,'mynet%d.ckpt'%epoch))
            print('mynet%d.ckpt saved'%epoch)
        if epoch%save_frequence == 0:        
            trainer.save_model(os.path.join(model_save_path,'mynet%d.ckpt'%epoch))
            print('mynet%d.ckpt saved'%epoch)


if __name__ == '__main__':
    main()

