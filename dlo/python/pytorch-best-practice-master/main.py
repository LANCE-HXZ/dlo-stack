# coding:utf8
from config import opt
import os
import torch as t
import models
from data.dataset import DogCat
from torch.utils.data import DataLoader
from torchnet import meter
from utils.visualize import Visualizer
from tqdm import tqdm
import numpy as np
import time

# cpu运行修改了此处
# pytorch-best-practice-master/models/BasicModule.py", line 19, in load
#     self.load_state_dict(t.load(path))
def test(**kwargs):
    # opt.parse(kwargs)
    # import ipdb
    # ipdb.set_trace()
    # configure model
    model = getattr(models, opt.model)().eval()
    if opt.load_model_path:
        model.load(opt.load_model_path)
    if opt.use_gpu: model.cuda()

    # data
    train_data = DogCat(opt.test_data_root, train = False, test=True)
    test_dataloader = DataLoader(train_data, batch_size=opt.batch_size, shuffle=False, num_workers=opt.num_workers)
    results = []
    cnt = 0 # 用于统计测试结果
    for ii, (data, path) in enumerate(test_dataloader):
        input = data
        if opt.use_gpu: input = input.cuda()
        score = model(input)
        probability = t.nn.functional.softmax(score)[:, 0].data.tolist()
        label = score.max(dim = 1)[1].data.tolist()
        for l in label:
            cnt+=l
        batch_results = [(label_, path_, probability_, cnt) for label_, path_, probability_ in zip(label, path, probability)]

        results += batch_results
    write_csv(results, opt.result_file)
    print(cnt)
    return results


def write_csv(results, file_name):
    import csv
    with open(file_name, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['id', 'label'])
        writer.writerows(results)


def train(**kwargs):
    # opt.parse(kwargs)
    vis = Visualizer(opt.env)

    savingData = []#
    # step1: configure model
    model = getattr(models, opt.model)()
    if opt.load_model_path:
        model.load(opt.load_model_path)
    if opt.use_gpu: model.cuda()

    # step2: data
    train_data = DogCat(opt.train_data_root, train=True)
    val_data = DogCat(opt.train_data_root, train=False)
    test_data = DogCat(opt.test_data_root, test=True)
    train_dataloader = DataLoader(train_data, opt.batch_size,
                                  shuffle=True, num_workers=opt.num_workers)
    val_dataloader = DataLoader(val_data, opt.batch_size,
                                shuffle=False, num_workers=opt.num_workers)
    test_dataloader = DataLoader(test_data, opt.batch_size, 
                                shuffle=False, num_workers=opt.num_workers)

    # step3: criterion and optimizer
    criterion = t.nn.CrossEntropyLoss()
    lr = opt.lr
    optimizer = t.optim.Adam(model.parameters(), lr=lr, weight_decay=opt.weight_decay)

    # step4: meters
    loss_meter = meter.AverageValueMeter()
    confusion_matrix = meter.ConfusionMeter(2)
    previous_loss = 1e100

    # train
    for epoch in range(opt.max_epoch+1):

        # validate and visualize
        val_cm, val_accuracy = val(model, val_dataloader)
        test_cm, test_accuracy = val(model, test_dataloader)
        vis.plot('test_accuracy', test_accuracy)
        vis.plot('lr', lr)
        vis.plot('val_accuracy', val_accuracy)
        vis.log("epoch:{epoch},lr:{lr},loss:{loss},train_cm:{train_cm},val_cm:{val_cm},test_cm:{test_cm}".format(
            epoch=epoch, loss=loss_meter.value()[0], val_cm=str(val_cm.value()), train_cm=str(confusion_matrix.value()), test_cm=str(test_cm.value()),
            lr=lr))
        print("epoch = ", epoch, "   loss = ", loss_meter.value()[0], "   lr = ", lr)
        batch_results = [(epoch, loss_meter.value()[0], lr, str(val_cm.value()), str(confusion_matrix.value()), str(test_cm.value()), val_accuracy, test_accuracy)]#
        savingData += batch_results#
        save_training_data(savingData, opt.traingData_file)#
        # update learning rate
        # if loss_meter.value()[0] > previous_loss:
        lr = lr * opt.lr_decay
            # # 第二种降低学习率的方法:不会有moment等信息的丢失
            # for param_group in optimizer.param_groups:
            #     param_group['lr'] = lr

        if epoch == opt.max_epoch:
            return

        previous_loss = loss_meter.value()[0]
        loss_meter.reset()
        confusion_matrix.reset()
        for ii, (data, label) in tqdm(enumerate(train_dataloader), total=len(train_data)/opt.batch_size):

            # train model 
            input = data
            target = label
            if opt.use_gpu:
                input = input.cuda()
                target = target.cuda()

            optimizer.zero_grad()
            score = model(input)
            loss = criterion(score, target)
            loss.backward()
            optimizer.step()

            # meters update and visualize
            loss_meter.add(loss.item())
            confusion_matrix.add(score.data, target.data)

            if ii % opt.print_freq == opt.print_freq - 1:
                vis.plot('loss', loss_meter.value()[0])

                # 进入debug模式
                if os.path.exists(opt.debug_file):
                    import ipdb;
                    ipdb.set_trace()
                    
        prefix = 'checkpoints/'
        name = time.strftime(prefix + '%m%d_%H:%M:%S_'+str(epoch+1)+'.pth')
        if epoch == 0:
            model.save(name)
        if np.mod(epoch+1, 10) == 0:
            model.save(name)


def save_training_data(results, file_name):
    import csv
    with open(file_name, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['epoch', 'loss', 'lr', 'val_cm', 'train_cm', 'test_cm', 'val_accuracy', 'test_accuracy'])
        writer.writerows(results)


def val(model, dataloader):
    '''
    计算模型在验证集上的准确率等信息
    '''
    model.eval()
    confusion_matrix = meter.ConfusionMeter(2)
    for ii, data in enumerate(dataloader):
        input, label = data
        val_input = input  #, volatile=True
        val_label = label.type(t.LongTensor)
        if opt.use_gpu:
            val_input = val_input.cuda()
            val_label = val_label.cuda()
        score = model(val_input)
        confusion_matrix.add(score.data.squeeze(), label.type(t.LongTensor))

    model.train()
    cm_value = confusion_matrix.value()
    accuracy = 100. * (cm_value[0][0] + cm_value[1][1]) / (cm_value.sum())
    return confusion_matrix, accuracy


def help():
    '''
    打印帮助的信息： python file.py help
    '''

    print('''
    usage : python file.py <function> [--args=value]
    <function> := train | test | help
    example: 
            python {0} train --env='env0701' --lr=0.01
            python {0} test --dataset='path/to/dataset/root/'
            python {0} help
    avaiable args:'''.format(__file__))

    from inspect import getsource
    source = (getsource(opt.__class__))
    print(source)


if __name__ == '__main__':
    import fire
    fire.Fire()
