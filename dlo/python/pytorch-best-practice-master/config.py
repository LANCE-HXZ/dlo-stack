# coding:utf8
import warnings


class DefaultConfig(object):
    env = 'default'  # visdom 环境
    model = 'ResNet34'  # 使用的模型，名字必须与models/__init__.py中的名字一致

    # train_data_root = '/home/hxz/data/crop/'  # 训练集存放路径
    train_data_root = '/home/lance/Data/7-CrossSquare/class/crop'
    test_data_root = '/home/lance/Data/7-CrossSquare/class/test1'   # 测试集存放路径
    # load_model_path = '/home/lance/Data/7-CrossSquare/class/model/0417_800.pth'
    load_model_path = '/home/lance/Workspaces/hxz_ws/src/dlo/python/pytorch-best-practice-master/checkpoints/1119_23:20:17_4.pth'
    # load_model_path = 'model_weights/crossnet_weights/0417_800.pth'
    # load_model_path = None   # 加载预训练的模型的路径，为None代表不加载

    # batch_size = 128  # batch size
    batch_size = 64  # batch size
    use_gpu = True  # user GPU or not
    num_workers = 32  # how many workers for loading data
    print_freq = 1  # print info every N batch

    debug_file = '/tmp/debug'  # if os.path.exists(debug_file): enter ipdb
    result_file = 'result.csv'
    traingData_file = 'trainingData.csv'#

    max_epoch = 500
    lr = 0.0001  # initial learning rate
    lr_decay = 0.95  # when val_loss increase, lr = lr*lr_decay
    weight_decay = 1e-4  # 损失函数


#     def parse(self, kwargs):
#         '''
#             根据字典kwargs 更新 config参数
#             '''
#         for k, v in kwargs.iteritems():
#             if not hasattr(self, k):
#                 warnings.warn("Warning: opt has not attribut %s" % k)
#             setattr(self, k, v)

#         print('user config:')
#         for k, v in self.__class__.__dict__.iteritems():
#             if not k.startswith('__'):
#                 print(k, getattr(self, k))


# DefaultConfig.parse = parse
opt = DefaultConfig()
# opt.parse = parse
