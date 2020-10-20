# from ClassList.msg import classlist
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
from config import opt
import os
import torch as t
import models
from data.dataset import DogCat
from torch.utils.data import DataLoader
from torchvision import transforms as T
from torchnet import meter
from utils.visualize import Visualizer
from tqdm import tqdm
import numpy as np
import PIL

model = getattr(models, opt.model)().eval()
if opt.load_model_path:
    model.load(opt.load_model_path)
if opt.use_gpu: model.cuda()
print('\n', 'Crop_predict_net model loaded')

def predict(data, results):
    # opt.parse(kwargs)
    # import ipdb
    # ipdb.set_trace()
    # configure model
    # data = transforms.ToTensor()(data)
    data = data.unsqueeze(0)
    # data
    # train_data = DogCat(opt.test_data_root, test=True)
    # test_dataloader = DataLoader(train_data, batch_size=opt.batch_size, shuffle=False, num_workers=opt.num_workers)
    
    # for ii, (data, path) in enumerate(test_dataloader):
    input = data
    if opt.use_gpu: input = input.cuda()
    score = model(input)
    probability = t.nn.functional.softmax(score)[:, 0].data.tolist()
    label = score.max(dim = 1)[1].data.tolist()

    # batch_results = [(label_, path_, probability_) for label_, path_, probability_ in zip(label, path, probability)]
    batch_results = [(label_, probability_) for label_, probability_ in zip(label, probability)]

    results += label
    # write_csv(results, opt.result_file)

    return results





class image_converter:
 
  def __init__(self):
    
    # normalize = T.Normalize(mean=[0.485, 0.456, 0.406],  # 数据转换 , 每个维度的值转换到规定区间内
    #                                 std=[0.229, 0.224, 0.225])
    # self.transforms = T.Compose([
    #                 T.Resize(224),
    #                 T.CenterCrop(224),
    #                 T.ToTensor(),
    #                 normalize
    #             ])

 
    # (rows,cols,channels) = cv_image.shape
    pil_img = PIL.Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    data = self.transforms(pil_img)
    # print(pil_img)
    class_result = predict(data, self.results)
    


def main(args):
  ic = image_converter()
 
if __name__ == '__main__':
    main(sys.argv)