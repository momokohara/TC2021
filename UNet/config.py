import torch
import os
from torch import cuda
from src.utils import Color_map
os.environ['CUDA_LAUNCH_BLOCKING'] = "1"

class config(object):
  model_path = "/content/drive/MyDrive/google colab/CamVid-Segmentation-Pytorch-master/UNet/model/"
  path = "/content/drive/MyDrive/google colab/CamVid/"
  load_model = "/content/drive/MyDrive/google colab/CamVid-Segmentation-Pytorch-master/UNet/model/state_dict.pt"
  batch = 4
  lr = 0.0001
  epochs = 40
  input_size = (128,128)
  if cuda.is_available(): device = torch.device("cuda")
  else: device = torch.device('cpu')
  code2id, id2code, name2id, id2name = Color_map(path+'class_dict.csv')