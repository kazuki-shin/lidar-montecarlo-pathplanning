from __future__ import division

from models import *
from utils.utils import *
from utils.datasets import *

import os
import sys
import time
import datetime
import argparse

from PIL import Image

import torch
from torch.utils.data import DataLoader
from torchvision import datasets
from torch.autograd import Variable

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator
from skimage import io
from torchvision import transforms

if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    os.makedirs("output", exist_ok=True)

    # Set up model
    model = Darknet('config/yolov3.cfg', img_size=416).to(device)

    model.load_darknet_weights('./weights/yolov3.weights')

    model.eval()  # Set in evaluation mode


    classes = load_classes('data/coco.names')  # Extracts class labels from file

    Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor

    imgs = []  # Stores image paths
    img_detections = []  # Stores detections for each image index

    image = Image.open('./test_images/photo-1531850629230-0496c86d6d23.jpeg')
    resized = transforms.Resize((416,416))(image)
    image_tensor = torch.Tensor(transforms.ToTensor()(resized)).unsqueeze(0)
    detections = model(image_tensor)

    #object confidence threshold
    conf_thres=0.8
    #iou thresshold for non-maximum suppression
    nms_thres=0.4
    detections = non_max_suppression(detections, conf_thres, nms_thres)[0]
    detected_classes = [classes[int(x)] for x in detections[:,-1]]

    if 'person' in detected_classes:
        print("Person detected")
    else:
        ("nobody there")
