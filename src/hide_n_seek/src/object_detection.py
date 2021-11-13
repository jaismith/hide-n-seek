#!/usr/bin/python3

import os
import sys
import time
import torch
import rospy
import torch.optim as optim
import torch.nn as nn
from PIL import Image
from torchvision import datasets, transforms
from matplotlib import patches, patheffects
import cv2
from cv_bridge import CvBridge, CvBridgeError

sys_paths = ['../']
for p in sys_paths:
    p = os.path.abspath(p)
    if p not in sys.path:
        sys.path.append(p)

from yolov3_pytorch.utils import *
from yolov3_pytorch.yolov3 import *
from yolov3_pytorch.yolov3_tiny import *

class_names = ['person', 'bicycle', 'car', 'motorbike', 'aeroplane', 'bus', \
'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', \
'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', \
'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', \
'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', \
'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', \
'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', \
'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', \
'donut', 'cake', 'chair', 'sofa', 'pottedplant', 'bed', 'diningtable', \
'toilet', 'tvmonitor', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', \
'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', \
'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

def image_callback(msg):
    sz = 416
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        raise e
    img_org = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    # img_org = Image.open(msg).convert('RGB')
    img_resized = img_org.resize((sz, sz))
    img_torch = image2torch(img_resized)

    model = Yolov3Tiny(num_classes=len(class_names))
    model.load_state_dict(torch.load('../yolov3_pytorch/yolov3_tiny_coco_01.h5'))

    all_boxes = model.predict_img(img_torch, conf_thresh=0.3)[0]
    boxes = nms(all_boxes, 0.3)
    plot_img_detections(img_resized, boxes, figsize=(8,8), class_names=class_names)

rospy.Subscriber('/camera_node/rgb/image_raw', Image, image_callback, queue_size=1)
