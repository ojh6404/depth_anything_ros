#!/usr/bin/env python


import numpy as np
import cv2

INPUT_WIDTH, INPUT_HEIGHT = 518, 518
RGB_MEAN = [0.485, 0.456, 0.406]
RGB_STD = [0.229, 0.224, 0.225]


def preprocess(image: np.ndarray):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.0
    image = cv2.resize(image, (INPUT_WIDTH, INPUT_HEIGHT), interpolation=cv2.INTER_CUBIC)
    image = (image - RGB_MEAN) / RGB_STD
    image = image.transpose(2, 0, 1)[None].astype("float32")
    return image
