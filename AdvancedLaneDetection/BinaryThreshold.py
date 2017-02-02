#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 20:59:43 2017

@author: ankur
"""

#functions and structures for binary thresholding

from collections import namedtuple

def get_ROI(img, x1, y1, x2, y2):
    return img[y1:y2, x1:x2, :]




Threshold = namedtuple('Threshold', ['min', 'max', 'dTh'])



RThresh = Threshold(150, 250, 5)
VThresh = Threshold(150, 250, 5)
#RdxThresh = Threshold(150, 250, 5)

#Adaptive Binary Threshold.
#We use R channel from RGB and V from HSV.
#We iteratively find threshold parameters that give just enough pixels.