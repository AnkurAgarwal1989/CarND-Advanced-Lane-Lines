#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 21:06:10 2017

@author: ankur
"""

#Common utility functions for image processing
import cv2
import numpy as np

def get_ROI(img, x1, y1, x2, y2):
    return img[y1:y2, x1:x2, :]

def grayscale(img):
    '''Applies the Grayscale transform
    This will return an image with only one color channel'''
    ##for mpimg use
    #return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def colorspace(img, c_space):
    return cv2.cvtColor(img, c_space)

def get_channel(img, c):
    return img[:, :, c]

#points are tuples of (x,y) 
def drawROI(img, p1, p2):
    out = cv2.rectangle(img, p1, p2, (255, 0, 0))
    return out

#points are tuples of (x,y) 
def drawPolygon(img, p1, p2, p3, p4):
    pts = np.array([p1, p2, p3, p4], dtype=np.int32)
    pts = pts.reshape((-1,1,2))
    out = cv2.polylines(img, [pts], True, (0,255,255))
    return out

#drawPolygon(img, (378, 10), (508, 10), (875, 235), (45, 235) )
#drawPolygon(img, (350, 30), (540, 30), (875, 235), (45, 235) )