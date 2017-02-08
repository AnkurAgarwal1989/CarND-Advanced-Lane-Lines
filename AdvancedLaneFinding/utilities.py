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
def draw_ROI(img, p1, p2):
    out = cv2.rectangle(img, p1, p2, (255, 0, 0))
    return out

#points are tuples of (x,y) 
def draw_polygon(img, p1, p2, p3, p4):
    pts = np.array([p1, p2, p3, p4], dtype=np.int32)
    pts = pts.reshape((-1,1,2))
    out = cv2.polylines(img, [pts], True, (0,255,255))
    return out

#drawPolygon(img, (380, 10), (505, 10), (875, 235), (45, 235) )
#drawPolygon(img, (350, 30), (540, 30), (875, 235), (45, 235) )

#Warp an image to an output size by applying M
#out_size is a tuple (640, 240)
def warp_image(img, M, out_size):
    return (cv2.warpPerspective(img, M, out_size, flags=cv2.INTER_LINEAR))

def get_warp_unwarp_matrices(w_cfg):
    src_pts = np.array([w_cfg['p1'], w_cfg['p2'], w_cfg['p3'], w_cfg['p4']], dtype=np.float32)
    
    offset = w_cfg['offset']
    img_x = w_cfg['image_shape'][0]
    img_y = w_cfg['image_shape'][1]
    dst_pts = np.array([[offset, 0], [img_x-offset, 0], [img_x-offset, img_y], [offset, img_y]], dtype=np.float32)
  
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    Minv = np.linalg.inv(M)
    return M, Minv

def undistort(image, calibration):
    mtx = calibration['mtx']
    dist = calibration['dist']
    return cv2.undistort(image, mtx, dist, None, mtx)

scale_X = 3.7/430 #(meters/pixels)
scale_Y = 3.048/34 #(meters/pixels)