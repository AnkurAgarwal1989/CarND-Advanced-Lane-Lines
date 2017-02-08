#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 20:59:43 2017

@author: ankur
"""

#functions and structures for binary thresholding

from collections import namedtuple
import numpy as np
import cv2
from . import utilities as cvUtils

ThresholdRange = namedtuple('ThresholdRange', ['min', 'max', 'dTh'])

#RdxThresh = Threshold(150, 250, 5)

#Adaptive Binary Threshold.
#We use R channel from RGB and V from HSV.
#We iteratively find threshold parameters that give just enough pixels.
## Input:
    #ROI image:3 channel
    #Initial values for R and V
    #Bailout: max iterations
## Output:
    #Binary image: 3 channel
    #RthreshValue
    #VthreshValue
# input ROI image is 3 channel
# returns a 3channel Binary image (0 and 255)
def binary_threshold(roi, config):
    
    R_Range = config['R_Range']
    V_Range = config['V_Range']
    R_Thresh = config['R_init']
    V_Thresh = config['V_init']
    bailout = config['bailout']
        
    thresh_img = np.zeros_like(roi[:, :, 0])
    
    total_pixels = thresh_img.shape[0]*thresh_img.shape[1]
    
    #Using R and G channels to mask out dark or grey road/ shadow values
    (B,G,R) = cv2.split(roi)
    
    R_dx = np.absolute(cv2.Sobel(R, cv2.CV_64F, 1, 0))
    R_dx *= 255/np.max(R_dx)
    
    hls = cvUtils.colorspace(roi, cv2.COLOR_BGR2HLS)
    S = cvUtils.get_channel(hls, 2)
    
    hsv = cvUtils.colorspace(roi, cv2.COLOR_BGR2HSV)
    V = cvUtils.get_channel(hsv, 2)
    
    #Create an initial binary image. We will refine this if we don't have enough pixels or too many pixels
    thresh_img[(V >= V_Thresh) | (R >= R_Thresh)] = 255
    
    #Count of Non-Zero mask pixels
    nzcount = np.count_nonzero(thresh_img)
    
    #Bailout Counter. If we can not reach a good value in n steps, stop wasting time and move on.
    counter = 0
    
    wiggleScope = True
    
    #We want the number of lane pixels to be within 1% to 3% of the total pixels in image
    minarea = 0.015*total_pixels
    maxarea = 0.025*total_pixels
    
    success = True
    
    while ((nzcount < minarea) | (nzcount >= maxarea)) & (wiggleScope):
        counter += 1
        if (counter == bailout):
            print("Unable to find a good value in {} steps. Bailing out!".format(bailout))
            success = False
            break
            
        #If there is still scope in moving ranges
        VwiggleScope = (V_Thresh >= V_Range.min) & (V_Thresh <= V_Range.max)
        RwiggleScope = (R_Thresh >= R_Range.min) & (R_Thresh <= R_Range.max)
        wiggleScope = VwiggleScope | RwiggleScope
        
        if wiggleScope:
            if (nzcount < minarea):
                if VwiggleScope:
                    V_Thresh -= V_Range.dTh
                if RwiggleScope:
                    R_Thresh -= R_Range.dTh
            else:
                if VwiggleScope:
                    V_Thresh += V_Range.dTh
                if RwiggleScope:
                    R_Thresh += R_Range.dTh
                    
            thresh_img = np.zeros_like(thresh_img)
            thresh_img[(V >= V_Thresh) | (R >= R_Thresh)] = 255
            nzcount = np.count_nonzero(thresh_img)
            
        else:
            print("Unable to find a good value in range. Bailing out!")
            success = False
            break
    
    if not success:
        nzcount = np.count_nonzero(thresh_img)
        thresh_img = np.zeros_like(thresh_img)
    
    
    print("%cnt", nzcount/total_pixels)
    print("steps", counter)    
    #print(nzcount, total_pixels)
        
    bin_img = np.dstack((thresh_img, thresh_img, thresh_img))
    config['R_init'] = R_Thresh
    config['V_init'] = V_Thresh
    return success, bin_img, config
