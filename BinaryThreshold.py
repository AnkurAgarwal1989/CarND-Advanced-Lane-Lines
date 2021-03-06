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
import utilities as cvUtils

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
    R_Thresh = config['R_best']
    V_Thresh = config['V_best']
    bailout = config['bailout']
    minLane = config['minLane']
    maxLane = config['maxLane']
        
    thresh_img = np.zeros_like(roi[:, :, 0])
    
    total_pixels = thresh_img.shape[0]*thresh_img.shape[1]
    
    #Using R and G channels to mask out dark or grey road/ shadow values
    (B,G,R) = cv2.split(roi)
    
    R_dx = np.absolute(cv2.Sobel(R, cv2.CV_64F, 1, 0))
    R_dx *= 255/np.max(R_dx)
    
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
    minarea = minLane * total_pixels
    maxarea = maxLane * total_pixels
    
    success = True
    ddth = 0
    while ((nzcount < minarea) | (nzcount >= maxarea)) & (wiggleScope):
        
        print(nzcount/total_pixels, counter, R_Thresh, V_Thresh)
        counter += 1
        if (counter == int(bailout/2)):
            ddth = 3
        if (counter == bailout):
            print("Unable to find a good value in {} steps. Bailing out!".format(bailout))
            success = False
            break
            
        #If there is still scope in moving ranges
        VwiggleScope = (V_Thresh >= V_Range.min) and (V_Thresh <= V_Range.max)
        RwiggleScope = (R_Thresh >= R_Range.min) and (R_Thresh <= R_Range.max)
        wiggleScope = VwiggleScope or RwiggleScope
        
        if wiggleScope:
            if (nzcount < minarea):
                if VwiggleScope:
                    V_Thresh -= (V_Range.dTh - ddth)
                if RwiggleScope:
                    R_Thresh -= (R_Range.dTh - ddth)
            else:
                if VwiggleScope:
                    V_Thresh += (V_Range.dTh - ddth)
                if RwiggleScope:
                    R_Thresh += (R_Range.dTh - ddth)
                    
            thresh_img = np.zeros_like(thresh_img)
            thresh_img[(V >= V_Thresh) | (R >= R_Thresh)] = 255
            nzcount = np.count_nonzero(thresh_img)
            
        else:
            print("Unable to find a good value in range. Bailing out!")
            success = False
            break    
    
    print("{:.2f} %cnt, {} steps, {} nzcnt".format(nzcount/total_pixels, counter, total_pixels))
        
    bin_img = np.dstack((thresh_img, thresh_img, thresh_img))
    config['R_best'] = R_Thresh
    config['V_best'] = V_Thresh

    if not success:
        nzcount = np.count_nonzero(thresh_img)
        thresh_img = np.zeros_like(thresh_img)
        config['R_best'] = config['R_init']
        config['V_best'] = config['V_init']
    return success, bin_img, config
