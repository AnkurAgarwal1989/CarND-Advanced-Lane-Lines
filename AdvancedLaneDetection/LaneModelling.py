# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 20:35:47 2017

@author: ankur
"""

import cv2
import numpy as np
# Functions to detect lanes and fit polynomial over lane lines
# For first frame or lost tracking: use histogram and sliding window
# If tracking. use previous line equation for targeted detection

'''
detectLanes: function to detect lanes using a histogram and sliding window
input: bin_img: single channel
       hist_height: height of bottom area to detect peaks on
       sw_height: sliding window height
       sw_width: sliding window width
       side of image: 'left' or 'right'
output: lane_pixels ((lane_y, lane_x)y and x locations of lane pixels)
'''


def detectLanes(bin_img, hist_height, sw_height, sw_width, num_white, side='left'):
    out_img = np.dstack((bin_img, bin_img, bin_img))
    
    lane_pixels = []
    base_strip = []
    midpoint = bin_img.shape[1]//2

    offset = 0
    
    # base strip to calc histogram on
    base_strip = bin_img[-hist_height : , :]
    
    if side == 'left':
        histogram = np.sum(base_strip[:, :midpoint], 0, dtype=np.int)
    else:
        histogram = np.sum(base_strip[:, midpoint:], 0, dtype=np.int)
        offset = midpoint
        
    nz_pixels = bin_img.nonzero()
    nz_pixels_y = np.array(nz_pixels[0])
    nz_pixels_x = np.array(nz_pixels[1])
    
    current_x = np.argmax(histogram) + offset
        
    
    for sw in range(bin_img.shape[0]-1, 0, -sw_height):
        #points = (x,y)
        win_p1 = (current_x - sw_width, sw)
        win_p2 = (current_x + sw_width, sw - sw_height)
        
        #draw the line about the centroid
        cv2.line(out_img, (current_x, win_p2[1]), (current_x, win_p1[1] ), color = (0, 255, 0), thickness=2)
        #Draw the rectangle around the sliding window
        cv2.rectangle(out_img, win_p1, win_p2, color=(255, 0, 0))
        print(sw, sw - sw_height, current_x)
        #find y pixels that belong to lanes
        sw_lane_pixels = ((nz_pixels_x >= win_p1[0]) & (nz_pixels_x < win_p2[0]) & 
                         (nz_pixels_y < win_p1[1]) & (nz_pixels_y >= win_p2[1])).nonzero()[0]
        
        
        #we found the pixels, now add them to a growing list
        lane_pixels.append(sw_lane_pixels)
        
        #if they pixels we found are good enough in number, use this new value to slide window
        if (len(sw_lane_pixels) >= num_white):
            current_x = np.int(np.mean(nz_pixels_x[sw_lane_pixels]))


    #each lane has some n (n<= imageheigt/windowheight lists of non zero pixels. concatenate to get x,y and then model lines)
    lane_pixels = np.concatenate(lane_pixels)
    
    lane_x = nz_pixels_x[lane_pixels]
    lane_y = nz_pixels_y[lane_pixels]
    return out_img, (lane_y, lane_x)


'''
trackLanes: function to track (and detect) lane pixels using previous detected line equations
input: bin_img: single channel
       line_fit: polynomial equation of line
       search_width: window width to search around
       side of image: 'left' or 'right'
output: lane_pixels ((lane_y, lane_x)y and x locations of the putative lane pixels)
'''
def trackLanes(bin_img, line_fit, search_width):
    out_img = np.dstack((bin_img, bin_img, bin_img))
    
    lane_pixels = []    
    
    nz_pixels = bin_img.nonzero()
    nz_pixels_y = np.array(nz_pixels[0])
    nz_pixels_x = np.array(nz_pixels[1])
    
    # get values of x for corresponding y
    fit_x = 0
    for deg,coeff in enumerate(line_fit[::-1]):
        fit_x += coeff*(nz_pixels_y**deg)
    
    lane_pixels = ( (nz_pixels_x > (fit_x - search_width) ) & 
                                   (nz_pixels_x < (fit_x + search_width) ) )
    
    lane_x = nz_pixels_x[lane_pixels]
    lane_y = nz_pixels_y[lane_pixels]
    out_img[lane_y, lane_x, :] = (0, 255, 255)
    return out_img, (lane_y, lane_x)


'''
trackLanes: function to fit n degree polynomial to lane pixels
input: lane_pixels: (x and y locations of the putative lane pixels)
       degree of polynomial: 2 or 3
output: line_fit: polynomial equation of line
'''
def fitLine(img, lane_pixels, degree=2):
    line_fit = None
    out_img = np.copy(img)
    # Fit a n order polynomial
    lane_y, lane_x = lane_pixels
    line_fit = np.polyfit(lane_y, lane_x, degree)
    
    # Generate x and y values for plotting
    #linespace for y, we know y is height pixels long
    plot_y = np.linspace(0, img.shape[0]-1, img.shape[0])
    #get values of x for corresponding y
    fit_x = 0
    for deg,coeff in enumerate(line_fit[::-1]):
        fit_x += coeff*(plot_y**deg)
    #fit_x = line_fit[0]*plot_y**2 + line_fit[1]*plot_y + line_fit[2]
    line_pts = np.vstack((fit_x, plot_y)).T
    
    cv2.polylines(out_img, np.int32([line_pts]), isClosed=False, color=(255, 255, 0), thickness=4)
    return out_img, line_fit

'''
getRoC: function returns Radius of Curvature(meters) for a given equation of line
input: equation of line
        y: y pixel location of curve
        scale_x = meters/pixel
        scale_y = meters/pixel
        
'''
def getRoC(line_fit, y):
    radius = 1000
    return radius