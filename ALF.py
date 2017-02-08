#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 7 13:35:66 2017

@author: ankur
"""
import os
import cv2

import Car
import BinaryThreshold as BT
import CameraCalibration as CC
import LaneModelling as LM
import utilities as laneUtils

#tuple(X,Y) containing binary image
bin_img_shape = (640, 240)

car_config = {}
car_config['tracking_window'] = 5
car_config['scale_X'] = 3.7/430 #(meters/pixels)
car_config['scale_Y'] = 3.048/34 #(meters/pixels)
car_config['image_shape'] = bin_img_shape

binary_thresh_config = {}
binary_thresh_config['R_Range'] = BT.ThresholdRange(140, 250, 5)
binary_thresh_config['V_Range'] = BT.ThresholdRange(140, 240, 5)
binary_thresh_config['R_init'] = 150
binary_thresh_config['V_init'] = 150
binary_thresh_config['bailout'] = 25

warp_config = {}
warp_config['P1'] = [402, 10]
warp_config['P2'] = [530, 10]
warp_config['P3'] = [930, 270]
warp_config['P4'] = [15, 270]
warp_config['offset'] = 100
warp_config['image_shape'] = bin_img_shape

#camera calibration
# try reading the calibration file, if we can't read one, calibrate the camera
camera_calibration_dir = os.path.join(os.getcwd(),"camera_cal")
calib_name = os.path.join(camera_calibration_dir,"calibration_pickle.p")
calibration = CC.get_calibration(calib_name)
if calibration is None:
    print("Could not find existing calibration. Attempting to calibrate with new images")
    calibration_images_dir = os.path.join(os.getcwd(), "camera_cal")
    print("Using Calibration files from Directory:\n ", calibration_images_dir)
    calibration = CC.CameraCalibration(calibration_images_dir)
    if calibration is None:
        print("Could not calibrate camera. Exiting")
        exit

          
#get warping, unwarping matrices
M, Minv = laneUtils.get_warp_unwarp_matrices(warp_config)


UNDCar = Car.Car(car_config)
curr_left_fit = None
curr_right_fit = None
for frame in video:
    #undistort image
    undist_img = laneUtils.undistort(frame, calibration)
	
    #Binary threshold image. We will use the values from the previous frameso save the config 
    successFlag, bin_img, binary_thresh_config = BT.binary_threshold(undist_img, binary_thresh_config)
    
    curr_left_fit = None
    curr_right_fit = None
    
    #if successful binary thresholding, go ahead with the lane detection
    #else go to next image, save time
    if successFlag:
        if UNDCar.is_left_lane_tracking:
            print("Tracking left lane")
            left_lane_img, (left_y, left_x) = LM.trackLanes(bin_img, curr_left_fit, 50)
        else:
            print("Detecting left lane in new frame")
            left_lane_img, (left_y, left_x) = LM.detectLanes(bin_img)
    		
        if UNDCar.is_right_lane_tracking:
            print("Tracking right lane")
            right_lane_img, (right_y, right_x) = LM.trackLanes(bin_img, curr_right_fit, 50)
        else:
            print("Detecting right lane in new frame")
            right_lane_img, (right_y, right_x) = LM.detectLanes(bin_img, side='right')
        
        curr_left_fit = LM.fitLine(bin_img, (left_y, left_x), degree=2)
        curr_right_fit = LM.fitLine(bin_img, (right_y, right_x), degree=2)
    		
    UNDCar.update(curr_left_fit, curr_right_fit)
	
    #if we know the position of both left and right lanes
    #draw and reproject on original undistorted image
    if UNDCar.driving_lane is not None:
        print("Drawing Lane")
		
		