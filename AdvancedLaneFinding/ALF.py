#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 7 13:35:66 2017

@author: ankur
"""
import os, sys
import cv2
import numpy as np

import Car
import BinaryThreshold as BT
import CameraCalibration as CC
import LaneModelling as LM
import utilities as laneUtils

#tuple(X,Y) containing binary image
bin_img_shape = (640, 240)

lane_config = {}
lane_config['tracking_window'] = 5
lane_config['scale_X'] = 3.7/430 #(meters/pixels)
lane_config['scale_Y'] = 3.048/34 #(meters/pixels)
lane_config['bin_image_shape'] = bin_img_shape

bt_config = {}
bt_config['R_Range'] = BT.ThresholdRange(140, 250, 5)
bt_config['V_Range'] = BT.ThresholdRange(140, 240, 5)
bt_config['R_init'] = 150
bt_config['V_init'] = 150
bt_config['bailout'] = 25
bt_config['minLane'] = 0.015
bt_config['maxLane'] = 0.025

warp_config = {}
warp_config['P1'] = [402, 10]
warp_config['P2'] = [530, 10]
warp_config['P3'] = [930, 270]
warp_config['P4'] = [15, 270]
warp_config['offset'] = 100
warp_config['bin_image_shape'] = lane_config['bin_image_shape']

#camera calibration
# try reading the calibration file, if we can't read one, calibrate the camera
def get_calibration():
    calibration = None
    camera_calibration_dir = os.path.join(os.getcwd(),"camera_cal")
    calib_name = os.path.join(camera_calibration_dir,"calibration_pickle.p")
    calibration = CC.read_calibration(calib_name)
    if calibration is None:
        print("Could not find existing calibration. Attempting to calibrate with new images")
        calibration_images_dir = os.path.join(os.getcwd(), "camera_cal")
        print("Using Calibration files from directory :\n ", calibration_images_dir)
        calibration = CC.CameraCalibration(calibration_images_dir)
    return calibration
        
          

# Function to process image. All steps of the pipeline are contained in this
# If debug is True, the output frame consists of diagnostic view
# if debug is False, the output image is the final output
# For each frame, we use the config data in the Car Object
# if required we update it and pass it back
def detect_lanes(img, Car_obj, debug = False):
    out_img = np.copy(img)
    
    #undistort image
    #DEBUG_OUT
    undist_img = laneUtils.undistort(img, Car_obj.cam_calib)
    
    #Init for left and right lane...start with None
    curr_left_fit = None
    curr_right_fit = None
    
    ROI_x1, ROI_y1 = 150, 500
    ROI_x2, ROI_y2 = img.shape[1]-ROI_x1, img.shape[0]
    
    roi = laneUtils.get_ROI(undist_img, ROI_x1, ROI_y1, ROI_x2, ROI_y2)

    #Binary threshold image. We will use the values from the previous frameso save the config
    #DEBUG_OUT
    successFlag, bin_img, Car_obj.bt_cfg = BT.binary_threshold(roi, Car_obj.bt_cfg)
    
    #if successful binary thresholding, go ahead with the lane detection
    #else go to next image, save time
    left_lane_img = np.zeros_like(bin_img)
    right_lane_img = np.zeros_like(bin_img)
    if successFlag:
        warped_bin_img = laneUtils.warp_image(bin_img, Car_obj.warp_M, Car_obj.bin_image_shape)
        
        if Car_obj.is_left_lane_tracking:
            print("Tracking left lane")
            left_lane_img, (left_y, left_x) = LM.trackLanes(warped_bin_img, curr_left_fit, 50)
        else:
            print("Detecting left lane in new frame")
            left_lane_img, (left_y, left_x) = LM.detectLanes(warped_bin_img)
    		
        if Car_obj.is_right_lane_tracking:
            print("Tracking right lane")
            right_lane_img, (right_y, right_x) = LM.trackLanes(warped_bin_img, curr_right_fit, 50)
        else:
            print("Detecting right lane in new frame")
            right_lane_img, (right_y, right_x) = LM.detectLanes(warped_bin_img, side='right')
            
        #combined binary images with lane detection debug output
        #DEBUG_OUT
        lane_img = cv2.bitwise_and(left_lane_img, right_lane_img)
            
        #new calculated line fits
        curr_left_fit = LM.fitLine(bin_img, (left_y, left_x), degree=2)
        curr_right_fit = LM.fitLine(bin_img, (right_y, right_x), degree=2)
        
        #Update the car object
        UNDCar.update(curr_left_fit, curr_right_fit)
    	
        #if we know the position of both left and right lanes
        #draw and reproject to original undistorted image
        if UNDCar.driving_lane is not None:
            print("Drawing Lane")
            #draw on warped_roi
            warped_roi = laneUtils.warp_image(roi, Car_obj.warp_M, Car_obj.bin_image_shape)
            #TODO Draw lane and add text here
            
            #unwarp and add to original image
            unwarped_roi = laneUtils.warp_image(warped_roi, Car_obj.warp_Minv, (roi.shape[1], roi.shape[0]))
            
            #Weighted addition to original color image
            out_roi = cv2.addWeighted(roi, 0.6, unwarped_roi, 0.4, 0)
            out_img[ROI_y1:ROI_y2, ROI_x1:ROI_x2, :] = out_roi
        
        else:
            print("No lane information. Drive carefully")
            #TODO Add warning text to out_img and return
    
    return Car_obj, out_img


def process_image(image_name, Car_obj, debug = False):
    image = cv2.imread(image_name)
    Car_obj, out_image = detect_lanes(image, Car_obj, debug)
    return Car_obj, out_image

def process_video(video_name, Car_obj, debug = False):
    out_video = None
    #for frame in video:
    #    Car_obj, out_frame = detect_lanes(frame, Car_obj, debug)
    return Car_obj, out_video
    
def main():
    
    # print command line arguments
    for arg in sys.argv[1:]:
        print(arg)
    input_name = sys.argv[1]
    #Get the calibration matrix    
    calibration = get_calibration()
    if calibration is None:
        print("Could not calibrate camera. Exiting")
        exit
        
    #We know the size of warp and unwarp images   
    #Get the Warp, Unwarp matrices
    M, Minv = laneUtils.get_warp_unwarp_matrices(warp_config)
    
    #Create a car object to hold everything
    UNDCar = Car.Car(lane_config, bt_config, calibration, M, Minv)
    
    '''if (input_name is image):
        process_image(input_name, UNDCar, True)
    elif (input_name is video):
        process_video(input_name, UNDCar, True)'''
    out_image = process_image(input_name, UNDCar)
    cv2.imwrite("outfile", out_image)
    
    
if __name__ == "__main__":
    main()