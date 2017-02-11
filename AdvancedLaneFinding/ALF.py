#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 7 13:35:66 2017

@author: ankur
"""
import os, sys
import cv2
import numpy as np
import imageio
import time

import Car
import BinaryThreshold as BT
import CameraCalibration as CC
import LaneModelling as LM
import utilities as laneUtils

#tuple(X,Y) containing binary image
bin_img_shape = (640, 240)

lane_config = {}
lane_config['tracking_window'] = 30
lane_config['scale_X'] = 3.7/427 #(meters/pixels)
lane_config['scale_Y'] = 3.048/72 #(meters/pixels)
lane_config['bin_image_shape'] = bin_img_shape

bt_config = {}
bt_config['R_Range'] = BT.ThresholdRange(140, 250, 5)
bt_config['V_Range'] = BT.ThresholdRange(140, 240, 5)
bt_config['R_init'] = 150
bt_config['V_init'] = 150
bt_config['R_best'] = 150
bt_config['V_best'] = 150
bt_config['bailout'] = 25
bt_config['minLane'] = 0.015
bt_config['maxLane'] = 0.025

warp_config = {}
warp_config['P1'] = [385, 0]
warp_config['P2'] = [602, 0]
warp_config['P3'] = [924, 203]
warp_config['P4'] = [83, 203]
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
def draw_lanes(img, Car_obj, debug = False):
    out_img = np.copy(img)
    
    #undistort image
    #DEBUG_OUT
    undist_img = laneUtils.undistort(img, Car_obj.cam_calib)
    
    #crop out ROI
    ROI_x1, ROI_y1 = 150, 490
    ROI_x2, ROI_y2 = img.shape[1]-ROI_x1, img.shape[0]
    roi = laneUtils.get_ROI(undist_img, ROI_x1, ROI_y1, ROI_x2, ROI_y2)
    #Binary threshold image. We will use the values from the previous frameso save the config
    #DEBUG_OUT
    successFlag, bin_img, Car_obj.bt_cfg = BT.binary_threshold(roi, Car_obj.bt_cfg)
    
    #if successful binary thresholding, go ahead with the lane detection
    #else go to next image, save time
    left_lane_img = np.zeros((Car_obj.bin_image_shape[1], Car_obj.bin_image_shape[0], 3), dtype=np.uint8)
    right_lane_img = np.zeros_like(left_lane_img)
    lane_img = np.zeros_like(left_lane_img)
    
    if successFlag:
        #Warp ROI to Bird's Eye view
        warped_bin_img = laneUtils.warp_image(bin_img, Car_obj.warp_M, Car_obj.bin_image_shape)
        #only need single channel
        warped_bin_img = warped_bin_img[:,:,0]
        
        #ToDo
        #Method detects or tracks lanes depending on state
        #DEBUG_OUT
        left_lane_img, right_lane_img = Car_obj.update(warped_bin_img)
        lane_img = cv2.addWeighted(left_lane_img, 1, right_lane_img, 1, 0)
    warped_color_roi = laneUtils.warp_image(roi, Car_obj.warp_M, Car_obj.bin_image_shape)
    out_img = Car_obj.draw_lanes(warped_color_roi);
    unwarped_roi = laneUtils.warp_image(out_img, Car_obj.warp_Minv, (roi.shape[1], roi.shape[0]))
    return Car_obj, lane_img
#    	2
#        #if we know the position of both left and right lanes
#        #draw and reproject to original undistorted image
#        if Car_obj.driving_lane is not None:
#            print("Drawing Lane")
#            #draw on warped_roi
#            warped_roi = laneUtils.warp_image(roi, Car_obj.warp_M, Car_obj.bin_image_shape)
#            #TODO Draw lane and add text here
#            
#            #unwarp and add to original image
#            unwarped_roi = laneUtils.warp_image(warped_roi, Car_obj.warp_Minv, (roi.shape[1], roi.shape[0]))
#            
#            #Weighted addition to original color image
#            out_roi = cv2.addWeighted(roi, 0.6, unwarped_roi, 0.4, 0)
#            out_img[ROI_y1:ROI_y2, ROI_x1:ROI_x2, :] = out_roi
#        
#        else:
#            print("No lane information. Drive carefully")
#            #TODO Add warning text to out_img and return
#    
#    return Car_obj, lane_img


def process_image(image_name, Car_obj, debug = False):
    image = cv2.imread(image_name)
    if image is None:
        print("Could not open image")
    print(image.shape)
    
    out_name =(os.path.splitext(os.path.basename(image_name))[0] + '_out.jpg')
    
    Car_obj, out_image = draw_lanes(image, Car_obj, debug)
    
    cv2.imwrite(out_name, out_image)
    return Car_obj, out_image

#Using imageio because OpenCV and 16.04Ubuntu have some weird issues with avi and mp4
def process_video(video_name, Car_obj, debug = False):
    out_video = None
    in_video = imageio.get_reader(video_name)
    fps = in_video.get_meta_data()['fps']

    out_name =(os.path.splitext(os.path.basename(video_name))[0] + '_out.avi')
    out_video = imageio.get_writer(out_name, fps = fps)
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out_video = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,240))
    for i, image in enumerate(in_video):
        #if i == 100:
            #break
        cv_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        start = time.time()
        Car_obj, cv_image = draw_lanes(cv_image, Car_obj, debug)
        end = time.time()
        dt = (end - start)
        #print("{:.3f} s, {:.2f} FPS".format(dt, 1/dt))
        out_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        out_video.append_data(out_image)
        #out_video.write(cv_image)
    out_video.close()
    #out_video.release()
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
    UNDCar, out_image = process_video(input_name, UNDCar)
    
    
if __name__ == "__main__":
    main()
