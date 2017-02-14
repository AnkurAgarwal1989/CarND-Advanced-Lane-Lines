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
import utilities as laneUtils

#tuple(X,Y) containing binary image
bin_img_shape = (640, 240)

lane_config = {}
lane_config['bin_image_shape'] = bin_img_shape
#lane_config['tracking_memory'] = 5##Harder challenge
lane_config['tracking_memory'] = 25#20
lane_config['scale_X'] = 3.7/420 #(meters/pixels)
lane_config['scale_Y'] = 3.048/33 #(meters/pixels)

lane_config['hist_height'] = 100
lane_config['sw_height'] = 24
lane_config['sw_width'] = 40
lane_config['num_white'] = 50
lane_config['search_width'] = 50
lane_config['min_track_length'] = 40
lane_config['no_track_frames'] = 15

lane_config['min_lane_width'] = 140
lane_config['max_lane_width'] = 520
#lane_config['min_RoC'] = 130##Harder Challenge
lane_config['min_RoC'] = 90

bt_config = {}
bt_config['R_Range'] = BT.ThresholdRange(140, 250, 5)
bt_config['V_Range'] = BT.ThresholdRange(140, 240, 5)
bt_config['R_init'] = 150
bt_config['V_init'] = 150
bt_config['R_best'] = 150
bt_config['V_best'] = 150
bt_config['bailout'] = 25
bt_config['minLane'] = 0.015
#bt_config['maxLane'] = 0.035 ##Harder challenge
bt_config['maxLane'] = 0.03

warp_config = {}
warp_config['P1'] = [402, 10]#[385, 0]
warp_config['P2'] = [530, 10]#[602, 0]
warp_config['P3'] = [930, 270]#[924, 203]
warp_config['P4'] = [15, 270]#[83, 203]
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
def _process(img, Car_obj, debug = False):
    
    #undistort image
    #DEBUG_OUT
    undist_img = laneUtils.undistort(img, Car_obj.cam_calib)

    #crop out ROI
    ROI_x1, ROI_y1 = 175, 450
    roi = laneUtils.get_ROI(undist_img, ROI_x1, ROI_y1)
  
    #Binary threshold image. We will use the values from the previous frameso save the config
    #DEBUG_OUT
    successFlag, bin_img, Car_obj.bt_cfg = BT.binary_threshold(roi, Car_obj.bt_cfg)
    
    #Calculate a good set of lane fits for the image
    #Update each lane object for filtering and tracking
    #DEBUG_OUT
    lane_img = Car_obj.get_lanes(successFlag, bin_img)
    
    #Draw lanes on the colored image
    #DEBUG_OUT
    lane_roi, ret = Car_obj.draw_lanes(roi);
            
    undist_img = laneUtils.set_ROI(undist_img, ROI_x1, ROI_y1, lane_roi)
    
    undist_img = Car_obj.annotate_image(undist_img, ret)
        
    return Car_obj, undist_img


def process_image(image_name, file_name, Car_obj, debug = False):
    image = cv2.imread(image_name)
    if image is None:
        print("Could not open image")
    print(image.shape)
    
    out_name =(os.path.splitext(os.path.basename(image_name))[0] + '_out.jpg')
    
    Car_obj, out_image = _process(image, Car_obj, debug)
    
    cv2.imwrite(out_name, out_image)
    return Car_obj, out_image

#Using imageio because OpenCV and 16.04Ubuntu have some weird issues with avi and mp4
def process_video(video_name, file_name, Car_obj, debug = False):
    out_video = None
    in_video = imageio.get_reader(video_name)
    fps = in_video.get_meta_data()['fps']

    out_name = file_name + '_out.avi'
    out_video = imageio.get_writer(out_name, fps = fps)
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out_video = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,240))
    for i, image in enumerate(in_video):
        #if i == 155:
            #break
        print()
        print('Frame # ', i)
        
        cv_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        start = time.time()
        Car_obj, cv_image = _process(cv_image, Car_obj, debug)
        end = time.time()
        dt = (end - start)
        print("{:.3f} s, {:.2f} FPS".format(dt, 1/dt))
        cv_image = cv2.putText(cv_image, str(i), (0,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        out_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        out_video.append_data(out_image)
        #out_video.append_data(cv_image)
    out_video.close()
    #out_video.release()
    return Car_obj, out_video
    
def get_data_type(name):
    print(name)
    name = os.path.basename(name)
    file_name, file_ext = os.path.splitext(name)
    print(file_ext)
    if file_ext in ['.jpg', '.png']:
        print("img")
        return file_name, 'image'
    if file_ext in ['.mp4', '.avi']:
        print("video")
        return file_name, 'video'
    
    
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
    print(input_name)
    #Create a car object to hold everything
    UNDCar = Car.Car(lane_config, bt_config, calibration, M, Minv)
    file_name, file_type = get_data_type(input_name)
    if (file_type is 'image'):
        UNDCar, out_image = process_image(input_name, file_name, UNDCar, True)
    elif (file_type is 'video'):
        UNDCar, out_image = process_video(input_name, file_name, UNDCar, True)
    #UNDCar, out_image = process_video(input_name, UNDCar)
    
    
if __name__ == "__main__":
    main()
