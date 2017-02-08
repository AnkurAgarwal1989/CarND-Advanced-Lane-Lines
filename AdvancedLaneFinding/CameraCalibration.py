#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 17:23:03 2017

@author: ankur
"""

import os
import glob
import cv2
import numpy as np
import pickle

def get_calibration(calib_name):
    with open(calib_name, "rb") as f:
        calibration = pickle.load(f)
        return calibration
    return None

###Class to perform Monocular Camera Calibration

class CameraCalibration:
    
    def __init__(self, pathToCalibrationImages, pathToCalibrationOutput=None, nx = 9,  ny = 6, num_images = 20, debug = False):
        self.nx = nx
        self.ny = ny
        self.num_images = num_images
        self.images_dir = pathToCalibrationImages
        if pathToCalibrationOutput is None:
            pathToCalibrationOutput = pathToCalibrationImages
        self.output_dir = pathToCalibrationOutput
        self.DEBUG = debug
        self.images_names = []
        
        self.calibration = {}
        
        if self.checkForImages():
            if self.runCalibration():
                self.writeCalibration();
                return self.calibration
        else:
            print("Unable to proceed with calibration. Exiting")
            return None
        
    #make sure we can find num_images in images_dir
    def checkForImages(self):
        self.images_names = glob.glob(os.path.join(self.images_dir, "calibration*.jpg"))
        if len(self.images_names) != self.num_images:
            print("Expected {} images, found {} calibration images.".format(self.num_images, len(self.images_names)))
            return False
        else:
            print("Found {} calibration images. Reading".format(self.num_images))
            return True
    
    def create_3D_obj_points(self):
        #Create a set of 3D object points nx*ny
        # (X,Y) vary according to checkerboard
        # Z remians 0
        obj = np.zeros((self.nx*self.ny,3), np.float32)
        obj[:, 0:2] = np.mgrid[0:self.nx, 0:self.ny].T.reshape(-1,2)
        return obj
    
    def get_image_corners(self):
        print("Calibration in progress");
        self.image_pts = []
        self.obj_pts = []
        good_images = 0
        img = cv2.imread(self.images_names[0])
        self.image_size = (img.shape[1], img.shape[0])
        for fname in self.images_names:
            img = cv2.imread(fname)

            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find the chessboard corners
            ret, img_corners = cv2.findChessboardCorners(gray, (self.nx, self.ny), None)
            
            # If found, draw corners
            if ret:
                if self.DEBUG:
                    debug_dir = os.path.join(self.images_dir, 'debug','corners')
                    if not os.path.isdir(debug_dir):
                        os.makedirs(debug_dir);
                    # Draw and display the corners
                    cv2.drawChessboardCorners(img, (self.nx, self.ny), img_corners, ret)
                    output_file_name = os.path.join(debug_dir, os.path.basename(fname))
                    cv2.imwrite(output_file_name, img=img)
                
                good_images += 1
                self.image_pts.append(img_corners)
                self.obj_pts.append(self.create_3D_obj_points())
                
            else:
                print("No corners found in image {} \n".format(fname))
        return good_images
                
    
    def runCalibration(self):
        good_images = self.get_image_corners()
        if good_images < 3:
            print("Not enough images.")       
            return                      
        # Do camera calibration given object points and image points
        ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(self.obj_pts, self.image_pts, self.image_size, None, None)
        
        if self.DEBUG:
            debug_dir = os.path.join(self.images_dir, 'debug','undistorted')
            if not os.path.isdir(debug_dir):
                os.makedirs(debug_dir);

            for fname in self.images_names:
                img = cv2.imread(fname)
                undist_img = cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
                debug_img = np.concatenate((img, undist_img), axis=1)
                output_file_name = os.path.join(debug_dir, os.path.basename(fname))
                cv2.imwrite(output_file_name, img=debug_img)
        
        return ret, self.mtx, self.dist
    
    def writeCalibration(self):
        #Save calibration file as a pickle
        calib_file_name = os.path.join(self.output_dir,"calibration_pickle.p")
        self.calibration["mtx"] = self.mtx
        self.calibration["dist"] = self.dist
        pickle.dump( self.calibration, open(calib_file_name, "wb"))
        print("Calibration written to {}".format(calib_file_name))


if __name__ == "__main__":
    print("Running Camera Calibration test")
    CameraCalibration("/home/ankur/UdacityND/CarND-Advanced-Lane-Lines/camera_cal", debug=False)
