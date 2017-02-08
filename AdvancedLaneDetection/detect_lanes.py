import numpy as np
import Car
import Line
 
#tuple(X,Y) containing binary image
bin_img_shape = (640, 240)

car_config = {}
car_config['tracking_window'] = 5
car_config['scale_X'] = 30/1000
car_config['scale_Y'] = 30/1000
car_config['image_shape'] = bin_img_shape

binary_thresh_config = {}
binary_thresh_config['RThresh'] = Threshold()
binary_thresh_config['VThresh'] = Threshold()
binary_thresh_config['bailout'] = 25

#get camera calibration.

warp_config = {}
warp_config['P1'] = (point)
warp_config['P2'] = (point)
warp_config['P3'] = (point)
warp_config['P4'] = (point)
car_config['image_shape'] = bin_img_shape

#get distortion matrix
#get inverse distortion matrix


UNDCar = Car.Car(config)
left_fit = None
right_fit = None
for frame in video:
	undist_img = undistort(frame)
	bin_img = binary_threshold(undist_img)
	if UNDCar.is_left_lane_tracking:
		print("Tracking left lane")
	else:
		print("Detecting left lane in new frame")
		
	if UNDCar.is_right_lane_tracking:
		print("Tracking right lane")
	else:
		print("Detecting right lane in new frame")
		
	UNDCar.update(left_fit, right_fit)
	
	#if we know the position of both left and right lanes
	#draw and reproject on original undistorted image
	if UNDCar.driving_lane is not None:
		print("Drawing Lane")
		
		