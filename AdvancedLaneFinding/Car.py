'''
Class to hold Vehicle related data. This gets added to the Deque

'''
import Line
import numpy as np

class Car():
    def __init__(self, lane_config, bt_config, camera_calibration, M, Minv):
        
        self.lane_cfg = lane_config
        self.bt_cfg = bt_config
        self.cam_calib = camera_calibration
        self.warp_M = M
        self.warp_Minv = Minv
        
        N = lane_config['tracking_window']
        self.scale_X = lane_config['scale_X']
        self.scale_Y = lane_config['scale_Y']
        #tuple (X, Y)
        self.bin_image_shape = lane_config['bin_image_shape']
        
        ###Line Objects
        self.left_Line = Line.Line(N, self.scale_X, self.scale_Y, self.bin_image_shape)
        self.right_Line = Line.Line(N, self.scale_X, self.scale_Y, self.bin_image_shape)
           
        # are lanes detected. 0-None, 1-left/right, 2-both
        self.lanes_detected = None
          
        # position from center. -ve car towards left, +ve car towards right
        self.dist_from_center = None
          
        #cars driving polynomial fit
        self.driving_lane = None
          
        #radius of curvature of the car in meters
        self.RoC = None
      
    def update(self, new_left_fit=None, new_right_fit=None):
        self.left_Line.update(new_left_fit)
        self.right_Line.update(new_right_fit)
        if (self.is_right_lane_tracking() and self.is_left_lane_tracking()):
            self.calc_driving_lane_fit()
            self.calc_RoC()
            self.calc_dist_from_center()
        else:
         self.driving_lane = None
         self.RoC = None
         self.dist_from_center = None
         
      
    def calc_driving_lane_fit(self):
        self.driving_lane = np.mean([self.left_Line.best_fit, self.right_Line.best_fit], axis = 0)
        
        
    def calc_RoC(self):
        left_RoC = self.left_Line.radius_of_curvature
        right_RoC = self.right_Line.radius_of_curvature
        self.RoC = (left_RoC + right_RoC)/2
   
    def calc_dist_from_center(self):
        left_base = self.left_Line.base_pos
        right_base = self.right_Line.base_pos
        center_base = (left_base + right_base)/2
        self.dist_from_center = center_base - (self.bin_image_shape[0]*self.scale_X)
   
    def is_right_lane_tracking(self):
        return self.right_Line.is_tracking
   
    def is_left_lane_tracking(self):
        return self.left_Line.is_tracking
    
    #function returns left and right lane points in pixels for plotting
    ##list of left and right (x,y)
    def get_lane_points_pixels(self):
        #linespace for y, we know y is height pixels long
        points_pixels = []
        points_pixels.append(self.left_Line.calc_lane_points())
        points_pixels.append(self.right_Line.calc_lane_points())
        return points_pixels                                
        
