'''
Class to hold Vehicle related data. This gets added to the Deque

'''
import Lane
import numpy as np
import cv2

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
        self.left_Line = Lane.Lane(N, self.scale_X, self.scale_Y, self.bin_image_shape, 'left')
        self.right_Line = Lane.Lane(N, self.scale_X, self.scale_Y, self.bin_image_shape, 'right')
           
        # are lanes detected. 0-None, 1-left/right, 2-both
        self.lanes_detected = None
          
        # position from center. -ve car towards left, +ve car towards right
        self.dist_from_center = None
          
        #cars driving polynomial fit
        self.driving_lane = None
          
        #radius of curvature of the car in meters
        self.RoC = None
      
    def update(self, bin_img=None):
        out_left_img, left_found = self.left_Line.find_lane(bin_img)
        out_right_img, right_found = self.right_Line.find_lane(bin_img, side='right')
        
        #if both left and right found, proceed with sanity check
        #If sanity check has failed, the current fit is bad
        if left_found and right_found:
            if not self.sanity_check():
                self.left_Line.current_fit=None
                self.right_Line.current_fit=None
        else:
            self.left_Line.current_fit=None
            self.right_Line.current_fit=None
            
        #After confirming, update the line state
        self.left_Line.update_state()
        self.right_Line.update_state()
        
        if (self.is_right_lane_tracking() and self.is_left_lane_tracking()):
            self.calc_driving_lane_fit()
            #self.calc_RoC()eee 
            self.calc_dist_from_center()
        else:
         self.driving_lane = None
         self.RoC = None
         self.dist_from_center = None
        return out_left_img, out_right_img
    
    '''
    Function to check if the lines are intersecting or too close
    '''
    def sanity_check(self):
        print("sanity checking")
        min_dist_between_lanes = np.min(np.abs(self.left_Line.curr_x - self.right_Line.curr_x))
        print('min dist ', min_dist_between_lanes)
        if min_dist_between_lanes < 200:
            print("Lines are too close or intersecting")
            return False
        
        print("RoC calc")
        left_roc = self.left_Line.curr_roc
        right_roc = self.right_Line.curr_roc
        roc_limit = 800
        print('lrc', left_roc, 'rrc', right_roc)
        if left_roc < roc_limit or right_roc < roc_limit:
            print("RoC too small")
            return False
        
        #if abs(left_roc - right_roc) > 1000:
            #return False
        
        
        return True
        
      
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
        
    #draw the left and right lanes on the colored warped image
    def draw_lanes(self, warped_img):
        out_img = warped_img
        if (self.is_right_lane_tracking() and self.is_left_lane_tracking()):
            left_points = self.left_Line.calc_lane_points()
            right_points = np.flipud(self.right_Line.calc_lane_points())
            pts = np.vstack((left_points, right_points))
    
            # Draw the lane onto the warped blank image
            warped_img = cv2.polylines(warped_img, np.int32([left_points]), isClosed=False, color=(255, 255, 0), thickness=5)
            
            warped_img = cv2.polylines(warped_img, np.int32([right_points]), isClosed=False, color=(255, 255, 0), thickness=5)
            
            #out_img = cv2.addWeighted(left_img,0.5, right_img, 0.5, 0)
            out_img = cv2.fillPoly(warped_img, np.int_([pts]), (0,255, 0))
            #out_img = cv2.addWeighted(out_img, 1, poly_img, 1, 0)
        return out_img
        
