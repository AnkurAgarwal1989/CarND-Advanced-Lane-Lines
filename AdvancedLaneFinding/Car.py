'''
Class to hold Vehicle related data. This gets added to the Deque

'''
import Lane
import numpy as np
import utilities as laneUtils
import cv2

class Car():
    def __init__(self, lane_config, bt_config, camera_calibration, M, Minv):
        
        self.lane_cfg = lane_config
        self.bt_cfg = bt_config
        self.cam_calib = camera_calibration
        self.warp_M = M
        self.warp_Minv = Minv
        
        self.scale_X = lane_config['scale_X']
        self.scale_Y = lane_config['scale_Y']
        #tuple (X, Y)
        self.bin_image_shape = lane_config['bin_image_shape']
        
        ###Line Objects
        self.left_Line = Lane.Lane(lane_config, 'left')
        self.right_Line = Lane.Lane(lane_config, 'right')
        
        self.min_lane_width = lane_config['min_lane_width']
        self.max_lane_width = lane_config['max_lane_width']
        self.min_RoC = lane_config['min_RoC']
           
        # are lanes detected. 0-None, 1-left/right, 2-both
        self.lanes_detected = None
          
        # position from center. -ve car towards left, +ve car towards right
        self.dist_from_center = None
          
        #cars driving polynomial fit
        self.driving_lane = None
          
        #radius of curvature of the car in meters
        self.RoC = None
        
    def get_lanes(self, successFlag, bin_img):
        lane_img = np.zeros((self.bin_image_shape[1], self.bin_image_shape[0], 3))
        if successFlag:
            #Warp ROI to Bird's Eye view
            warped_bin_img = laneUtils.warp_image(bin_img, self.warp_M, self.bin_image_shape)
            #only need single channel
            warped_bin_img = warped_bin_img[:,:,0]

            left_lane_img, right_lane_img = self.update(warped_bin_img)
            lane_img = cv2.addWeighted(left_lane_img, 1, right_lane_img, 1, 0)
        else:
            _,_ = self.update(None)
        return lane_img
      
    def update(self, bin_img=None):
        if bin_img is None:
            self.left_Line.current_fit=None
            self.right_Line.current_fit=None
            self.left_Line.update_state()
            self.right_Line.update_state()
            return None, None
            
        out_left_img, left_found = self.left_Line.find_lane(bin_img)
        out_right_img, right_found = self.right_Line.find_lane(bin_img)
        
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
            self.calc_RoC()
            self.calc_dist_from_center()
        return out_left_img, out_right_img
    
    '''
    Function to check if the lines are intersecting or too close
    '''
    def sanity_check(self):
        print("sanity checking")
        dist = np.abs(self.left_Line.curr_x - self.right_Line.curr_x)
        min_dist_between_lanes = np.min(dist)
        max_dist_between_lanes = np.max(dist)
        print('min dist ', min_dist_between_lanes)
        print('max dist ', max_dist_between_lanes)
        if min_dist_between_lanes < self.min_lane_width or max_dist_between_lanes  > self.max_lane_width:
            print("Lines are too close or too far")
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
        
    #draw the left and right lanes on the colored image
    #Take the ROI image. Warp it bird's eye view.
    #Plot the lines and then warp it back to original view
    #Returns 
    def draw_lanes(self, roi):
        out_img = roi
        print("Drawing lane")
        
        if (self.is_right_lane_tracking() and self.is_left_lane_tracking()):
            print("Left and right are tracking")
            
            #Warp to bird's eye view
            warped_roi = laneUtils.warp_image(roi, self.warp_M, self.bin_image_shape)
        
            left_points = self.left_Line.calc_lane_points()
            right_points = np.flipud(self.right_Line.calc_lane_points())
            pts = np.vstack((left_points, right_points))
    
            # Draw the lane onto the warped blank image
            warped_roi = cv2.polylines(warped_roi, np.int32([left_points]), isClosed=False, color=(255, 255, 0), thickness=5)
            
            warped_roi = cv2.polylines(warped_roi, np.int32([right_points]), isClosed=False, color=(255, 255, 0), thickness=5)
            
            #out_img = cv2.addWeighted(left_img,0.5, right_img, 0.5, 0)
            warped_roi = cv2.fillPoly(warped_roi, np.int_([pts]), (0,255, 0))
            unwarped_lane_roi = laneUtils.warp_image(warped_roi, self.warp_Minv, (roi.shape[1], roi.shape[0]))
            
            _,g,_ = cv2.split(unwarped_lane_roi)
            #img2gray = unwarped_roi[unwarped_roicv2.cvtColor(unwarped_roi,cv2.COLOR_BGR2GRAY)
            ret, mask = cv2.threshold(g, 254, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(mask)
            
            #Blank out all area except fro lane on warped image with lanes drawn
            img_fg = cv2.bitwise_and(unwarped_lane_roi, unwarped_lane_roi, mask=mask)
            #blank out area in original roi
            img_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
            
            final_roi = cv2.add(img_bg, img_fg)
            out_img = cv2.addWeighted(final_roi, 0.4,roi, 0.6, 0)
        return out_img
        
