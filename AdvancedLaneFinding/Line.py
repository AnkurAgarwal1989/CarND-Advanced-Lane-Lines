'''
Class to hold Line data.
We put user facing things here:
RoC, distance from center
'''
from collections import deque
import numpy as np

class Line():
    def __init__(self, N, scale_X, scale_Y, img_shape):
        self.frame_memory = N

        # is it tracking
        self.is_tracking = False
        
        #frames since detection/ tracking
        self.no_lane_detected_frames = 0
        
        # x values of the last n fits of the line
        self.recent_fits = deque(maxlen = N)
        
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        
        #polynomial coefficients for the most recent fit
        self.current_fit = None
        
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float')
        
        #bposition of base of lane
        self.base_pos = None
        
        #scales in X and Y (meters/ pixels)
        self.scale_X = scale_X
        self.scale_Y = scale_Y
                
        self.bin_image_shape = img_shape
	
	#when a few frames go by without new lanes, we need to restart from scratch. clear out older data
	#this flag will also be used by the histogram/ sw algorithm to do its thing
    def reinitialize(self):
        self.is_tracking = False
        self.recent_fits.clear()
        self.best_fit = None
        self.current_fit = None
        self.base_pos = None
	
    	#function called every time a new frame is forwared to the pipeline
    '''
    if the line fit is None, no line was detected...see if we can keep using the older data
    if a line was fit, use that. In any case, if we have data (old or new), try to calc RoC and base position
    '''
    def update(self, curr_fit):
        if curr_fit is None:
            self.no_lane_detected()
        else:
            self.lane_detected(curr_fit)
        
        if self.is_tracking:
            self.calc_RoC(10)
            self.calc_base_position()
            
    def no_lane_detected(self):
        self.no_lane_detected_frames += 1
        if self.no_lane_detected_frames == 5:
            self.reinitialize()
            
    def lane_detected(self, curr_fit):
        self.is_tracking = True
        self.no_lane_detected_frames = 0
        self.current_fit = curr_fit
        self.recent_fits.append(self.current_fit)
        
        self.best_fit = np.mean(self.recent_fits, axis = 0)
        
    #RoC in meters
    def calc_RoC(self, y):
        self.radius_of_curvature = 1000
        
    #position of base of the lane in meters	
    def calc_base_position(self):
        self.base_pos = 15
            
    #calc lane points in pixels for plotting
    #vertical array of (x,y)
    def calc_lane_points(self):
        plot_y = np.linspace(0, self.bin_image_shape[0]-1, self.bin_image_shape[0])
        # Generate x values for each y for plotting
        fit_x = 0
        for deg,coeff in enumerate(self.best_fit[::-1]):
            fit_x += coeff*(plot_y**deg)
        line_pts = np.vstack((fit_x, plot_y)).T
        return line_pts