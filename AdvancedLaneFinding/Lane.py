'''
Class to hold Lane data.
We put user facing things here:
RoC, distance from center
'''
from collections import deque
import numpy as np
import cv2

class Lane():
    
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
	
    '''
    fit_line: function to fit n degree polynomial to lane pixels
    input: lane_pixels: (x and y locations of the putative lane pixels)
	       degree of polynomial: 2 or 3
    output: line_fit: polynomial equation of line
    '''
    def fit_line(self, bin_img, lane_pixels, degree=2):
        line_fit = None
        out_img = np.dstack((bin_img, bin_img, bin_img))
        # Fit a n order polynomial
        lane_y, lane_x = lane_pixels
        line_fit = np.polyfit(lane_y, lane_x, degree)
        
        
        # Generate x and y values for plotting
        #linespace for y, we know y is height pixels long
        plot_y = np.linspace(0, self.bin_image_shape[0]-1, self.bin_image_shape[0])
        #get values of x for corresponding y
        fit_x = 0
        for deg,coeff in enumerate(line_fit[::-1]):
            fit_x += coeff*(plot_y**deg)
    
        line_pts = np.vstack((fit_x, plot_y)).T

        cv2.polylines(out_img, np.int32([line_pts]), isClosed=False, color=(255, 0, 0), thickness=5)
        return out_img, line_fit
        
        
    '''
    detectLanes: function to detect lanes using a histogram and sliding window
    input: bin_img: single channel
    hist_height: height of bottom area to detect peaks on
    sw_height: sliding window height
    sw_width: sliding window width
    side of image: 'left' or 'right'
    output: lane_pixels ((lane_y, lane_x)y and x locations of lane pixels)
    '''
    def detect_lane(self, bin_img, hist_height=100, sw_height=24, sw_width=30, num_white=50, side='left'):
        out_img = np.dstack((bin_img, bin_img, bin_img))
        lane_pixels = []
        base_strip = []
        midpoint = bin_img.shape[1]//2
        
        offset = 0
        # base strip to calc histogram on
        base_strip = bin_img[-hist_height : , :]
        
        if side == 'left':
            histogram = np.sum(base_strip[:, :midpoint], 0, dtype=np.int)
        else:
            histogram = np.sum(base_strip[:, midpoint:], 0, dtype=np.int)
            offset = midpoint
        
        nz_pixels = bin_img.nonzero()
        nz_pixels_y = np.array(nz_pixels[0])
        nz_pixels_x = np.array(nz_pixels[1])
        
        current_x = np.argmax(histogram) + offset


        for sw in range(bin_img.shape[0]-1, 0, -sw_height):
            #points = (x,y)
            win_p1 = (current_x - sw_width, sw)
            win_p2 = (current_x + sw_width, sw - sw_height)
        
            #draw the line about the centroid
            cv2.line(out_img, (current_x, win_p2[1]), (current_x, win_p1[1] ), color = (0, 255, 0), thickness=2)
            #Draw the rectangle around the sliding window
            cv2.rectangle(out_img, win_p1, win_p2, color=(0, 0, 255))
            #print(sw, sw - sw_height, current_x)
            #find y pixels that belong to lanes
            sw_lane_pixels = ((nz_pixels_x >= win_p1[0]) & (nz_pixels_x < win_p2[0]) & (nz_pixels_y < win_p1[1]) & (nz_pixels_y >= win_p2[1])).nonzero()[0]


            #we found the pixels, now add them to a growing list
            lane_pixels.append(sw_lane_pixels)
        
            #if they pixels we found are good enough in number, use this new value to slide window
            if (len(sw_lane_pixels) >= num_white):
                current_x = np.int(np.mean(nz_pixels_x[sw_lane_pixels]))


        #each lane has some n (n<= imageheigt/windowheight lists of non zero pixels. concatenate to get x,y and then model lines)
        lane_pixels = np.concatenate(lane_pixels)

        lane_x = nz_pixels_x[lane_pixels]
        lane_y = nz_pixels_y[lane_pixels]

        if (len(lane_y) > 0):
           fit_img, self.current_fit = self.fit_line(bin_img, (lane_y, lane_x), degree=2)
           out_img = cv2.addWeighted(out_img, 1, fit_img, 1, 0)
        return out_img


    '''
    trackLanes: function to track (and detect) lane pixels using previous detected line equations
    input: bin_img: single channel
    line_fit: polynomial equation of line
    search_width: window width to search around
    side of image: 'left' or 'right'
    output: lane_pixels ((lane_y, lane_x)y and x locations of the putative lane pixels)
    '''
    def track_lane(self, bin_img, search_width=50):
        out_img = np.dstack((bin_img, bin_img, bin_img))
        
        lane_pixels = []    

        nz_pixels = bin_img.nonzero()
        nz_pixels_y = np.array(nz_pixels[0])
        nz_pixels_x = np.array(nz_pixels[1])
        
        # get values of x for corresponding y
        fit_x = 0
        for deg,coeff in enumerate(self.best_fit[::-1]):
            fit_x += coeff*(nz_pixels_y**deg)
            lane_pixels = ( (nz_pixels_x > (fit_x - search_width) ) & 
									   (nz_pixels_x < (fit_x + search_width) ) )
		
        #for Plotting
        plot_y = np.linspace(0, bin_img.shape[0]-1, bin_img.shape[0])
        #get values of x for corresponding y
        fit_x = 0
        for deg,coeff in enumerate(self.best_fit[::-1]):
            fit_x += coeff*(plot_y**deg)
        line_pts = np.vstack((fit_x, plot_y)).T
        cv2.polylines(out_img, np.int32([line_pts]), isClosed=False, color=(255, 255, 0), thickness=5)
        
        lane_x = nz_pixels_x[lane_pixels]
        lane_y = nz_pixels_y[lane_pixels]
        out_img[lane_y, lane_x, :] = (255, 0, 0)
        
        if (len(lane_y) > 0):
            fit_img, self.current_fit = self.fit_line(bin_img, (lane_y, lane_x), degree=2)
            out_img = cv2.addWeighted(out_img, 1, fit_img, 1, 0)
        return out_img
        

    #function called by Car object whenever a good BT frame achieved
    #This function then decides which (detection or tracking) to call.
    def find_lane(self, bin_img, side='left'):
        out_img = None
        if bin_img is None:
            self.current_fit = None
        else:
            if self.is_tracking:
                out_img = self.track_lane(bin_img)
            else:
                out_img = self.detect_lane(bin_img, side=side)
                
        self.update_state()
        return out_img
        
    #function called every time a new frame is forwared to the pipeline
    '''
    if the line fit is None, no line was detected...see if we can keep using the older data
    if a line was fit, use that. In any case, if we have data (old or new), try to calc RoC and base position
    '''
    def update_state(self):
        if self.current_fit is None:
            self.no_lane_detected()
        else:
            self.lane_detected()
        
        if self.is_tracking:
            self.calc_RoC(10)
            self.calc_base_position()
            
    def no_lane_detected(self):
        self.no_lane_detected_frames += 1
        if self.no_lane_detected_frames == 5:
            self.reinitialize()
            
    def lane_detected(self):
        self.is_tracking = True
        self.no_lane_detected_frames = 0
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
