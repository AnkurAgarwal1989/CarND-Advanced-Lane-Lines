'''
Class to hold Lane data.
We put user facing things here:
RoC, distance from center
'''
from collections import deque
import numpy as np
import cv2

class Lane():
    
    def __init__(self, lane_cfg, lane_type='left'):
        
        self.lane_type = lane_type
        self.tracking_memory = lane_cfg['tracking_memory']
        self.no_track_frames = lane_cfg['no_track_frames']
        
        #tuple(x,y)        
        self.bin_image_shape = lane_cfg['bin_image_shape']
        

        #scales in X and Y (meters/ pixels)
        self.scale_X = lane_cfg['scale_X']
        self.scale_Y = lane_cfg['scale_Y']
        
        self.hist_height = lane_cfg['hist_height']
        self.sw_height = lane_cfg['sw_height']
        self.sw_width = lane_cfg['sw_width']
        self.num_white = lane_cfg['num_white']

        self.search_width = lane_cfg['search_width']
        self.min_track_length = lane_cfg['min_track_length']

        self.min_RoC = lane_cfg['min_RoC']
        
        # is it tracking
        self.is_tracking = False
        
        #frames since detection/ tracking
        self.no_lane_detected_frames = 0
        
        # x values of the last n fits of the line
        self.recent_fits = deque(maxlen = self.tracking_memory)
        
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        
        #polynomial coefficients for the most recent fit
        self.current_fit = None
        self.curr_x = None
        #self.curr_roc = None
        
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float')
        
        #bposition of base of lane
        self.base_pos = None
        
	
    #when a few frames go by without new lanes, we need to restart from scratch. clear out older data
    #this flag will also be used by the histogram/ sw algorithm to do its thing
    def reinitialize(self):
        self.is_tracking = False
        self.recent_fits.clear()
        self.best_fit = None
        self.current_fit = None
        self.curr_x = None
        self.base_pos = None
        self.curr_roc = None
	
    '''
    fit_line: function to fit n degree polynomial to lane pixels
    input: lane_pixels: (x and y locations of the putative lane pixels)
	       degree of polynomial: 2 or 3
    output: line_fit: polynomial equation of line
    '''
    def fit_line(self, bin_img, lane_pixels, degree=2):
        self.current_fit = None
        self.curr_x = None
        out_img = np.dstack((bin_img, bin_img, bin_img))
        # Fit a n order polynomial
        lane_y, lane_x = lane_pixels
        self.current_fit = np.polyfit(lane_y, lane_x, degree)
        
        
        # Generate x and y values for plotting
        #linespace for y, we know y is height pixels long
        plot_y = np.linspace(0, bin_img.shape[0]-1, bin_img.shape[0])
        #get values of x for corresponding y
        fit_x = 0
        for deg,coeff in enumerate(self.current_fit[::-1]):
            fit_x += coeff*(plot_y**deg)
            
        self.curr_x = fit_x
        line_pts = np.vstack((self.curr_x, plot_y)).T
        cv2.polylines(out_img, np.int32([line_pts]), isClosed=False, color=(255, 0, 0), thickness=2)
        
        print("fit_line")
        print(self.current_fit)
        print(self.best_fit)
        
        if self.best_fit is not None:
            fit_x = 0
            for deg,coeff in enumerate(self.best_fit[::-1]):
                fit_x += coeff*(plot_y**deg)
                
            #self.curr_x = fit_x
            line_pts = np.vstack((self.curr_x, plot_y)).T
            cv2.polylines(out_img, np.int32([line_pts]), isClosed=False, color=(0, 255, 0), thickness=2)
        
        return out_img
        
        
    '''
    detectLanes: function to detect lanes using a histogram and sliding window
    input: bin_img: single channel
    hist_height: height of bottom area to detect peaks on
    sw_height: sliding window height
    sw_width: sliding window width
    side of image: 'left' or 'right'
    output: lane_pixels ((lane_y, lane_x)y and x locations of lane pixels)
    '''
    def detect_lane(self, bin_img):
        out_img = np.dstack((bin_img, bin_img, bin_img))
        lane_pixels = []
        base_strip = []
        midpoint = bin_img.shape[1]//2
        
        offset = 0
        # base strip to calc histogram on
        base_strip = bin_img[-self.hist_height : , :]
        
        if self.lane_type == 'left':
            histogram = np.sum(base_strip[:, :midpoint], 0, dtype=np.int)
        else:
            histogram = np.sum(base_strip[:, midpoint:], 0, dtype=np.int)
            offset = midpoint
        
        nz_pixels = bin_img.nonzero()
        nz_pixels_y = np.array(nz_pixels[0])
        nz_pixels_x = np.array(nz_pixels[1])
        
        current_x = np.argmax(histogram) + offset


        for sw in range(bin_img.shape[0]-1, 0, -self.sw_height):
            #points = (x,y)
            win_p1 = (current_x - self.sw_width, sw)
            win_p2 = (current_x + self.sw_width, sw - self.sw_height)
        
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
            if (len(sw_lane_pixels) >= self.num_white):
                current_x = np.int(np.mean(nz_pixels_x[sw_lane_pixels]))


        #each lane has some n (n<= imageheigt/windowheight lists of non zero pixels. concatenate to get x,y and then model lines)
        lane_pixels = np.concatenate(lane_pixels)

        lane_x = nz_pixels_x[lane_pixels]
        lane_y = nz_pixels_y[lane_pixels]

        if (len(lane_y) > 0):
           fit_img = self.fit_line(bin_img, (lane_y, lane_x), degree=2)
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
    def track_lane(self, bin_img):
        out_img = np.dstack((bin_img, bin_img, bin_img))
        
        lane_pixels = []    

        nz_pixels = bin_img.nonzero()
        nz_pixels_y = np.array(nz_pixels[0])
        nz_pixels_x = np.array(nz_pixels[1])
        
        if self.best_fit is None:
            track_fit = self.current_fit
        else:
            track_fit = self.best_fit
        
        # get values of x for corresponding y
        fit_x = 0
        for deg,coeff in enumerate(track_fit[::-1]):
            fit_x += coeff*(nz_pixels_y**deg)
            lane_pixels = ( (nz_pixels_x > (fit_x - self.search_width) ) & 
									   (nz_pixels_x < (fit_x + self.search_width) ) )
		
        #for Plotting
        plot_y = np.linspace(0, bin_img.shape[0]-1, bin_img.shape[0])
        #get values of x for corresponding y
        fit_x = 0
        for deg,coeff in enumerate(track_fit[::-1]):
            fit_x += coeff*(plot_y**deg)
        line_pts = np.vstack((fit_x, plot_y)).T
        cv2.polylines(out_img, np.int32([line_pts]), isClosed=False, color=(0, 0, 255), thickness=5)
        
        lane_x = nz_pixels_x[lane_pixels]
        lane_y = nz_pixels_y[lane_pixels]
        #out_img[lane_y, lane_x, :] = (255, 0, 0)
        print("track length y", len(np.unique(lane_y)), len(lane_y))
        if (len(lane_y) > 0 and len(np.unique(lane_y)) > self.min_track_length):
            fit_img = self.fit_line(bin_img, (lane_y, lane_x), degree=2)
            out_img = cv2.addWeighted(out_img, 1, fit_img, 1, 0)
        return out_img
        

    #function called by Car object whenever a good BT frame achieved
    #This function then decides which (detection or tracking) to call.
    def find_lane(self, bin_img):
        found_good_lane = False
        out_img = None
        self.current_fit = None
        if bin_img is None:
            self.current_fit = None
        else:
            if self.is_tracking:
                out_img = self.track_lane(bin_img)
            else:
                out_img = self.detect_lane(bin_img)
            if self.current_fit is not None:
                found_good_lane = self.verify_RoC()
                
        return out_img, found_good_lane
        
    #function called every time a new frame is forwared to the pipeline
    '''
    if the line fit is None, no good line was detected...see if we can keep using the older data
    if a line was fit, use that. In any case, if we have data (old or new), try to calc RoC and base position
    '''
    def update_state(self):
        if self.current_fit is None:
            self.no_lane_detected()
        else:
            self.lane_detected()
        
        if self.is_tracking:
            self.calc_RoC()
            self.calc_base_position()
            
    def no_lane_detected(self):
        self.curr_x = None
        self.curr_roc = None
        self.no_lane_detected_frames += 1
        print(self.lane_type ,' No lane detected ', self.no_lane_detected_frames)
        if self.no_lane_detected_frames >= self.no_track_frames:
            print(self.lane_type ,' Resetting')
            self.reinitialize()
            
    def lane_detected(self):
        
        self.is_tracking = True
        self.no_lane_detected_frames = 0
        self.recent_fits.append(self.current_fit)
        print(self.lane_type ,' detected')
        print(self.lane_type ,' averaging over ', len(self.recent_fits))
        self.best_fit = np.mean(self.recent_fits, axis = 0)
        #print('curr fit', self.current_fit)
        #print('Recent fits', self.recent_fits)
        
    #RoC in pixels
    # check if greater than threshold
    def verify_RoC(self):
        #self.radius_of_curvature = 1000
        print("RoC calc")
        y_eval = self.bin_image_shape[1]
        pix_roc = ((1 + (2*self.current_fit[0]*y_eval + self.current_fit[1])**2)**1.5) / np.absolute(2*self.current_fit[0])
        print(self.lane_type ,' roc', pix_roc)
        if pix_roc <= self.min_RoC:
            print(self.lane_type, "RoC too small")
            return False
        return True
        
    #RoC in meters
    def calc_RoC(self):
        #calculate with best fit
        self.radius_of_curvature = 10
    
    #position of base of the lane in meters	
    def calc_base_position(self):
        #calculate with best fit
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
