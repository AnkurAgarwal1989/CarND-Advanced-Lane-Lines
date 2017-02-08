'''
Class to hold Vehicle related data. This gets added to the Deque

'''
import Line

class Car():
    def __init__(self, config):
		
		N = config['tracking_window']
		scale_X = config['scale_X']
		scale_Y = config['scale_Y']
		#tuple (X, Y)
		self.image_shape = config['image_shape']
		
		self.left_Line = Line(N, scale_X, scale_Y)
		self.right_Line = Line(N, scale_X, scale_Y)
	
        # are lanes detected. 0-None, 1-left/right, 2-both
        self.lanes_detected = None
		
		# position from center. -ve car towards left, +ve car towards right
		self.dist_from_center = None
		     
        #polynomial coefficients of last left lane
        self.last_best_fit_left = None

		#polynomial coefficients of last right lane
        self.last_best_fit_right = None		
		
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
		self.dist_from_center = center_base - (self.image_shape[0]*scale_X)
	
	def is_right_lane_tracking(self):
		return self.right_Line.is_tracking
	
	def is_left_lane_tracking(self):
		return self.left_Line.is_tracking
