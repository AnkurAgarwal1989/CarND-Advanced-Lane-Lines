
# coding: utf-8

# In[1]:

import Line
import numpy as np


# In[29]:

def average_test(test_line, line_fits):
    for line_fit in line_fits:
        test_line.update(line_fit)
    average_fit = np.mean(line_fits, axis=0)
    assert (np.array_equal(test_line.best_fit, average_fit)), "Lines not being averaged correctly"
    return 1


# In[30]:

def no_track_test(test_line):
    for _ in range(15):
        test_line.update(None)
    assert (test_line.is_tracking==False), "Error in Tracking with no good frames"
    assert (test_line.no_lane_detected_frames == 15), "Tracking loss frames counter error"
    return 1


# In[36]:

def track_test(test_line, fit):
    test_line.update(fit)
    for _ in range(3):
        test_line.update(None)
    assert (test_line.is_tracking==True), "Error tracking lost too quickly"
    assert (np.array_equal(test_line.best_fit, fit)), "Best fit not good"
    assert (test_line.no_lane_detected_frames == 3), "Tracking loss frames counter error"
    return 1


# In[37]:

line_fits = np.random.rand(5, 3)
test_line = Line.Line(5, 10, 10)

print("Average test", average_test(test_line, line_fits))
print("Tracking Loss test", no_track_test(test_line))
print("Tracking test", track_test(test_line, line_fits[0]))


# In[ ]:



