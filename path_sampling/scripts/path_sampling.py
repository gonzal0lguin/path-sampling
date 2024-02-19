import numpy as np
import numpy.linalg as ln 
from scipy.signal import argrelextrema, find_peaks, savgol_filter
import matplotlib.pyplot as plt

class PathSampler(object):
    def __init__(self, waypoint_dst=0.5):
        self._waypoint_dst = waypoint_dst

    def sample_plan_uniform(self, path: np.ndarray):
        L = 0
        waypoints = []
        for i in range(1, len(path)):
            segment_l = self.calculate_segment_lenght(
                path[i-1][:2],
                path[i][:2]
            ) 
            L += segment_l
            
            if L >= self._waypoint_dst:
                waypoints.append(path[i])
                L -= self._waypoint_dst

        if L < self._waypoint_dst and len(waypoints)>0:
            waypoints.pop()
        waypoints.append(path[-1])

        return waypoints

    def sample_curvature(self, path, th=10):
        np.save('path4.npy', path)
        curvature_values = self.calculate_curvature(path)
        # Find indices of local maxima in curvature
        
        maxima_indices = argrelextrema(curvature_values, np.greater)[0]

        # Extract corresponding x and y values for the maxima
        wp = path[maxima_indices]


        # wp = np.concatenate([wp, path[-1]])
    
        return wp
    
    def sample_curve_from_gradient(self, path, th=0.001):

        x_values = path[:, 0]
        y_values = path[:, 1]

        y_smooth = savgol_filter(y_values, 51, 3)
        x_smooth = savgol_filter(x_values, 51, 3)

        # Calculate the gradient in both x and y directions
        dx = np.gradient(x_smooth)
        dy = np.gradient(y_smooth)

        # Calculate the magnitude of the gradient
        gradient_magnitude = np.abs(dx) + np.abs(dy)
        gradient_magnitude = -gradient_magnitude - np.min(-gradient_magnitude)


        # gradient_magnitude = low_pass_filter(-gradient_magnitude+0.0075, alpha=0.3)
        peaks, _ = find_peaks(gradient_magnitude, height=0.001, distance=100)

        return path[peaks]

    def sample_direction(self):
        pass
    
    @staticmethod
    def calculate_curvature(path):
        dx_dt = np.gradient(path[:, 0])
        dy_dt = np.gradient(path[:, 1])
        
        dx2_dt2 = np.gradient(dx_dt)
        dy2_dt2 = np.gradient(dy_dt)

        numerator = np.abs(dx_dt * dy2_dt2 - dy_dt * dx2_dt2)
        denominator = (dx_dt**2 + dy_dt**2)**(3/2)

        curvature_values = numerator / denominator

        return curvature_values #/ np.linalg.norm(curvature_values)

    @staticmethod
    def calculate_segment_lenght(s1, s2):
        x1, y1 = s1
        x2, y2 = s2
        return np.sqrt( (x1 - x2)**2 + (y1 - y2)**2 )
    
    @staticmethod
    def low_pass_filter(data, alpha=0.1):
        filtered_data = data.copy()
        for i in range(1, len(data)):
            filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
        return filtered_data