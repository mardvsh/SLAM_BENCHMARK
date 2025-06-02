import numpy as np
from scipy.ndimage import gaussian_filter1d

class TemporalSmoother:
    def __init__(self, lidar_data, sigma=1.0):
        self.lidar_data = lidar_data
        self.sigma = sigma

    def apply_smoothing(self):
        smoothed = gaussian_filter1d(self.lidar_data, sigma=self.sigma, axis=0)
        return smoothed

