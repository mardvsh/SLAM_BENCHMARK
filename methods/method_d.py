import numpy as np

class GaussianNoiseAdder:
    def __init__(self, lidar_data, noise_std=0.05):
        self.lidar_data = lidar_data
        self.noise_std = noise_std

    def apply_noise(self):
        noise = np.random.normal(0, self.noise_std, self.lidar_data.shape)
        return self.lidar_data + noise

