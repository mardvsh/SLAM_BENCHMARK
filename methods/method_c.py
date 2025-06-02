import numpy as np

class SpatialTemporalFilter:
    def __init__(self, lidar_data, odom_data):
        self.lidar_data = lidar_data
        self.odom_data = odom_data
    
    def apply_filter(self):
        filtered_data = []
        for scan in self.lidar_data:
            # Перемещение сканов в новое положение с учётом одометрии
            x_offset = self.odom_data['x']
            y_offset = self.odom_data['y']
            theta_offset = self.odom_data['theta']
            
            x_new = scan[0] + x_offset
            y_new = scan[1] + y_offset
            x_rot = x_new * np.cos(theta_offset) - y_new * np.sin(theta_offset)
            y_rot = x_new * np.sin(theta_offset) + y_new * np.cos(theta_offset)
            
            filtered_data.append([x_rot, y_rot])
        
        return np.array(filtered_data)

# Пример использования
lidar_data = np.random.rand(100, 2)  # Пример данных от лидара
odom_data = {'x': 1.0, 'y': 1.5, 'theta': np.pi/6}  # Данные одометрии
filter = SpatialTemporalFilter(lidar_data, odom_data)
filtered_scan = filter.apply_filter()
