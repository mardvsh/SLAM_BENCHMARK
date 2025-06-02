import numpy as np

class ScanWithOdometry:
    def __init__(self, odom_data, lidar_data):
        self.odom_data = odom_data  # Данные от одометрии
        self.lidar_data = lidar_data  # Данные от лидара
    
    def apply_odometry(self):
        # Математическое моделирование движения робота на основе одометрии
        x_offset = self.odom_data['x']
        y_offset = self.odom_data['y']
        theta_offset = self.odom_data['theta']
        
        # Применение смещения к данным лидара
        transformed_lidar_data = []
        for point in self.lidar_data:
            x_new = point[0] + x_offset
            y_new = point[1] + y_offset
            # Преобразование с учётом поворота
            x_rot = x_new * np.cos(theta_offset) - y_new * np.sin(theta_offset)
            y_rot = x_new * np.sin(theta_offset) + y_new * np.cos(theta_offset)
            transformed_lidar_data.append([x_rot, y_rot])
        
        return np.array(transformed_lidar_data)

# Пример использования
odom_data = {'x': 0.5, 'y': 0.2, 'theta': np.pi/4}  # Пример данных от одометрии
lidar_data = np.random.rand(100, 2)  # Пример случайных данных от лидара

scan = ScanWithOdometry(odom_data, lidar_data)
transformed_scan = scan.apply_odometry()
