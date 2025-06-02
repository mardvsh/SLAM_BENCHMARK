import numpy as np

class WeightedFilter:
    def __init__(self, lidar_data, max_distance=10.0, min_weight=0.5):
        self.lidar_data = lidar_data
        self.max_distance = max_distance
        self.min_weight = min_weight
    
    def apply_filter(self):
        filtered_data = []
        for point in self.lidar_data:
            distance = np.linalg.norm(point)
            weight = max(0, 1 - (distance / self.max_distance))
            if weight >= self.min_weight:
                filtered_data.append([point[0], point[1], weight])
        
        return np.array(filtered_data)

# Пример использования
lidar_data = np.random.rand(100, 2) * 10  # Пример случайных данных от лидара
filter = WeightedFilter(lidar_data)
filtered_scan = filter.apply_filter()

