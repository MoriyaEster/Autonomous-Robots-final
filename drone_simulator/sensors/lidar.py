import random

class Lidar:
    def __init__(self, max_distance=300):
        self.max_distance = max_distance

    def measure_distance(self, direction, environment, position):
        # Simulate distance measurement with noise
        distance = min(self.max_distance, environment.get_distance(direction, position))
        noise = random.uniform(-5, 5)  # Adding noise
        return max(0, int(distance + noise))
