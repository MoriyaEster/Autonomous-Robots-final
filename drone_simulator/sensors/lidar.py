import random
from typing import List

from drone_simulator.core.map import Map

class Lidar:
    def __init__(self, max_distance: int =300):
        self.max_distance: int = max_distance

    def measure_distance(self, direction: int, environment: Map, position: List[float]) -> float:
        # Simulate distance measurement with noise
        distance = min(self.max_distance, environment.get_distance(direction, position))
        noise = random.uniform(-5, 5)  # Adding noise
        return max(0, distance + noise)
