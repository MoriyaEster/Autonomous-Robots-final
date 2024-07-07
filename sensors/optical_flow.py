import random
from typing import List

class OpticalFlow:
    def __init__(self):
        self.position: List[float] = [0, 0]

    def get_position(self) -> List[float]:
        # Simulate position tracking with noise
        return [self.position[0] + random.uniform(-0.1, 0.1), self.position[1] + random.uniform(-0.1, 0.1)]

    def set_position(self, position: List[float]):
        self.position = position
