import random

class Speed:
    def __init__(self, drone_radius: int, max_speed: float=10.0):
        self.max_speed: float = max_speed
        self.drone_radius = drone_radius
        self.current_speed = 0

    def get_speed(self) -> float:
        noise = random.uniform(self.drone_radius // 5, self.drone_radius // 3)
        return max(self.drone_radius + noise, self.max_speed)
