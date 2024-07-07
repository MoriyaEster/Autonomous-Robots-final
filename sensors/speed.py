import random

class Speed:
    def __init__(self, speed: int, max_speed: float=20.0):
        self.max_speed: float = max_speed
        self.current_speed = speed

    def get_speed(self) -> float:
        noise = random.uniform(self.current_speed // 5, self.current_speed // 3)
        return min(self.current_speed + noise, self.max_speed)
