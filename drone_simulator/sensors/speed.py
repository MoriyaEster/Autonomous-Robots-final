import random

class Speed:
    def __init__(self, max_speed=2.0):
        self.max_speed = max_speed
        self.current_speed = 0

    def get_speed(self):
        return random.uniform(0.9, 1)  # Speed range from 1 to 3 units per move

    def set_speed(self, speed):
        self.current_speed = min(speed, self.max_speed)
