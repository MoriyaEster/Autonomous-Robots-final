import random

class Gyroscope:
    def __init__(self):
        self.rotation: int = 0

    def set_rotation(self, rotation: int) -> None:
        self.rotation = rotation