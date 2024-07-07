import random

class Gyroscope:
    def __init__(self):
        self.rotation: int = 0

    def get_rotation(self) -> float:
        # Simulate rotation measurement with noise
        return self.rotation + random.uniform(-1, 1)

    def set_rotation(self, rotation: int) -> None:
        self.rotation = rotation
