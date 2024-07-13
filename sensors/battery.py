import time
class Battery:
    def __init__(self, duration: int):
        self.len_path: int = 0
        self.duration: int = duration
        self.charging: bool = False


class Battery:
    def __init__(self, duration: int):
        self.duration: int = duration
        self.power: int = duration

    def is_dead(self) -> bool:
        return self.power <= 0

    def get_remaining_time(self) -> float:
        return max(0, self.power)

    def battery_low(self):
        return self.power <= self.duration * 0.2

    def charging(self):
        self.power += 1
        time.sleep(0.001)
