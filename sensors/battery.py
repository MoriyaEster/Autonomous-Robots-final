import time


class Battery:
    def __init__(self, duration: float, charging_time: int = 5):
        self.duration: float = duration
        self.start_time: float = time.time()
        self.charging: bool = False
        self.charging_time: int = charging_time

    def is_dead(self) -> bool:
        return (time.time() - self.start_time) >= self.duration

    def is_going_to_empty(self) -> bool:
        return (time.time() - self.start_time) >= self.duration * 0.7

    def get_remaining_time(self) -> float:
        return max(0, self.duration - (time.time() - self.start_time))

    def charge(self):
        time.sleep(self.charging_time)
        print("charging")
        self.charging = True
        self.start_time = time.time()

