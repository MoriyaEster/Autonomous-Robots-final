import time


class Battery:
    def __init__(self, duration: float):
        self.duration: float = duration
        self.start_time: float = time.time()

    def is_dead(self) -> bool:
        return (time.time() - self.start_time) >= self.duration

    def get_remaining_time(self) -> float:
        return max(0, self.duration - (time.time() - self.start_time))
