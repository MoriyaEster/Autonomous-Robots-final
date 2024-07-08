
class Battery:
    def __init__(self, duration: int):
        self.len_path: int = 0
        self.duration: int = duration
        self.charging: bool = False

    def is_dead(self) -> bool:
        return self.len_path >= self.duration

    def is_going_to_empty(self) -> bool:
        return self.len_path >= self.duration * 0.8

    def get_remaining_buttery(self) -> float:
        return max(0, self.duration - self.len_path)

    def charge(self):
        self.charging = True
        self.len_path -= 1
        #time.sleep(0.01)

