import time


class Battery:
    def __init__(self, duration: int):
        self.duration: int = duration  # in moves
        self.power: int = duration

    def is_dead(self) -> bool:
        return self.power <= 0

    def get_remaining_moves(self) -> float:
        return max(0, self.power)

    def battery_low(self):
        """
        Checks if the drone's battery is low.

        Returns:
        bool: True if the battery power is less than or equal to 20% of its duration, False otherwise.
        """
        return self.power <= self.duration * 0.2

    def charging(self):
        """
        Increases the drone's power by 1 and pauses briefly to simulate charging.
        """
        self.power += 1
        time.sleep(0.001)  # to simulate charging
