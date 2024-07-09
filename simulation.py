import sys
from pathlib import Path
from time import sleep

# Determine the current directory
current_dir = Path(__file__).resolve().parent

# Determine the parent directory
parent_dir = current_dir.parent.parent

print(parent_dir)
# Add the parent directory to sys.path
sys.path.insert(0, str(parent_dir))

import pygame
from core.drone import Drone
from core.map import Map
from ui.interface import DroneSimulatorUI

class Simulation:
    def __init__(self, map_file: str, drone_radius: int, battery_duration_in_seconds: int, speed_in_pixels_per_move: int):
        pygame.init()
        self.map: Map = Map(map_file)
        self.drone: Drone = Drone(self.map, drone_radius, battery_duration_in_seconds, speed_in_pixels_per_move)
        self.ui: DroneSimulatorUI = DroneSimulatorUI(self.drone, self.map)

    def run(self):
        self.ui.run()
        pygame.quit()

if __name__ == "__main__":
    sim = Simulation('tracks/p13.png', 10, 5, 5)  # Update with the correct map path
    sim.run()
