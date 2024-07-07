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
from drone_simulator.core.drone import Drone
from drone_simulator.core.map import Map
from drone_simulator.ui.interface import DroneSimulatorUI

class Simulation:
    def __init__(self, map_file: str, drone_radius: int):
        pygame.init()
        self.map: Map = Map(map_file)
        self.drone: Drone = Drone(self.map, drone_radius)
        self.ui: DroneSimulatorUI = DroneSimulatorUI(self.drone, self.map)

    def run(self):
        self.ui.run()
        pygame.quit()

if __name__ == "__main__":
    sim = Simulation('p11.png', 10)  # Update with the correct map path
    sim.run()
