from time import sleep
import pygame
from pygame import Surface
from drone_simulator.core.drone import Drone
from drone_simulator.core.map import Map
class DroneSimulatorUI:
    def __init__(self, drone: Drone, map: Map):
        self.drone: Drone = drone
        self.map: Map = map
        self.screen: Surface = pygame.display.set_mode((800, 600))

    def update(self):
        # Update the display
        self.map.display_map(self.screen, self.drone.position, self.drone.graph)
        pygame.display.flip()

    def handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def run(self) -> None:
        running = True
        while running:
            running = self.handle_events()
            sensor_data = self.drone.sense(self.map)
            self.drone.decide_next_move(sensor_data)
            self.update()
            # uncomment for slower rendering
            # sleep(0.015)
