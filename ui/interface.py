from time import sleep
import pygame
from pygame import Surface
from core.drone import Drone
from core.map import Map
from utils.helper import draw_battery_dead_message, draw_battery_bar, draw_battery_low_message


class DroneSimulatorUI:
    def __init__(self, drone: Drone, map: Map):
        self.drone: Drone = drone
        self.map: Map = map
        self.screen: Surface = pygame.display.set_mode((800, 600))

    def update(self):
        # Update the display
        self.map.display_map(self.screen, self.drone.position, self.drone.graph, self.drone.radius)
        remaining_time = self.drone.battery.get_remaining_time()
        total_time = self.drone.battery.duration
        draw_battery_bar(self.screen, remaining_time, total_time)

        if self.drone.battery.is_dead():
            draw_battery_dead_message(self.screen)
        elif self.drone.battery.is_going_to_empty():
            draw_battery_low_message(self.screen)

        pygame.display.flip()

    def handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def run(self) -> None:
        running = True
        while running:
            if not self.handle_events():
                running = False
                continue
            sensor_data = self.drone.sense(self.map)
            self.drone.decide_next_move(sensor_data)
            self.update()
            if self.drone.battery.is_dead():
                print("Battery died")
            elif self.drone.battery.is_going_to_empty():
                print("Battery low")
            # Uncomment for slower rendering
            # sleep(0.015)
