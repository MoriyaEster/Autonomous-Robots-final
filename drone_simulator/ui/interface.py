import pygame

class DroneSimulatorUI:
    def __init__(self, drone, map):
        self.drone = drone
        self.map = map
        self.screen = pygame.display.set_mode((800, 600))

    def update(self):
        # Update the display
        self.map.display_map(self.screen, self.drone.position, self.drone.graph)
        pygame.display.flip()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def run(self):
        running = True
        while running:
            running = self.handle_events()
            sensor_data = self.drone.sense(self.map)
            self.drone.decide_next_move(sensor_data)
            self.update()
