import pygame

class Map:
    def __init__(self, map_file):
        self.map_file = map_file
        self.map_data = self.load_map()
        self.map_array = pygame.surfarray.array3d(self.map_data)  # Load the map as a 3D array (RGB)

    def load_map(self):
        # Load map from file and convert to a suitable format
        map_data = pygame.image.load(self.map_file)
        map_data = pygame.transform.scale(map_data, (800, 600))  # Scale to desired size
        return map_data

    def get_distance(self, direction, position):
        x, y = position
        radius = 10  # Drone radius
        max_distance = 300  # Sensor range

        distance = 0
        while distance < max_distance:
            if direction == 0:  # Front
                y -= 1
            elif direction == 270:  # Left
                x -= 1
            elif direction == 90:  # Right
                x += 1
            elif direction == 180:  # Backward
                y += 1

            distance += 1

            if self.map_array[int(x)][int(y)][0] == 0 and self.map_array[int(x)][int(y)][1] == 0 and self.map_array[int(x)][int(y)][2] == 0:
                break

        return distance

    def display_map(self, screen, drone_position):
        # Display map with current drone position and path
        screen.blit(self.map_data, (0, 0))
        pygame.draw.circle(screen, (0, 255, 0), drone_position, 10)  # Draw drone with radius 10
