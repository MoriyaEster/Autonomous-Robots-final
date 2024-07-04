import pygame
import networkx as nx

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

    def mark_sensor_area(self, position, distance, direction):
        x, y = position
        for i in range(int(distance)):
            if direction == 0:  # Front
                y -= 1
            elif direction == 270:  # Left
                x -= 1
            elif direction == 90:  # Right
                x += 1
            elif direction == 180:  # Backward
                y += 1

            if 0 <= x < self.map_array.shape[0] and 0 <= y < self.map_array.shape[1]:
                # Ensure we only color non-obstacle areas
                if self.map_array[int(x)][int(y)][0] != 0 or self.map_array[int(x)][int(y)][1] != 0 or \
                        self.map_array[int(x)][int(y)][2] != 0:
                    self.map_array[int(x)][int(y)] = [255, 255, 0]  # Color in yellow

    def display_map(self, screen, drone_position, graph):
        # Create a surface from the updated map_array
        updated_map_surface = pygame.surfarray.make_surface(self.map_array)
        screen.blit(updated_map_surface, (0, 0))
        pygame.draw.circle(screen, (0, 255, 0), drone_position, 10)  # Draw drone with radius 10

        # Draw the graph
        for edge in graph.edges:
            start_pos = graph.nodes[edge[0]]['position']
            end_pos = graph.nodes[edge[1]]['position']
            pygame.draw.line(screen, (0, 0, 255), start_pos, end_pos, 1)

        for node in graph.nodes:
            pos = graph.nodes[node]['position']
            pygame.draw.circle(screen, (0, 0, 255), pos, 5)
