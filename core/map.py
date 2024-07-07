import math
from typing import List
import pygame
from pygame import Surface
import networkx as nx


class Map:
    def __init__(self, map_file: str):
        """
               Initialize a Map object.

               Args:
               - map_file (str): Path to the image file representing the map.

               Attributes:
               - map_file (str): Path to the map image file.
               - map_data (Surface): Loaded map data as a Pygame Surface.
               - map_array (ndarray): Map data converted to a 3D array (RGB format).
               - drone_history_locations (List[tuple[int, int]]): List to store historical locations of the drone.

               Notes:
               - Initializes the map by loading and scaling the image file.
               - Converts the map data into a 3D array format for manipulation.
               """
        self.map_file: str = map_file
        self.map_data: Surface = self.load_map()
        self.map_array = pygame.surfarray.array3d(self.map_data)  # Load the map as a 3D array (RGB)
        self.drone_history_locations: List[tuple[int, int]] = []

    def load_map(self):
        """
               Load the map image from the file and scale it to a standard size.

               Returns:
               - Surface: Pygame Surface object containing the loaded and scaled map image.

               Notes:
               - Uses Pygame's image loading and transformation functions.
               - Scales the map image to a fixed size of 800x600 pixels.
               """
        # Load map from file and convert to a suitable format
        map_data = pygame.image.load(self.map_file)
        map_data = pygame.transform.scale(map_data, (800, 600))  # Scale to desired size
        return map_data

    def get_distance(self, direction, position):
        """
               Calculate the distance in a specified direction until an obstacle is encountered.

               Args:
               - direction (int): Direction in degrees (0, 90, 180, 270) representing front, right, back, left.
               - position (tuple[int, int]): Starting position coordinates (x, y).

               Returns:
               - int: Distance in pixels until an obstacle is encountered or max distance (300 pixels) is reached.

               Notes:
               - Uses the map_array to determine obstacle positions.
               - Stops calculation when an obstacle (black pixel) is encountered or max distance is reached (300 pixels).
               """
        x, y = position
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

            if self.map_array[int(x)][int(y)][0] == 0 and self.map_array[int(x)][int(y)][1] == 0 and \
                    self.map_array[int(x)][int(y)][2] == 0:
                break

        return distance

    def mark_sensor_area(self, position, distance, direction):
        """
               Mark the area in the map that the sensor can detect based on distance and direction.

               Args:
               - position (tuple[int, int]): Starting position coordinates (x, y).
               - distance (int): Distance in pixels that the sensor can detect.
               - direction (int): Direction in degrees (0, 90, 180, 270) representing front, right, back, left.

               Notes:
               - Marks the area in yellow color on the map_array.
               - Only marks areas that are not obstacles (non-black pixels).
               """
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
                    self.map_array[int(x)][int(y)] = [255, 128, 0]  # Color in yellow

    def display_map(self, screen, drone_position, graph, drone_radius):
        """
                Display the map, graph, and drone on the Pygame screen.

                Args:
                - screen (Surface): Pygame Surface object representing the screen.
                - drone_position (tuple[int, int]): Current position of the drone (x, y).
                - graph (Graph): NetworkX Graph object representing the graph to display.
                - drone_radius (int): Radius of the drone circle to draw.

                Notes:
                - Draws the map, edges and nodes of the graph, historical drone locations, and current drone position on the screen.
                """
        # Create a surface from the updated map_array
        updated_map_surface = pygame.surfarray.make_surface(self.map_array)
        screen.blit(updated_map_surface, (0, 0))

        # Draw the graph
        for edge in graph.edges:
            start_pos = graph.nodes[edge[0]]['position']
            end_pos = graph.nodes[edge[1]]['position']
            pygame.draw.line(screen, (255, 0, 0), start_pos, end_pos, 2)

        for node in graph.nodes:
            pos = graph.nodes[node]['position']
            pygame.draw.circle(screen, (0, 0, 255), pos, 3)

        self.drone_history_locations.append(drone_position)

        for location in self.drone_history_locations:
            pygame.draw.circle(screen, (0, 213, 255), location, 3)

        pygame.draw.circle(screen, (0, 255, 0), drone_position, drone_radius)  # Draw drone with radius 5

    def get_distance_between_points(self, point, second_point) -> float:
        return math.sqrt((point[0] - second_point[0]) ** 2 + (point[1] - second_point[1]) ** 2)

    def is_point_in_valid_spot(self, point: tuple[int, int], drone_radius):
        """
               Check if a point is within a valid spot on the map for the drone to move.

               Args:
               - point (tuple[int, int]): Point coordinates (x, y) to check.
               - drone_radius (int): Radius of the drone for clearance.

               Returns:
               - bool: True if the point is in a valid spot, False otherwise.

               Notes:
               - Checks if the point and the area around it within the drone's radius are not obstacles.
               - Assumes the map_array contains obstacle information.
               """
        x, y = point
        radius = drone_radius // 2 + 1
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if (self.get_distance_between_points(point, (int(x + i), int(y + j))) <= radius):
                    if (int(x + i) >= len(self.map_array) or int(y + j) >= len(self.map_array[int(x + i)])):
                        return False
                    if self.map_array[int(x + i)][int(y + j)][0] == 0 and self.map_array[int(x + i)][int(y + j)][
                        1] == 0 and self.map_array[int(x + i)][int(y + j)][2] == 0:
                        return False
        return True