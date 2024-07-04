import pygame
import networkx as nx
import random
from concurrent.futures import ThreadPoolExecutor

def image_to_matrix(img, threshold=128):
    img = img.point(lambda p: p > threshold and 1)  # Convert to binary using threshold
    matrix = []
    for y in range(img.height):
        row = []
        for x in range(img.width):
            row.append(img.getpixel((x, y)))
        matrix.append(row)
    return matrix

class Drone:
    def __init__(self, x, y, velocity, radius, matrix, window_width, window_height):
        self.x = x
        self.y = y
        self.history = []
        self.sensor_history = set()
        self.path_history = []
        self.is_in_return = False
        self.directions = [(1, 0), (-1, 0), (0, -1), (0, 1)]
        self.current_direction = (1, 0)
        self.direction_index = 0
        self.velocity = velocity
        self.radius = radius
        self.matrix = matrix
        self.window_width = window_width
        self.window_height = window_height
        self.executor = ThreadPoolExecutor()
        self.sensor_range = 3 * 10
        self.node_counter = 0
        self.current_node = None
        self.graph = nx.Graph()
        self.add_node(x, y)

    def add_node(self, x, y):
        node_id = self.node_counter
        self.graph.add_node(node_id, pos=(x, y))
        self.node_counter += 1
        return node_id

    def add_edge(self, node1, node2):
        self.graph.add_edge(node1, node2)

    def move(self):
        new_x = self.x + self.current_direction[0] * self.velocity
        new_y = self.y + self.current_direction[1] * self.velocity

        if self.is_inside_track(new_x, new_y):
            self.history.append((self.x, self.y))
            self.path_history.append((self.x, self.y))
            self.x = new_x
            self.y = new_y
            self.update_sensor_history()

            if self.should_turn_around():
                print(f"Turning around at ({self.x}, {self.y})")
                self.turn_around()

            if self.should_place_node():
                new_node = self.add_node(new_x, new_y)
                if self.current_node is not None:
                    self.add_edge(self.current_node, new_node)
                self.current_node = new_node
        else:
            self.turn_90_degrees()

    def should_place_node(self):
        sensor_readings = self.get_sensor_readings()
        if max(sensor_readings) - min(sensor_readings) > self.sensor_range / 2:
            return True
        return False

    def turn_90_degrees(self):
        self.direction_index = (self.direction_index + 1) % 4
        self.current_direction = self.directions[self.direction_index]

    def turn_around(self):
        least_yellow = float('inf')
        best_direction_index = self.direction_index

        for i in range(4):
            direction_index = (self.direction_index + i) % 4
            direction = self.directions[direction_index]
            yellow_count = self.count_yellow_in_direction(direction)

            if yellow_count < least_yellow:
                least_yellow = yellow_count
                best_direction_index = direction_index

        self.direction_index = best_direction_index
        self.current_direction = self.directions[self.direction_index]

    def count_yellow_in_direction(self, direction):
        dx, dy = direction
        count = 0
        for step in range(1, self.sensor_range + 1):
            check_x = self.x + dx * step
            check_y = self.y + dy * step
            if (check_x, check_y) in self.sensor_history:
                count += 1
            else:
                break
        return count

    def is_inside_track(self, x, y):
        if x < 0 or x >= self.window_width or y < 0 or y >= self.window_height or self.matrix[y][x] == 0:
            return False

        for dx in range(-self.radius, self.radius + 1):
            for dy in range(-self.radius, self.radius + 1):
                distance = (dx ** 2 + dy ** 2) ** 0.5
                if distance <= self.radius:
                    check_x = x + dx
                    check_y = y + dy
                    if check_x < 0 or check_x >= self.window_width or check_y < 0 or check_y >= self.window_height or self.matrix[check_y][check_x] == 0:
                        return False
        return True

    def get_sensor_readings(self):
        readings = []
        for dx, dy in self.directions:
            distance = 0
            for step in range(1, self.sensor_range + 1):
                check_x = self.x + dx * step
                check_y = self.y + dy * step
                if 0 <= check_x < self.window_width and 0 <= check_y < self.window_height and self.matrix[check_y][check_x] == 1:
                    distance = step
                else:
                    break
            readings.append(distance)
        return readings

    def update_sensor_history(self):
        sensor_readings = self.get_sensor_readings()
        for (dx, dy), distance in zip(self.directions, sensor_readings):
            for step in range(1, distance + 1):
                check_x = self.x + dx * step
                check_y = self.y + dy * step
                self.sensor_history.add((check_x, check_y))

    def should_turn_around(self):
        if (self.x, self.y) in self.path_history:
            for dx, dy in self.directions:
                for step in range(1, self.sensor_range + 1):
                    check_x = self.x + dx * step
                    check_y = self.y + dy * step
                    if (check_x, check_y) not in self.sensor_history:
                        print(f"Missing yellow at ({check_x}, {check_y})")
                        return True
            print(f"Turning around at ({self.x}, {self.y}) - All yellow detected")
            return True
        return False

    def draw_sensors(self, screen, color):
        for point in self.sensor_history:
            pygame.draw.circle(screen, color, point, 2)

    def draw_history(self, screen, color):
        for point in self.history:
            pygame.draw.circle(screen, color, point, 2)

    def draw_path(self, screen, color):
        if len(self.path_history) > 1:
            pygame.draw.lines(screen, color, False, self.path_history, 2)

    def draw(self, screen, color):
        pygame.draw.circle(screen, color, (self.x, self.y), 10)
        self.draw_graph(screen)

    def draw_graph(self, screen):
        pos = nx.get_node_attributes(self.graph, 'pos')
        for node, position in pos.items():
            pygame.draw.circle(screen, (0, 0, 255), position, 5)
        for edge in self.graph.edges:
            start_pos = pos[edge[0]]
            end_pos = pos[edge[1]]
            pygame.draw.line(screen, (0, 0, 255), start_pos, end_pos, 1)

def build_track_from_matrix(matrix, cell_size):
    track = []
    for y in range(len(matrix)):
        for x in range(len(matrix[0])):
            if matrix[y][x] == 1:
                rect = pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size)
                track.append(rect)
    return track

def find_closest_track_point(start_x, start_y, track):
    min_distance = float('inf')
    closest_point = (start_x, start_y)

    for segment in track:
        center_x, center_y = segment.center
        distance = ((center_x - start_x) ** 2 + (center_y - start_y) ** 2) ** 0.5
        if distance < min_distance:
            min_distance = distance
            closest_point = (center_x, center_y)

    return closest_point
