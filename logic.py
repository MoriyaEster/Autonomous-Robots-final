import time
from PIL import Image
from concurrent.futures import ThreadPoolExecutor
import pygame


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
        self.is_in_return = False
        self.directions = [(1 * velocity, 0), ((-1) * velocity, 0), (0, (-1) * velocity), (0, 1 * velocity)]
        self.directions_coloring = [(1, 0), ((-1), 0), (0, (-1)), (0, 1)]
        self.current_readings = []
        self.direction = "up"
        self.velocity = velocity
        self.radius = radius
        self.matrix = matrix
        self.window_width = window_width
        self.window_height = window_height
        self.executor = ThreadPoolExecutor()
        self.is_going_up = True
        self.is_going_down = True
        self.is_going_left = True
        self.is_going_right = True

    def calculate_directions_step(self, velocity):
        self.directions = [(1 * velocity, 0), ((-1) * velocity, 0), (0, (-1) * velocity), (0, 1 * velocity)]

    def check_if_in_track_if_so_color(self, x, y):
        if self.is_inside_track(x, y):
            self.history.append((x, y))

    def append_radius(self):
        if self.is_in_return:
            return
        directions = []
        current_readings = self.get_sensor_readings(self.directions_coloring)
        for index in range(current_readings[0]):
            directions.append((self.x + index, self.y))
        for index in range(current_readings[1]):
            directions.append((self.x - index, self.y))
        for index in range(current_readings[2]):
            directions.append((self.x, self.y - index))
        for index in range(current_readings[3]):
            directions.append((self.x, self.y + index))

        self.executor.map(lambda d: self.check_if_in_track_if_so_color(d[0], d[1]), directions)

    def move(self, dx, dy):
        new_x = int(self.x + dx)
        new_y = int(self.y + dy)
        if self.is_inside_track(new_x, new_y):
            self.history.append((self.x, self.y))
            self.append_radius()
            self.x = new_x
            self.y = new_y

    def is_inside_track(self, x, y):
        result = (self.matrix[y][x] == 1)
        return result

    def get_sensor_readings(self, directions):
        readings = list(self.executor.map(lambda d: self.calculate_sensor_range(d[0], d[1]), directions))
        self.current_readings = readings
        return readings

    def calculate_sensor_range(self, dx, dy):
        distance = 0
        while distance < max(self.window_width, self.window_height):
            x = self.x + dx * distance
            y = self.y + dy * distance
            if x < 0 or x >= self.window_width or y < 0 or y >= self.window_height or not self.is_inside_track(x, y):
                break
            if distance > self.radius:
                return distance
            distance += 1
        return distance

    def draw_history(self, screen, color):
        for point in self.history:
            pygame.draw.circle(screen, color, point, 2)

    def draw(self, screen, color):
        # Draw the line part of the arrow
        end = (self.x + 1, self.y)
        arrow_length = 20
        # Determine the direction of the arrowhead
        direction = self.direction
        if direction == "up":
            arrow_dir = pygame.math.Vector2(0, -1)
        elif direction == "down":
            arrow_dir = pygame.math.Vector2(0, 1)
        elif direction == "left":
            arrow_dir = pygame.math.Vector2(-1, 0)
        elif direction == "right":
            arrow_dir = pygame.math.Vector2(1, 0)
        else:
            raise ValueError("Invalid direction. Use 'up', 'down', 'left', or 'right'.")

        # Normalize the direction
        arrow_dir = arrow_dir.normalize()

        # Calculate the end points of the arrowhead
        left_end = end + arrow_dir.rotate(150) * arrow_length
        right_end = end + arrow_dir.rotate(-150) * arrow_length

        # Draw the arrowhead
        pygame.draw.polygon(screen, color, [end, left_end, right_end])
        # pygame.draw.circle(screen, DRONE_COLOR, (self.x, self.y), 10)


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
