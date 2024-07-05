import random
import networkx as nx
from drone_simulator.sensors.lidar import Lidar
from drone_simulator.sensors.gyroscope import Gyroscope
from drone_simulator.sensors.optical_flow import OpticalFlow
from drone_simulator.sensors.speed import Speed

class Drone:
    def __init__(self, map):
        self.radius = 10  # Drone radius
        self.lidar_front = Lidar()
        self.lidar_left = Lidar()
        self.lidar_right = Lidar()
        self.gyroscope = Gyroscope()
        self.optical_flow = OpticalFlow()
        self.speed_sensor = Speed()

        # Initial position in an open area
        self.position = [130, 130]
        self.rotation = 0
        self.map = map

        # Graph to represent the map
        self.graph = nx.Graph()
        self.current_node = None
        self.previous_node = None

        # Track visits to nodes
        self.node_visit_count = {}
        self.stuck_counter = 0

    def move(self, direction):
        speed = self.speed_sensor.get_speed()
        noise = random.uniform(-0.05, 0.05)  # Adding noise to movement
        new_position = self.position.copy()
        if direction == "forward":
            if self.rotation == 0:
                new_position[1] -= speed + noise
            elif self.rotation == 90:
                new_position[0] += speed + noise
            elif self.rotation == 180:
                new_position[1] += speed + noise
            elif self.rotation == 270:
                new_position[0] -= speed + noise
        elif direction == "left":
            self.rotation = (self.rotation - 90) % 360
        elif direction == "right":
            self.rotation = (self.rotation + 90) % 360
        elif direction == "backward":
            self.rotation = (self.rotation + 180) % 360

        # Ensure new position is valid (not colliding with obstacles)
        if not self.is_collision(new_position):
            self.position = new_position
            self.stuck_counter = 0  # Reset stuck counter
            self.add_node_if_significant_change()
            print(f"Moved {direction} to {self.position}")
        else:
            print(f"Collision detected when moving {direction} from {self.position}")
            self.stuck_counter += 1  # Increment stuck counter
            return False  # Indicate that the move resulted in a collision

        # Update sensors' positions
        self.optical_flow.set_position(self.position)
        self.gyroscope.set_rotation(self.rotation)
        return True  # Indicate successful move

    def is_collision(self, position):
        x, y = position
        radius = self.radius

        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if (i**2 + j**2) <= radius**2:  # Check within the circle of the drone's radius
                    if self.map.map_array[int(x + i)][int(y + j)][0] == 0 and self.map.map_array[int(x + i)][int(y + j)][1] == 0 and self.map.map_array[int(x + i)][int(y + j)][2] == 0:
                        return True
        return False

    def sense(self, environment):
        data = {
            'lidar_front': self.lidar_front.measure_distance(self.rotation, environment, self.position),
            'lidar_left': self.lidar_left.measure_distance((self.rotation - 90) % 360, environment, self.position),
            'lidar_right': self.lidar_right.measure_distance((self.rotation + 90) % 360, environment, self.position),
            'rotation': self.gyroscope.get_rotation(),
            'position': self.optical_flow.get_position(),
            'speed': self.speed_sensor.get_speed(),
        }

        # Mark the sensor absorbed areas on the map
        environment.mark_sensor_area(self.position, data['lidar_front'], self.rotation)
        environment.mark_sensor_area(self.position, data['lidar_left'], (self.rotation - 90) % 360)
        environment.mark_sensor_area(self.position, data['lidar_right'], (self.rotation + 90) % 360)

        return data

    def add_node_if_significant_change(self):
        significant_change_detected = False
        sensor_data = self.sense(self.map)
        for direction in ['lidar_front', 'lidar_left', 'lidar_right']:
            if sensor_data[direction] < self.radius * 3:
                significant_change_detected = True
                break

        if significant_change_detected:
            node_id = (int(self.position[0]), int(self.position[1]))
            if node_id not in self.graph:
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=self.position.copy(), visits=0)
                    if self.current_node is not None:
                        self.graph.add_edge(self.current_node, node_id)
                    self.previous_node = self.current_node
                    self.current_node = node_id
                    self.node_visit_count[node_id] = 0  # Initialize visit count

            # Increment visit count
            if node_id in self.node_visit_count:
                self.node_visit_count[node_id] += 1

    def is_node_far_enough(self, new_node_id):
        for node in self.graph.nodes:
            distance = ((new_node_id[0] - node[0])**2 + (new_node_id[1] - node[1])**2)**0.5
            if distance < 50:  # Ensure nodes are at least 50 cm apart
                return False
        return True

    def decide_next_move(self, sensor_data):
        if self.node_visit_count.get(self.current_node, 0) > 3:
            self.attempt_alternate_moves()
        else:
            front_distance = sensor_data['lidar_front']
            left_distance = sensor_data['lidar_left']
            right_distance = sensor_data['lidar_right']
            buffer_distance = self.radius + 10

            if front_distance > buffer_distance:
                if not self.move("forward"):  # Try to move forward
                    self.change_direction(left_distance, right_distance)
            else:
                self.change_direction(left_distance, right_distance)

        # Fallback if the drone is stuck for too long
        if self.stuck_counter > 5:
            print("Drone is stuck. Trying to turn around.")
            self.stuck_counter = 0  # Reset counter after turning around
            self.attempt_alternate_moves()

    def change_direction(self, left_distance, right_distance):
        if left_distance > right_distance:
            if not self.move("left"):  # Try to move left
                if not self.move("right"):  # If left is blocked, try moving right
                    self.move("backward")  # If both are blocked, try moving backward
        else:
            if not self.move("right"):  # Try to move right
                if not self.move("left"):  # If right is blocked, try moving left
                    self.move("backward")  # If both are blocked, try moving backward

    def attempt_alternate_moves(self):
        if not self.move("backward"):
            if not self.move("left"):
                if not self.move("right"):
                    self.move("forward")
