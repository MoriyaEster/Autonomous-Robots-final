from collections import deque
import math
import random
from typing import List, Tuple, Union
import networkx as nx
from networkx import Graph
import pygame
from sensors.lidar import Lidar
from sensors.gyroscope import Gyroscope
from sensors.optical_flow import OpticalFlow
from sensors.speed import Speed
from core.map import Map
from sensors.battery import Battery


class Drone:
    def __init__(self, map: Map, drone_radius: int, battery_duration_in_seconds: int, speed_in_pixels_per_move: int):
        self.starting_position: List[float] = [130.0, 130.0]
        self.radius = drone_radius  # Drone radius
        self.lidar_front: Lidar = Lidar()
        self.lidar_back: Lidar = Lidar()
        self.lidar_left: Lidar = Lidar()
        self.lidar_right: Lidar = Lidar()
        self.gyroscope: Gyroscope = Gyroscope()
        self.optical_flow: OpticalFlow = OpticalFlow()
        self.speed_sensor: Speed = Speed(speed_in_pixels_per_move)
        self.battery: Battery = Battery(battery_duration_in_seconds)
        self.returning_home = False
        self.home_path = []
        self.min_distance_between_points: int = max(self.radius // 2 + 1, 5)
        self.current_path = []
        self.home_point = self.find_valid_track_point(self.starting_position, map, self.radius)
        self.home_node = tuple(self.home_point)

        # Initial position in an open area
        self.map: Map = map
        self.position: List[float] = self.home_point
        self.home_point = self.position
        self.rotation: int = 0

        # Graph to represent the map
        self.graph: Graph = nx.Graph()
        self.current_node: Union[tuple[int, int], None] = None
        self.previous_node: Union[tuple[int, int], None] = None

        # Track visits to nodes
        self.node_visit_count: dict[Union[tuple[int, int], None], int] = {}
        self.stuck_counter: int = 0

    def find_valid_track_point(self, starting_position: List[float], map: Map, radius) -> List[float]:
        """
           Finds a valid track point within a given radius on the map.

           This function checks if the starting position is within a valid spot on the map. If not, it generates new random points
           within the map boundaries until a valid point is found.

       """
        if map.is_point_in_valid_spot((int(starting_position[0]), int(starting_position[1])), radius):
            return starting_position
        invalid_point = True

        new_point = [0.0, 0.0]

        while invalid_point:
            center_x, center_y = float(random.randint(20, 790)), float(random.randint(20, 585))
            new_point = [center_x, center_y]
            if map.is_point_in_valid_spot((int(new_point[0]), int(new_point[1])), radius):
                invalid_point = False

        return new_point

    def move(self, direction):
        """
        Move the drone in the specified direction.

        Parameters:
        direction (str): The direction to move the drone. Valid values are "forward", "backward", "left", and "right".

        Returns:
        bool: True if the move is successful, False if the move results in a collision.
        """
        speed = self.speed_sensor.get_speed()
        noise = random.uniform(-0.02, 0.02)  # Adjust noise to make it more realistic
        new_position = self.position.copy()
        if direction == "forward":
            self.rotation = 0
            new_position[1] -= speed + noise
        elif direction == "backward":
            self.rotation = 180
            new_position[1] += speed + noise
        elif direction == "left":
            self.rotation = 270
            new_position[0] -= speed + noise
        elif direction == "right":
            self.rotation = 90
            new_position[0] += speed + noise

        # Ensure new position is valid (not colliding with obstacles)
        if new_position != self.position and self.map.is_point_in_valid_spot(
                (int(new_position[0]), int(new_position[1])), self.radius):
            self.position = new_position
            self.stuck_counter = 0  # Reset stuck counter
        else:
            self.stuck_counter += 1  # Increment stuck counter
            return False  # Indicate that the move resulted in a collision

        # Update sensors' positions
        self.optical_flow.set_position(self.position)
        self.gyroscope.set_rotation(self.rotation)
        return True  # Indicate successful move

    def sense(self, environment: Map) -> dict[str, Union[int, float]]:
        """
        Update sensor readings based on the drone's current position and environment.

        Args:
        - environment (object): The environment or map instance the drone operates in.

        This method simulates the drone's sensors (e.g., camera, lidar) updating based on its current position
        and the environment. It updates various sensor readings that can be queried later for navigation and
        obstacle avoidance.

        Example:
        >>> from core.map import Map
        >>> environment = Map('p11.png')
        >>> drone = Drone(environment, [50, 50], drone_radius=7)
        >>> drone.optical_flow.get_position() == drone.position
        False

        Notes:
        - This example assumes the drone has been initialized with a map instance and its initial position.
        - The `optical_flow` and `gyroscope` attributes are placeholders for actual sensor objects.
        """
        data: dict[str, Union[int, float]] = {
            'lidar_front': self.lidar_front.measure_distance(0, environment, self.position),
            'lidar_backward': self.lidar_back.measure_distance(180, environment, self.position),
            'lidar_left': self.lidar_left.measure_distance(270, environment, self.position),
            'lidar_right': self.lidar_right.measure_distance(90, environment, self.position),
        }

        environment.mark_sensor_area(self.position, data['lidar_front'], 0)
        environment.mark_sensor_area(self.position, data['lidar_backward'], 180)
        environment.mark_sensor_area(self.position, data['lidar_left'], 270)
        environment.mark_sensor_area(self.position, data['lidar_right'], 90)

        return data

    def is_node_far_enough(self, new_node_id):
        """
        Check if two nodes are far enough apart based on the drone's radius.
        """
        for node in self.graph.nodes:
            distance = self.map.get_distance_between_points(new_node_id, node)
            if distance < self.min_distance_between_points:  # Ensure nodes are at least 10 cm apart
                return False
        return True

    def generate_nodes_based_on_sensor_data(self, sensor_data: dict[str, Union[int, float]]):
        """
        Generate nodes based on sensor data that meets a certain threshold.


        Returns:
        - list: List of nodes generated based on sensor data that meets or exceeds the threshold.

        Notes:
        - Assumes sensor_data is a dictionary where keys are directions ('left', 'right', 'front', 'back').
        - Nodes are generated based on distances provided in sensor_data.
        - Only nodes meeting or exceeding the threshold distance are included in the output list.
        """
        # Add current position
        current_position_node = (int(self.position.copy()[0]), int(self.position.copy()[1]))
        if current_position_node not in self.graph and self.map.is_point_in_valid_spot(current_position_node,
                                                                                       self.radius):
            self.graph.add_node(current_position_node, position=current_position_node, visited=True)

        front_distance = sensor_data['lidar_front']
        back_distance = sensor_data['lidar_backward']
        left_distance = sensor_data['lidar_left']
        right_distance = sensor_data['lidar_right']

        for step in range(0, int(front_distance), self.min_distance_between_points):
            new_y_position = self.position[1] - step
            node_id = (int(self.position[0]), int(new_y_position))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id, self.radius):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)

        for step in range(0, int(back_distance), self.min_distance_between_points):
            new_y_position = self.position[1] + step
            node_id = (int(self.position[0]), int(new_y_position))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id, self.radius):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)

        for step in range(0, int(left_distance), self.min_distance_between_points):
            new_x_position = self.position[0] - step
            node_id = (int(new_x_position), int(self.position[1]))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id, self.radius):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)

        for step in range(0, int(right_distance), self.min_distance_between_points):
            new_x_position = self.position[0] + step
            node_id = (int(new_x_position), int(self.position[1]))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id, self.radius):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)

    def get_non_neighbors_of_a_node_within_min_distance(self, node):
        """
        Get nodes that are not neighbors of a given node and are within a minimum distance.

        Args:
        - node (tuple): Coordinates of the node for which non-neighbors are to be found.
        - nodes (list of tuples): List of nodes where each node is represented as a tuple of coordinates.
        - min_distance (float): Minimum distance threshold within which non-neighboring nodes should lie.

        Returns:
        - list: List of nodes that are not neighbors of the given node and are within the minimum distance.

        Notes:
        - Assumes nodes are represented as tuples of coordinates (x, y).
        - Non-neighbors are determined based on Euclidean distance.
        - Only nodes within the specified minimum distance from the given node are included in the output list.
        """
        if node not in self.graph:
            raise ValueError("Node not found in the graph")

        # Get all nodes in the graph
        all_nodes = set(self.graph.nodes)

        # Get all neighbors of the specified node
        neighbors = set(self.graph.neighbors(node))

        # Add the node itself to the neighbors set to exclude it from the result
        neighbors.add(node)

        # Nodes without an edge to the specified node
        non_neighbors = all_nodes - neighbors

        return list(filter(lambda x: self.map.get_distance_between_points(node, x) <= self.min_distance_between_points,
                           list(non_neighbors)))

    def generate_edges_for_graph_by_node(self, node):
        """
        Generate edges for a graph based on a central node within a maximum distance.

        Args:
        - nodes (list of tuples): List of nodes where each node is represented as a tuple of coordinates.
        - node (tuple): Coordinates of the central node for which edges are to be generated.
        - max_distance (float): Maximum distance threshold for considering nodes as neighbors.

        Returns:
        - list of tuples: List of edges represented as tuples (node1, node2) within the maximum distance.

        Notes:
        - Assumes nodes are represented as tuples of coordinates (x, y).
        - Edges are generated based on Euclidean distance between nodes.
        - Only nodes within the specified maximum distance from the central node are included in the edges.
        """
        non_neighbors = self.get_non_neighbors_of_a_node_within_min_distance(node)
        for non_neighbor in non_neighbors:
            self.graph.add_edge(node, non_neighbor)

    def find_farthest_unvisited_node(self) -> Union[Tuple[int, int], None]:
        """
        Find the unvisited node that is the farthest from any visited node using BFS.

        Returns:
        - tuple or None: Coordinates of the unvisited node that is farthest from any visited node,
        or None if all nodes are visited.

        Notes:
        - Assumes nodes are represented as tuples of coordinates (x, y).
        - Uses BFS to find the closest visited node for each unvisited node.
        - Returns None if all nodes in the graph are visited.
        """
        # Step 1: Get all visited and unvisited nodes
        visited_nodes = [node for node, data in self.graph.nodes(data=True) if data["visited"]]
        unvisited_nodes = [node for node, data in self.graph.nodes(data=True) if not data["visited"]]

        if not unvisited_nodes:
            return None

        farthest_node = None
        max_distance = -1

        # Step 2: For each unvisited node, perform BFS to find the closest visited node
        for unvisited in unvisited_nodes:
            queue = deque([(unvisited, 0)])  # (node, distance)
            visited = set()

            while queue:
                current_node, distance = queue.popleft()
                current_node = (int(current_node[0]), int(current_node[1]))

                if current_node in visited:
                    continue

                # Mark the current node as visited
                visited.add(current_node)

                # Check if the current node is one of the visited nodes
                if current_node in visited_nodes:
                    if distance > max_distance:
                        max_distance = distance
                        farthest_node = unvisited
                    break

                # Add all neighbors to the queue
                neighbors = self.graph.neighbors(current_node)
                for neighbor in neighbors:
                    if neighbor not in visited:
                        queue.append((neighbor, distance + 1))

        return farthest_node

    def move_one_step_towards(self, current_position, desired_position):
        """
        Move one step closer from the start position towards the goal position.

        Args:
        - start (tuple): Starting position coordinates (x, y).
        - goal (tuple): Goal position coordinates (x, y).
        - step_size (float): Step size for movement towards the goal.

        Returns:
        - tuple: New position after moving one step closer towards the goal.

        Notes:
        - Assumes positions are represented as tuples of coordinates (x, y).
        - Moves towards the goal by `step_size` units.
        - The step size determines how far to move towards the goal in each step.
        """
        current_x, current_y = current_position
        desired_x, desired_y = desired_position

        # Calculate the differences in x and y directions
        dx = desired_x - current_x
        dy = desired_y - current_y

        speed = self.speed_sensor.get_speed()
        new_x = current_x + (speed if dx > 0 else -speed)
        new_y = current_y + (speed if dy > 0 else -speed)

        return new_x, new_y

    def decide_next_move(self, sensor_data: dict[str, Union[int, float]]):
        """
        Decide the next move from the current position towards the target position, avoiding obstacles if possible.

        """

        # if the battery died the drone stuck in the place
        if self.battery.is_dead():
            return

        # If the battery is low or the remaining power is just sufficient to get home, the drone returns home.
        if self.battery.battery_low() or ((self.battery.power - self.calculate_path_to_home()) - 5 <= 0):
            if not self.returning_home:
                self.returning_home = True
                self.current_path = nx.shortest_path(self.graph, source=(int(self.position[0]), int(self.position[1])),
                                                     target=self.home_node)[1:]
            if not self.current_path:
                while self.battery.power < self.battery.duration:
                    self.battery.charging()
                self.returning_home = False

        self.generate_nodes_based_on_sensor_data(sensor_data)
        path = None
        if self.current_path:
            path = self.current_path
        else:
            next_node = self.find_farthest_unvisited_node()
            try:
                path = nx.shortest_path(self.graph, source=(int(self.position[0]), int(self.position[1])),
                                        target=next_node)[1:]
            except Exception as e:
                pass
        if path:
            next_node = path.pop(0)
        if next_node:
            if self.map.get_distance_between_points((int(self.position[0]), int(self.position[1])),
                                                    next_node) > self.speed_sensor.get_speed():
                new_position = self.move_one_step_towards(self.position, [next_node[0], next_node[1]])
                if self.map.is_point_in_valid_spot(new_position, self.radius):
                    self.position = [new_position[0], new_position[1]]
                else:
                    self.attempt_alternate_moves()

            if next_node[0] == int(self.position[0]) and next_node[1] == int(self.position[1]):
                self.graph.nodes[next_node]['visited'] = True
            else:
                self.position = [next_node[0], next_node[1]]
                self.graph.nodes[next_node]['visited'] = True
        if path:
            self.current_path = path

        self.battery.power -= 1

    def calculate_path_to_home(self):
        """
           Calculates the length of the shortest path from the current position to the home node and saves it in the home_path attribute.

           Returns:
               int: Length of the path to the home node, or 0 if the path cannot be calculated.
           """
        if not self.graph.has_node(self.home_node):
            return 0

        if not self.graph.has_node((int(self.position[0]), int(self.position[1]))):
            return

        try:
            home_path = nx.shortest_path(self.graph, source=(int(self.position[0]), int(self.position[1])),
                                         target=self.home_node)
        except Exception as e:
            print(e)
            return 0

        return len(home_path)

    def attempt_alternate_moves(self):
        """
           Attempts to move the drone in alternate directions: backward, left, right, and forward.
        """
        if not self.move("backward"):
            if not self.move("left"):
                if not self.move("right"):
                    self.move("forward")