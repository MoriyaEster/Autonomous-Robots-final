import math
import random
from typing import List, Union
import networkx as nx
from networkx import Graph
from drone_simulator.sensors.lidar import Lidar
from drone_simulator.sensors.gyroscope import Gyroscope
from drone_simulator.sensors.optical_flow import OpticalFlow
from drone_simulator.sensors.speed import Speed
from drone_simulator.core.map import Map

class Drone:
    def __init__(self, map: Map):
        self.radius = 10  # Drone radius
        self.lidar_front: Lidar = Lidar()
        self.lidar_back: Lidar = Lidar()
        self.lidar_left: Lidar = Lidar()
        self.lidar_right: Lidar = Lidar()
        self.gyroscope: Gyroscope = Gyroscope()
        self.optical_flow: OpticalFlow = OpticalFlow()
        self.speed_sensor: Speed = Speed()
        self.min_distance_between_points: int = 10
        self.current_path = []

        # Initial position in an open area
        self.position: List[float] = [130.0, 130.0]
        self.rotation: int = 0
        self.map: Map = map

        # Graph to represent the map
        self.graph: Graph = nx.Graph()
        self.current_node: Union[tuple[int, int], None] = None
        self.previous_node: Union[tuple[int, int], None] = None

        # Track visits to nodes
        self.node_visit_count: dict[Union[tuple[int, int], None], int] = {}
        self.stuck_counter: int = 0

    def move(self, direction):
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
        if new_position != self.position and self.map.is_point_in_valid_spot((int(new_position[0]), int(new_position[1]))):
            self.position = new_position
            self.stuck_counter = 0  # Reset stuck counter
            # self.add_node_if_significant_change()
            # self.add_nodes_in_open_space()  # Add nodes in open space
            # print(f"Moved {direction} to {self.position}")
        else:
            # print(f"Collision detected when moving {direction} from {self.position}")
            self.stuck_counter += 1  # Increment stuck counter
            return False  # Indicate that the move resulted in a collision

        # Update sensors' positions
        self.optical_flow.set_position(self.position)
        self.gyroscope.set_rotation(self.rotation)
        return True  # Indicate successful move



    def sense(self, environment: Map) -> dict[str, Union[int, float]]:
        data: dict[str, Union[int, float]] = {
            'lidar_front': self.lidar_front.measure_distance(0, environment, self.position),
            'lidar_backward': self.lidar_back.measure_distance(180, environment, self.position),
            'lidar_left': self.lidar_left.measure_distance(270, environment, self.position),
            'lidar_right': self.lidar_right.measure_distance(90, environment, self.position),
            # 'rotation': self.gyroscope.get_rotation(),
            # 'position': self.optical_flow.get_position(),
            # 'speed': self.speed_sensor.get_speed(),
        }

        # Mark the sensor absorbed areas on the map
        # environment.mark_sensor_area(self.position, data['lidar_front'], self.rotation)
        # environment.mark_sensor_area(self.position, data['lidar_backward'], (self.rotation + 180) % 360)
        # environment.mark_sensor_area(self.position, data['lidar_left'], (self.rotation - 90) % 360)
        # environment.mark_sensor_area(self.position, data['lidar_right'], (self.rotation + 90) % 360)

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
            distance = self.get_distance_between_points(new_node_id, node)
            if distance < self.min_distance_between_points:  # Ensure nodes are at least 10 cm apart
                return False
        return True

    def add_nodes_in_open_space(self):
        directions: dict[int, tuple[int, int]] = {
            0: (0, -1),  # Front (up)
            90: (1, 0),  # Right
            180: (0, 1),  # Backward (down)
            270: (-1, 0)  # Left
        }

        for direction, (dx, dy) in directions.items():
            x, y = self.position
            for i in range(100):  # 1 meter = 100 cm
                x += dx
                y += dy

                if self.map.is_point_in_valid_spot((int(x), int(y))):
                    break

            node_id = (int(x), int(y))
            if node_id not in self.graph and self.is_node_far_enough(node_id):
                self.graph.add_node(node_id, position=[x, y], visits=0)
                if self.current_node is not None:
                    self.graph.add_edge(self.current_node, node_id)
                self.previous_node = self.current_node
                self.current_node = node_id
                self.node_visit_count[node_id] = 0  # Initialize visit count


    def generate_nodes_based_on_sensor_data(self, sensor_data: dict[str, Union[int, float]]):
        
        # add my current position
        current_position_node = (int(self.position.copy()[0]), int(self.position.copy()[1]))
        if current_position_node not in self.graph and self.map.is_point_in_valid_spot(current_position_node):
            self.graph.add_node(current_position_node, position=current_position_node, visited=True)
        
        front_distance = sensor_data['lidar_front']
        back_distance = sensor_data['lidar_backward']
        left_distance = sensor_data['lidar_left']
        right_distance = sensor_data['lidar_right']
        # print(sensor_data)
        
        for step in range(self.min_distance_between_points, int(front_distance), self.min_distance_between_points):
            new_y_position = self.position[1] - step
            node_id = (int(self.position[0]), int(new_y_position))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)
        
        for step in range(self.min_distance_between_points, int(back_distance), self.min_distance_between_points):
            new_y_position = self.position[1] + step
            node_id = (int(self.position[0]), int(new_y_position))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)
        
        for step in range(self.min_distance_between_points, int(left_distance), self.min_distance_between_points):
            new_x_position = self.position[0] - step
            node_id = (int(new_x_position), int(self.position[1]))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)
                    
        for step in range(self.min_distance_between_points, int(right_distance), self.min_distance_between_points):
            new_x_position = self.position[0] + step
            node_id = (int(new_x_position), int(self.position[1]))
            if node_id not in self.graph and self.map.is_point_in_valid_spot(node_id):
                if self.is_node_far_enough(node_id):
                    self.graph.add_node(node_id, position=node_id, visited=False)
                    self.generate_edges_for_graph_by_node(node_id)
                    

    def get_non_neighbors_of_a_node(self, node):
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
        
        return list(filter(lambda x: self.get_distance_between_points(node, x) <= self.min_distance_between_points * 2, list(non_neighbors)))

    def generate_edges_for_graph_by_node(self, node):
        non_neighbors = self.get_non_neighbors_of_a_node(node)
        for non_neighbor in non_neighbors:
            self.graph.add_edge(node, non_neighbor)
            
                                            
    def find_closest_unvisited_node(self) -> Union[tuple[int, int], None]:
        min_distance = float('inf')
        closest_node = None

        # Iterate over all nodes and their attributes
        for node_id, node_data in self.graph.nodes(data=True):
            if not node_data['visited']:
                node_position = node_id  # Assuming node_id is a tuple (x, y)
                distance = self.get_distance_between_points(self.position, node_position)
                if distance < min_distance:
                    min_distance = distance
                    closest_node = node_id

        return closest_node
    
    def move_one_step_towards(self, current_position, desired_position):
        current_x, current_y = current_position
        desired_x, desired_y = desired_position
        
        # Calculate the differences in x and y directions
        dx = desired_x - current_x
        dy = desired_y - current_y
        
        speed = self.speed_sensor.get_speed()
        new_x = current_x + (speed if dx > 0 else -speed) 
        new_y = current_y + (speed if dy > 0 else -speed)
        
        return new_x, new_y
    
    def get_distance_between_points(self, point, second_point) -> float:
        return math.sqrt((point[0] - second_point[0]) ** 2 + (point[1] - second_point[1]) ** 2)
        
    def decide_next_move(self, sensor_data: dict[str, Union[int, float]]):
        self.generate_nodes_based_on_sensor_data(sensor_data)
        next_node = self.find_closest_unvisited_node()
        path = None
        if(self.current_path):
            path = self.current_path
        else:
            try:
                path = nx.shortest_path(self.graph, source=(int(self.position[0]), int(self.position[1])), target=next_node)[1:]
            except Exception as e:
                pass
        if(path):
            next_node = path.pop(0)
        if(next_node):
            if(self.get_distance_between_points((int(self.position[0]), int(self.position[1])), next_node) > self.min_distance_between_points * 2):
                print("problem!!!!!")
                
            if(next_node[0] == int(self.position[0]) and next_node[1] == int(self.position[1])):
                self.graph.nodes[next_node]['visited'] = True
            else:
                self.position = [next_node[0], next_node[1]]
                self.graph.nodes[next_node]['visited'] = True
        else:
            print("DONE")
        if(path):
            self.current_path = path
            
                    
    # def decide_next_move(self, sensor_data: dict[str, Union[int, float]]):
    #     self.generate_nodes_based_on_sensor_data(sensor_data)
    #     next_node = self.find_closest_unvisited_node()
    #     path = None
    #     try:
    #         path = nx.shortest_path(self.graph, source=(int(self.position[0]), int(self.position[1])), target=next_node)[1:]
    #     except Exception as e:
    #         pass
    #     if(path):
    #         next_node = path.pop()
    #     if(next_node):
    #         if(next_node[0] == int(self.position[0]) and next_node[1] == int(self.position[1])):
    #             self.graph.nodes[next_node]['visited'] = True
    #         elif(self.get_distance_between_points(self.position, next_node) <= self.speed_sensor.get_speed()):
    #             self.position = [next_node[0], next_node[1]]
    #             self.graph.nodes[next_node]['visited'] = True
    #         else:
    #             new_position = self.move_one_step_towards(self.position, [next_node[0], next_node[1]])
    #             if(self.map.is_point_in_valid_spot(new_position)):
    #                 self.position = [new_position[0], new_position[1]]
    #             else:
    #                 print("im stuck here")
    #                 self.attempt_alternate_moves()
    #     else:
    #         self.attempt_alternate_moves()
    
    # def decide_next_move(self, sensor_data: dict[str, Union[int, float]]):
    #     if self.node_visit_count.get(self.current_node, 0) > 3:
    #         self.attempt_alternate_moves()
    #     else:
    #         front_distance = sensor_data['lidar_front']
    #         left_distance = sensor_data['lidar_left']
    #         right_distance = sensor_data['lidar_right']
    #         buffer_distance = self.radius + 10

    #         if front_distance > buffer_distance:
    #             if not self.move("forward"):  # Try to move forward
    #                 self.change_direction(left_distance, right_distance)
    #         else:
    #             self.change_direction(left_distance, right_distance)

    #     # Fallback if the drone is stuck for too long
    #     if self.stuck_counter > 5:
    #         print("Drone is stuck. Trying to turn around.")
    #         self.stuck_counter = 0  # Reset counter after turning around
    #         self.attempt_alternate_moves()

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
