import random
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

        self.position = [125, 125]  # Initial position in the middle of the screen
        self.rotation = 0
        self.map = map

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

        # Ensure new position is valid (not colliding with obstacles)
        if not self.is_collision(new_position):
            self.position = new_position
        else:
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
        return data

    def decide_next_move(self, sensor_data):
        front_distance = sensor_data['lidar_front']
        left_distance = sensor_data['lidar_left']
        right_distance = sensor_data['lidar_right']

        # Adding buffer distance to account for drone's radius
        buffer_distance = self.radius + 10

        if front_distance > buffer_distance:
            if not self.move("forward"):  # Try to move forward
                if left_distance > right_distance:
                    self.move("left")  # If collision, turn left if more space on the left
                else:
                    self.move("right")  # Else, turn right
        elif left_distance > right_distance:
            self.move("left")
        else:
            self.move("right")
