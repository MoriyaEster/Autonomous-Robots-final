
### README for Drone Simulator Project

---

## Drone Simulator

The Drone Simulator project is a Python-based simulation environment for autonomous drones. The simulator provides functionalities for drone navigation, obstacle detection, and environment mapping using various sensors.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Features](#features)
3. [Requirements](#requirements)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Project Structure](#project-structure)
7. [Modules Description](#modules-description)
8. [License](#license)

---

## Introduction

This project simulates the behavior of a drone navigating through a predefined map. The drone uses several sensors to detect obstacles, measure distances, and map its environment. The primary objective is to demonstrate autonomous navigation and mapping capabilities.

---

## Features

- **Autonomous Navigation**: The drone can navigate through the map autonomously, avoiding obstacles.
- **Sensor Integration**: Uses multiple sensors like LiDAR, Gyroscope, and Optical Flow for environment perception.
- **Graph-Based Mapping**: Creates a graph-based map of the environment using the drone's sensor data.
- **Visualization**: Provides a visual representation of the drone's path and environment using Pygame.

---

## Requirements

- Python 3.7 or higher
- Pygame
- NetworkX

---

## Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/MoriyaEster/Autonomous-Robots-final.git
    cd Autonomous-Robots-final
    ```

2. **Install the required packages**:
    ```bash
    pip install -r requirements.txt
    ```

---

## Usage

To run the drone simulation, execute the `simulation.py` script:

```bash
python simulation.py
```

This will start the simulation with a predefined map and display the drone's navigation in a Pygame window.

---

## Project Structure

```
drone_simulator/
│
├── core/
│   ├── drone.py         # Drone class definition and behavior
│   └── map.py           # Map class for handling map loading and processing
│
├── sensors/
│   ├── lidar.py         # LiDAR sensor simulation
│   ├── gyroscope.py     # Gyroscope sensor simulation
│   ├── optical_flow.py  # Optical Flow sensor simulation
│   └── speed.py         # Speed sensor simulation
│
├── ui/
│   └── interface.py     # Pygame UI for drone simulation
│
├── utils/
│   └── helper.py        # Helper functions for the project
│
├── simulation.py        # Main script to run the simulation
└── requirements.txt     # List of dependencies
```

---

## Modules Description

### 1. core/drone.py

Defines the `Drone` class, which simulates a drone's behavior, including movement, sensing, and mapping. The drone uses various sensors to navigate and map its environment.

### 2. core/map.py

Defines the `Map` class, which handles loading and processing of the map file. It includes functionalities for detecting distances and updating the map based on sensor data.

### 3. sensors/

- **lidar.py**: Simulates a LiDAR sensor for measuring distances to obstacles.
- **gyroscope.py**: Simulates a gyroscope for detecting the drone's orientation.
- **optical_flow.py**: Simulates an optical flow sensor for measuring the drone's movement.
- **speed.py**: Simulates a speed sensor for measuring the drone's speed.

### 4. ui/interface.py

Defines the `DroneSimulatorUI` class, which uses Pygame to provide a visual interface for the drone simulation. It displays the map, the drone's path, and sensor readings.

### 5. utils/helper.py

Provides helper functions for noise addition, point rotation, map loading, and drone drawing.

### 6. simulation.py

The main script that initializes the simulation. It sets up the map, the drone, and the UI, and starts the simulation loop.

---

## License

This project is licensed under the MIT License. See the LICENSE file for details.

---

## Contributing

Contributions are welcome! Please open an issue or submit a pull request with any improvements or bug fixes.

---

## Acknowledgements

Special thanks to all the contributors and the open-source community for their invaluable support and resources.

---

This README provides an overview of the Drone Simulator project, detailing its features, structure, and usage. For more detailed information, refer to the source code and comments within each module.

---
