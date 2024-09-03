# A-Star-Differential-Drive-Robot-Navigation

## Project Description
This repository contains the implementation of the A* Algorithm for differential drive robot navigation, developed as part of ENPM661 Project 3 Phase 2.

## 1. Pygame 2D Simulation

![Video GIF](https://github.com/suhasnagaraj99/A-Star-Differential-Drive-Robot-Navigation/blob/main/proj3p2part1.gif)

### Assumptions and Map Description
- The robot is assumed to be a mobile robot with some radius.
- The robot clearance is given as input
- 2 values of Wheel RPMs are given as an input to achieve differential drive motion
- Actions Set: If the two RPMs provided by the user are RPM1 and RPM2, then the action space consisting
of 8 possible actions for the A* algorithm is:
  - [0, RPM1]
  - [RPM1, 0]
  - [RPM1, RPM1]
  - [0, RPM2]
  - [RPM2, 0]
  - [RPM2, RPM2]
  - [RPM1, RPM2]
  - [RPM2, RPM1]
  
![alt text](https://github.com/suhasnagaraj99/A-Star-Differential-Drive-Robot-Navigation/blob/main/661p3p2_action_set.png?raw=false)

- The 2 arguments (RPMs of the Left wheel and Right wheel) are converted to the translation and rotation of the robot.
- To check for duplicate nodes, Euclidean distance threshold is 1.0 unit (for x,y).
- Goal threshold: 1.5 units radius
- The map is as given below:
![alt text](https://github.com/suhasnagaraj99/A-Star-Differential-Drive-Robot-Navigation/blob/main/661p3p2_map.png?raw=true)
- The above map represents the space for clearance = 0 mm. For a clearance and robot radius, the obstacles (including the walls) should be bloated by (clearance + robot radius) mm distance on each side.
- The origin is not at the bottom left corner of the map. It is at a distance of 500mm in the x direction and 1000mm in the y direction from the botton left corner.
- The robot considered is Turtlebot3 waffle and its specifications are taken from the official documentation.

### Required Libraries
Before running the code, ensure that the following Python libraries are installed:

- `pygame`
- `numpy`
- `heapq`
- `time`
- `math`
- `sortedcollections`
- `OrderedSet`

You can install them using pip if they are not already installed:

```bash
pip install pygame numpy math sortedcollections ordered-set
```

### Running the Code
Follow these steps to run the code:

#### Run the Python Script:

1. Execute the proj3p2_suhas_swaraj.py file by running the following command in your terminal:

```bash
python3 proj3p2_suhas_swaraj.py
```
2. The script will prompt you to input the 2 RPMs, clearance, coordinates for the initial and goal nodes and the initial orientation.
3. Ensure that the entered coordinates are valid within the environment.
4. After entering the coordinates, the script will display an animation showing the node exploration and the optimal path found by the A* algorithm.

#### Demo Inputs

You can use the following demo inputs to test the code:
- **Robot Description**
  - RPM 1: 10
  - RPM 2: 20
  - clearance: 25
- **Initial Node:**
  - x-coordinate: 0
  - y-coordinate: 0
  - Initial Orientation: 0

- **Goal Node:**
  - x-coordinate: 5250
  - y-coordinate: 0

Simply enter these values when prompted to see the A* algorithm in action.

## 2. ROS2 Humble & Gazebo Turtlebot3 3D Simulation

![Video GIF](https://github.com/suhasnagaraj99/A-Star-Differential-Drive-Robot-Navigation/blob/main/proj3p2part2.gif)

- Part 2 (current) is an expansion of the Part 1 of this project (where the path was simulated in 2d using pygame).
- In addition to generating the path, the robot is made to follow the path in a simulation environment (Gazebo).
- The assumptions and the map description remains unchanged.

### Prerequisites

- ROS2 Humble and Gazebo installed and properly configured on your system.
- Required dependencies installed. (Make sure to install all necessary ROS 2 packages and Python dependencies.)

### Setup Instructions

1. **Copy the Packages**

   - Copy the 2 packages, `turtlebot3_project3` and `astar`, into the src folder of a ROS 2 workspace.

2. **Build the Packages**

   - Open a terminal in the root of your ROS 2 workspace and build the packages using the following command:
     ```bash
     colcon build
     ```

3. **Source the Workspace**

   - After the build is complete, source the workspace by running:
     ```bash
     source install/setup.bash
     ```

4. **Launch TurtleBot3 Waffle**

   - Open a new terminal tab, source the workspace, and launch the TurtleBot3 Waffle with the following command:
     ```bash
     ros2 launch turtlebot3_project3 competition_world.launch.py
     ```

5. **Run the A * Node**

   - In another new terminal tab, source the workspace, and run the A* node with:
     ```bash
     ros2 run astar control --ros-args -p use_sim_time:=true
     ```

6. **Input Goal Coordinates**

   - When prompted, provide the goal x and goal y coordinates in the terminal.

7. **Pathfinding and Movement**

   - The A* node will generate a path and move the TurtleBot3 Waffle towards the specified goal following the computed path.

### Important Notes

- **Sourcing Workspace:** Remember to source the workspace before running the code in any new terminal. This ensures that ROS 2 packages are properly found.
- **TurtleBot3 Model:** There is no need to select the TurtleBot3 model manually from `bashrc` as the launch files are configured to always use the Waffle model.
- **Dependencies:** Ensure that all required dependencies are installed before running the code. Missing dependencies may prevent the system from working correctly.

### Demo Inputs

You can use the following demo inputs to test the code:

- **Goal Node:**
  - x-coordinate: 5250
  - y-coordinate: 0

### Troubleshooting

- If you encounter issues, double-check that all steps have been followed correctly.
- Ensure that all necessary packages and dependencies are installed and up-to-date.

For further assistance, please consult the [ROS 2 documentation](https://docs.ros.org/en/humble/)
