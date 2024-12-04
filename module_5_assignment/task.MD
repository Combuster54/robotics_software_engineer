# Module 5 Assignment: Enhancing Robot Control for Smooth Motion

## How to use this project

### Commands

Clone the repository

```
git clone https://github.com/Combuster54/robotics_software_engineer.git -b asigment_5
```

Go into the repo and change into assignments branch

```
src robotics_software_engineer/
git switch assignments
```

Go back to your workspace and compile 

```
colcon build && source install/setup.bash
```

## Objective

This assignment is focused on enhancing the control of robots to achieve smoother motion and improved efficiency. You will work on improving a camera-based line-following system, introducing a PI controller, and optimizing the TurtleBot3's path to a goal with minimal energy consumption. Additionally, you will add a position controller to a robotic arm URDF model.

## Tasks

### Task 1: Improve Camera-Based Line Following

#### Improvements

    - Using a PI controller shows a better perfomance in reach and keep the robot in the line. Now, the robot is able to follow the line even in curves.
    - There're no more oscilation behavior meanwhile Turtlebot3 is following the line.

### How to Run Task1

1. Launch Simulation Environment

```sh
ros2 launch module_5_assignment tb3_world.launch.py
```

2. Run the line follower with PI Controller executable

```sh
ros2 run module_5_assignment piController 
```

### Task 2: Optimize Goal Selection and Path Planning

- **Select a goal** for the TurtleBot3 based on the criterion of minimal energy consumption:
  - **Energy consumption** should be evaluated based on the distance to the goal.
  - Select the shortest path to the goal and plan the TurtleBot3's movement accordingly.

- **Develop a ROS 2 node** that plans and executes the TurtleBot3’s movement to the selected goal with minimal energy consumption.

### Task 3: Add a Position Controller to Robotic Arm URDF

1. Run simulation && controller_manager

```sh
ros2 launch module_5_assignment gazebo_3dof_arm.launch.py 
```

2. Give angle for each joint

```sh
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 0.5, -0.0]"
```

### Task 4: Implement and Visualize LQR for Multi-Goal Following

- **Objective:** Enhance the multi-goal following behavior of TurtleBot3 using LQR and analyze the impact of different Q and R matrix values on the robot's path and performance.


1. Run Turtlebot3 Simulation && Rviz2 && LQR node && Markers

```sh
ros2 launch module_5_assignment tb3_empty.launch.py
```


2. How to make your own ros2 bag

```sh
ros2 bag record -o <file_name> /odom /tf /tf_static /imu /clock /visualization_marker_array /cmd_vel
```

3. Run Bag Files

```sh
ros2 bag play <bag_name>
```

# LQR Trajectory Analysis
This document provides an analysis of different configurations for the Q and R matrices in the LQR (Linear Quadratic Regulator) and their impact on trajectory tracking performance.

Overview

The following trajectories were tested:

Green points: represent waypoints to guide the robot.
Red line: represents the actual trajectory of the robot.
Three configurations of Q (state cost) and R (control cost) matrices were tested. Below is a detailed analysis of the results.

1. First Set of 


Results:

![First Configuration](doc/first_set_qr.jpg)

- Balanced tracking with slight deviations.
- The trajectory is smooth with minimal overshooting.
- Suitable for general-purpose tasks.

2. Second Set of 


Results:

-The trajectory closely follows the waypoints.
- Exhibits oscillatory behavior due to aggressive control.
- Suitable for scenarios requiring high accuracy but prone to instability.

3. Third Set of 


Results:

- The trajectory prioritizes smoothness over accuracy.
- Significant deviations from waypoints.
- Suitable for energy conservation and smooth paths.
- Comparative Analysis

Table

First Configuration: Provides a balanced trade-off between tracking accuracy and control effort.
Suitable for general-purpose applications.

Second Configuration: Focuses heavily on accuracy but results in oscillations. Not recommended for scenarios where stability is critical.


Third Configuration: Prioritizes smoothness and minimal control effort but sacrifices accuracy.Ideal for applications prioritizing energy efficiency or smooth trajectories.