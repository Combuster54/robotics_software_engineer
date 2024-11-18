# Module 2 Assignment: Developing Custom ROS 2 Nodes and Launch Files

## module_2_assignment package provided two set of patterns for turtles inside turtlesim

- CirclePattern: Moves the turtle in a circular path using a radius input.
- BackandForwardPattern: Moves the turtle in a back-and-forth motion using a linear velocity input.

In addition, you can change those parameters dynamically using the command

```sh
ros2 param set <node_name> <parameter> <value>
```

## How to clone && build the project


1. Clone the repository

```sh
git clone https://github.com/Combuster54/robotics_software_engineer.git
```

2. Go into the repo and change into assignments branch

```sh
src robotics_software_engineer/
git switch assignments
```

3. Get back to your workspace and compile 

```sh
colcon build && source install/setup.bash
```

## Execute Task 1 && Task 2

### Goals: Create a Custom ROS 2 Node && Develop a Launch File

1. Launch Turtle Circle Pattern

```sh
ros2 launch module_2_assignment turtle_circle_pattern.launch.py
```

2. Change the radius param dynamically using

```sh
ros2 param set /circle_pattern radius 2.0
```

## Execute Task 3 && Task 4

### Goals: Modify the Turtlesim Simulation Environment && Modify Turtle Behavior with Parameters


1. Initialize the enviroment with 5 turtles and the backward_and_forward movement

```sh
ros2 launch module_2_assignment task3.launch.py
```

2. change the velocity for every turtle

```sh
ros2 param set /backward_and_forward_pattern l_velocity 5.0
```