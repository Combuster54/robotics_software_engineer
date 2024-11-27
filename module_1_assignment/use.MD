# Module 1 Assignment: Introduction to OOP Concepts in C++ for Robotics

## Commands

first, clone the repository

```
git clone https://github.com/Combuster54/robotics_software_engineer.git
```

then go into the repo and change into assignments branch

```
src robotics_software_engineer/
git switch assignments
```

get back to your workspace and compile 

```
colcon build && source install/setup.bash
```

## Run Task 1

```
ros2 run module_1_assignment task1
```

Output Expected
```sh
combuster@DESKTOP-LMQHD5S:~/robotisim_ws$ ros2 run module_1_assignment task1
The Robot AlphaBot is moving forward
The Robot AlphaBot has been stopped
The Robot BetaBot is moving backward
The Robot BetaBot has been stopped
```

## Run Task 2

```sh
ros2 run module_2_assignment task2
```

Output Expected
```sh
combuster@DESKTOP-LMQHD5S:~/robotisim_ws$ ros2 run module_1_assignment task2
Temperature Sensor Reading: Sensor Data: 25.5
Distance Sensor Reading: Sensor Data: 150
```
