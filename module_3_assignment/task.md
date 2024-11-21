# Module 3 Assignment: URDF and Robot Creation in ROS 2

## module_3_assignment package provided urdf models
- Ackerman Mobile Robot
- 3DOF Arm Robot

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

## Explanation

### 3DOF Arm Robot

This robot has 3 degrees of freedom (DOF). Each revolute joint has a range of <-45,+45>

### Ackerman Robot

This robot has some similarities with convetional diff_drive robot, but here we add 2 additional links and joints to simulate the rotation movement. Each wheel has a range of <-45,+45>

## Execute Task 1 && Task 2

### Goals: Create a 3DOF Arm Robot

1. Launch Arm Robot on Rviz

```sh
ros2 launch module_3_assignment rviz_3dof_arm.launch.py
```

2. Launch Arm Robot on Rviz

```sh
ros2 launch module_3_assignment gazebo_3dof_arm.launch.py
```

## Execute Task 3

1. Launch Arm Robot + Diff Drive Robot on Rviz

```sh
ros2 launch module_3_assignment rviz_robot.launch.py
```

2. Launch Arm Robot + Diff Drive Robot on Gazebo

```sh
ros2 launch module_3_assignment gazebo_robot.launch.py
```

3. Launch Ackerman Robot on Rviz

```sh
ros2 launch module_3_assignment rviz_ackerman.launch.py
```

4. Launch Ackerman Robot on Gazebo

```sh
ros2 launch module_3_assignment gazebo_ackerman.launch.py
```