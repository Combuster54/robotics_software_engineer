# Module 5 Assignment: Enhancing Robot Control for Smooth Motion

## How to use this project

### Commands

Clone the repository

```
git clone https://github.com/Combuster54/robotics_software_engineer.git
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


1. Run Turtlebot3 Simulation && Rviz2

```sh
ros2 launch module_5_assignment tb3_empty.launch.py
```


2. How to make your own ros2 bag

```sh
ros2 bag record -o <file_name> /odom /tf /tf_static /imu /clock /visualization_marker_array /cmd_vel
```

  - **Subtasks:**

    a. **Visualize Goals and Path in RViz:**
       - Implement visualizations in RViz to display the positions of the goals and the robot’s path.
       - Add markers for each goal and a trail of points to represent the robot's odometry during the path following.

    b. **Experiment with Different Q and R Values:**
       - Test the LQR controller with three different sets of Q and R matrices.
       - Record and compare how these values affect the robot's behavior, particularly in terms of stability, speed, and smoothness when transitioning between goals.

    c. **Document and Analyze the Results:**
       - Provide a detailed report on the performance of the LQR controller with different Q and R settings.
       - Include visualizations from RViz to illustrate the robot’s path and goal positions for each scenario.
       - Discuss which set of Q and R values provided the best balance between responsiveness and smoothness in the robot’s motion.

---
### Submission Process

1. **Create Files:**
   - Navigate to the `module_5_assignment` package.
   - Create the required files for the improved line-following, goal selection, and position control tasks.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_5_assignment` package.
   - Provide details about the files you created, including explanations of the code and the commands needed to run your simulations and controllers.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.

## Learning Outcome

By completing this assignment, you will:
- Learn to control robots to create smoother and more efficient motion.
- Enhance your understanding of using controllers and path planning algorithms to optimize robot behavior.

