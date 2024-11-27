# Module 4 Assignment: Sensor Data Manipulation for Robot Control


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

### Task 1: Restructure Line Following Project Code

- You can see the new Line Folowing Project Code in line_following.hpp && line_following.cpp 

### Task 2: Design Software Flow for Maze Solving

- Go into doc folder

### Task 3: Implement Maze Solving in Gazebo

To run launch task 3, (Square Maze World + Wall Follower + Velocity and Acceleration by IMU)

```sh
ros2 launch module_4_assignment tb3_square_maze.launch.py
```

### Task 4: Fix Errors in Launch Files

- Successfully run the **robot_sensing_debug** packge by removing errors and
  - Document the error debugging process.
  - Provide a summary of the errors encountered and how they were resolved.

### Submission Process

1. **Create Files:**
   - Navigate to the `module_4_assignment` package.
   - Create the required files, including the maze environment, refactored code, and debugging documentation.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_4_assignment` package.
   - Provide details about the files you created, including explanations of the code and the commands needed to run your simulations and tests.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.

## Learning Outcome

By completing this assignment, you will:
- Learn to control robots based on sensor data.
- Improve your skills in debugging and fixing issues in complex robotic systems.
