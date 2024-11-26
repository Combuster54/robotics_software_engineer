# Debug errors inside launch files

### camera_line_following.launch.py

type_error : name_error 
variable: spawn_turtlebot_cmd
error: spawn_tb3.launch.py 
fixed_to: spawn_turtlebot3.launch.py

type_error : name_error 
variable: line_following
error: robot_sensing 
fixed_to: robot_sensing_debug

type_error : robot initial pose
variable : x_pose , y_pose
fixed_to : -6.155355 , 2.116028


### custom_sensors.launch.py

type_error : name_error 
variable: pkgPath
error: robot_sensing 
fixed_to: robot_sensing_debug


### lidar_maze_solving.launch.py

type_error : robot initial pose
variable : x_pose , y_pose
fixed_to : -4.402807 , -4.044180

type_error : name_error 
variable: world
error: mazes.world
fixed_to: maze.world

type_error : name_error 
variable: maze_solver
error: robot_sensing 
fixed_to: robot_sensing_debug