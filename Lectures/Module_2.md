
# Module # 2 : ROS2 Communication and Turtlesim ( V1.0 )
- Package Created : *drive_mobile_robot*
    - ros fundamentals + basic Mathematics programming
        - Finding what messages to write
        - ROS2 nodes communication , launch files .
    - Mathematics Fundamentals for robot Motion
    - Bring Mathematics from module_5 to this simulation
- Questions
    - How to find out which topic to publish on and why?
        - We have generic convenstions.
    - Type of message udnerstanding ?
---
## Lectures

- `a_module_intro`
- `b_ros2_installation_workspace_source`
    - Install ros2 humble , workspace robotics_ws , package creation , source
- `c_package_creation_sourcing`
    - Development Environment setup
        - [screen] : How sourcing works
- `d_basic_node_communication`
    - CMD line publishing
    - ROS2 topic + rqt_graph
        - Nodes Communication with One Another
    - CPP custom Nodes for publishing
    - Hosla , start from totally basic nodes and make smaller changes
    - *overlay*
        - [screen] : write down how nodes communicate upon nodes and topics
        - [face] :
            - What topics to publish and where to use which one -> important question
            - Why ROS have wrappers on simple C++ data types ?
- `e_turtlesim_node_drive_node`
    - First write node to drive
    - Then show teleop node -> its code on github as well
    - *overlay* :
        - [screen] : Process of nodes working before writing code
        - [face] :
            - Here you saw that already written code does amazing job and interestingly very usefull code is already written
            - But we need to learn how it is written in detail and then utilizing open source code will start to make sense
- `f_launch_files`
    - Launch single tbsim + drive Node -> then add  teleop node as well
    - Launch Multi tbsim + drive Node -> reach the error point of topic name issues
    - *overlay* :
        - [screen] : Launch file working written
- `g_multi_robot_drive`
    - Arguments , remappings
    - *overlay* :
        - Hear all endings , overlay for all of these
            - Ga -> single sim multi Robot
            - Gb -> problem is publish topic is not changing
        - [screen] : Multi Robots when ever we need we need to seperate so communication is not streamlined like namespaces.
- `h_github_communities_issue _templates`
    - *overlay* :
        - [screen]
        - [face]
- `i_turtlebot3_setup`
    - Drive
    - Gazebo visualize
    - Help them not freak out
    - *overlay* :
        - [screen] : explain turtlebot3 folders i little bit , sourcing , models
        - [face] : Lets bring in a robot package which will not make sense but from topics prespective there is alot to understand , Hosla keh we will understand in detail how it is running
<!-- - `k_multi_tb3sim`
    - Bring in multi tb3sims in a gazebo world using launch files
    - Break down each file launching
    - Drive them straight
    - show how we can drive them in sqaures -->

---
## Formating and linting
- clang-format -i  f_formate.cpp
- clang-tidy f_formate.cpp --fix
---