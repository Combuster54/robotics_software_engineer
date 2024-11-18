from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 "{x: 1.0, y: 10.0, theta: 0.0, name: 'turtle2'}"],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 "{x: 3.0, y: 8.0, theta: 0.0, name: 'turtle3'}"],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 "{x: 7.0, y: 4.0, theta: 0.0, name: 'turtle4'}"],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 "{x: 9.0, y: 2.0, theta: 0.0, name: 'turtle5'}"],
            output='screen'
        )
    ])
