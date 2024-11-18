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
        Node(
            package='module_2_assignment',
            executable='circle_pattern',
            name='circle_pattern',
            parameters=[
                {'radius': 1.0}
            ]
        ),
    ])
