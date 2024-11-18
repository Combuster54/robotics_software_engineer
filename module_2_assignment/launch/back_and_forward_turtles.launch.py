from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='module_2_assignment',
            executable='backward_and_forward_pattern',
            name='backward_and_forward_pattern',
            parameters=[
                {'l_velocity': 1.0}
            ]
        )
    ])
