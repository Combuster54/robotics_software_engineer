from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_path = get_package_share_directory('module_3_assignment')
    urdf_file = os.path.join(pkg_path, 'urdf', '3dof_arm.urdf')

    return LaunchDescription([
        # Node for publishing robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{"use_sim_time": True},
                        {"publish_frequency": 100.0}
                        ],
            arguments=[urdf_file]
        ),
        # Node for publishing joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        # Gazebo simulator process
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Node for spawning the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-entity', '3dof_arm'
            ]
        )
    ])
