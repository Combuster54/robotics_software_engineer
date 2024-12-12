#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory("module_6_assignment")
    rviz_config_file = os.path.join(pkg_share_dir, 'config', 'config.rviz')

    ekf_node = Node(
        package='module_6_assignment',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
    )

    marker_node = Node(
        package='module_6_assignment',
        executable='marker_pub',
        name='marker_pub_node',
        output='screen',
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([ 
        ekf_node,
        marker_node,
        rviz_node
    ])