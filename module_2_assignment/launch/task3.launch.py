from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_name = 'module_2_assignment'
    launch1_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        'multi_turtles.launch.py'
    )
    launch2_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        'back_and_forward_turtles.launch.py'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch1_path)
        ),
        
        # Incluir el segundo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch2_path)
        )
    ])
