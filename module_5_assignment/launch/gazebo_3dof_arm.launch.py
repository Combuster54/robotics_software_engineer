from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


os.environ['GAZEBO_GUI_FULLSCREEN'] = 'false'  # Deshabilita pantalla completa
os.environ['GAZEBO_GUI_SIZE'] = '1280x720'  # Cambia el tamaño inicial

def generate_launch_description():
    pkg_path = get_package_share_directory('module_5_assignment')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file = os.path.join(pkg_path, 'urdf', '3dof_arm.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.3')
    

    world = os.path.join(
        get_package_share_directory('module_5_assignment'),
        'worlds',
        'empty.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    
    spawn_3dof_arm = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-entity', '3dof_arm',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose
            ]
        )
    rsp =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{"use_sim_time": True},
                        {"publish_frequency": 100.0}
                        ],
            arguments=[urdf_file]
        )

    controller_manager_node = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["forward_position_controller"]
        )
    
    return LaunchDescription([

        gzserver_cmd,
        gzclient_cmd,
        rsp,
        spawn_3dof_arm,
        controller_manager_node
    ])
