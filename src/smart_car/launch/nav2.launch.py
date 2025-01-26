import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Define package name and gather directories
    package_name = 'smart_car'
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'smartcar.urdf')
    world_path = os.path.join(get_package_share_directory(package_name), 'world', 'smalltown.world')
    ekf_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')

    # Nav2 directories
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    nav2_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    nav2_map_path = os.path.join(get_package_share_directory(package_name), 'config', 'smalltown_world.yaml') 
    nav2_bt_path = os.path.join(get_package_share_directory(package_name), 'config', 'bt_nav2.xml')

    # Path to the controllers.yaml file
    controllers_yaml_path = os.path.join(get_package_share_directory(package_name), 'config', 'controllers.yaml')

    # Gazebo launch file path from gazebo_ros package
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    # Define the nav2 launch file path
    nav2_launch_file = os.path.join(nav2_bringup_path, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/home/ros2/ros2_ws2/src/smart_car/rviz/nav2.rviz',
        ),
        
        # Start Gazebo and load the specified world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': world_path}.items()
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_smart_car',
            output='screen',
            arguments=['-entity', 'smart_car', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0']
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]), 
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),



        # Load controllers from controllers.yaml
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            parameters=[controllers_yaml_path],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),

        # Odometry Publisher
        Node(
            package='smart_car',
            executable='odom.py',
            name='odometry_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]      
        ),
        
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[("odometry/filtered", "/odom")]
        ),
        
        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'slam': LaunchConfiguration('slam'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params_path,
                'map': nav2_map_path,
                'auto_start': 'true',
                'bt_xml_file': nav2_bt_path
            }.items()
        )
    ])
