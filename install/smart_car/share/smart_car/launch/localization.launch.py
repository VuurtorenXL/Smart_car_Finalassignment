import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'smart_car'
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'smartcar.urdf')
    world_path = os.path.join(get_package_share_directory(package_name), 'world', 'smalltown.world')
    ekf_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    
    # Gazebo launch file path from gazebo_ros package
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    
    return LaunchDescription([
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
                'use_sim_time': True
                }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/ros2/ros2_ws2/src/smart_car/rviz/smart_car.rviz']
        ),
        
        # Odometry Publisher
        Node(
            package='smart_car',
            executable='odom.py',
            name='odometry_publisher',
            output='screen',
            parameters= [{'use_sim_time': True}]      
        ),
        
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters= [ekf_config_path,
            {'use_sim_time': True}],
            remappings=[("odometry/filtered","/odom")]
            
        )
    ])
