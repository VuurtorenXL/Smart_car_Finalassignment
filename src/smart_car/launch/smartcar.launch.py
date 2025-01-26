import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_directory('smart_car'), 'urdf', 'smartcar.urdf')
    rviz2_path = os.path.join(get_package_share_directory('smart_car'), 'launch ', 'smartcar.launch.py')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]  
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'rviz2_config_file' : rviz2_path
            }]
        ),
        Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # Set to true if in simulation
            'odom0': '/odom',  # Adjust to your odom topic if available
            'imu0': '/imu/data',  # Your IMU topic
            'imu0_config': [True, True, True, False, False, False,  # Orientation (x, y, z)
                            True, True, True,  # Angular velocity (x, y, z)
                            True, True, True], # Linear acceleration (x, y, z)
            'imu0_differential': False,  # Whether or not to treat the IMU data differentially
            'imu0_remove_gravitational_acceleration': True  # Removes gravitational component
            }]
        )
    ])
