import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Paths
    urdf_path = os.path.join(get_package_share_directory('smart_car'), 'urdf', 'smartcar.urdf')
    world_path = os.path.join(get_package_share_directory('smart_car'), 'world', 'smalltown.world')

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

     #   Node(
    #      package='controller_manager',
   #       executable='controller_manager',
  #        name='controller_manager',
 #         output='screen'
#),


        Node(
            package='smart_car',  # Replace with your package name
            executable='keyboard_control.py',    # The name of your Python script without the `.py`
            name='keyboard_control',
            output='screen'
        ),

        Node(
            package='smart_car',  # Replace with your package name
            executable='vehicle_status.py',    # The name of your Python script without the `.py`
            name='vehicle_status',
            output='screen'
        ),




        



        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/ros2/ros2_ws2/src/smart_car/rviz/rviz_smartcar_working.rviz']
        ),
        
        # Odometry Publisher
        Node(
            package='smart_car',
            executable='odom.py',
            name='odometry_publisher',
            output='screen'      
        ),
        
        # Robot Localization Node (EKF)
        #Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[{
        #        'use_sim_time': True,  # Set to True for simulation
        #        'odom0': '/odom', 
        #        'imu0': '/imu/data', 
        #        'imu0_config': [True, True, True, False, False, False,  # Orientation (x, y, z)
        #                        True, True, True,  # Angular velocity (x, y, z)
        #                        True, True, True], # Linear acceleration (x, y, z)
        #        'imu0_differential': False,
        #        'imu0_remove_gravitational_acceleration': True
        #    }]
        #)
    ])
