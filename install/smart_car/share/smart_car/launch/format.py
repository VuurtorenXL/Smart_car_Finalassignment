import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directories
    pkg_smart_car = get_package_share_directory('smart_car')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to localization file (e.g., Extended Kalman Filter (EKF) config)
    robot_localization_file_path = os.path.join(pkg_smart_car, 'config', 'ekf.yaml')

    # Path to your robot's URDF file
    urdf_file = os.path.join(pkg_smart_car, 'urdf', 'smartcar.urdf')

    # Path to RViz configuration file
    rviz_config_file = os.path.join(pkg_smart_car, 'config', 'smartcar_nav2.rviz')

    # Path to Nav2 parameters file
    nav2_params_file = os.path.join(pkg_smart_car, 'config', 'nav2_params.yaml')

    # Path to behaviour Tree parameters file
    bh_tree_file = os.path.join(pkg_smart_car, 'config', 'bh_tree_nav2.xml')

    # Path to your world file (Gazebo world)
    world_file_path = os.path.join(pkg_smart_car, 'world', 'smalltown.world')

    # Path to map file for SLAM and localization
    map_file_path = os.path.join(pkg_smart_car, 'map', 'smalltown_world.yaml')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation (Gazebo) clock')

    declare_slam = DeclareLaunchArgument(
        'slam', default_value='True', description='Whether to run SLAM or not')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=map_file_path, description='Full path to map yaml file to load')

    # Odometry node
    odometry_node = Node(
        package='smart_car',
        executable='odom.py',  # Ensure the script is executable
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'smartcar', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Robot State Publisher (publishes /robot_description and /tf_static)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[urdf_file]
    )

    # RViz2 visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # EKF Localization (robot_localization)
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch the ROS 2 Navigation stack (Nav2)
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),  # Enable or disable SLAM
            'map': map_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),  # Sync with simulation clock
            'params_file': nav2_params_file,  # Nav2 parameters configuration
            'autostart': 'true',  # Auto-start the navigation stack
            'default_bt_xml_filename': bh_tree_file
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        declare_map_yaml_cmd,
        gazebo_launch,  # Launch Gazebo with the world
        spawn_entity,  # Spawn the robot model in the Gazebo world
        joint_state_publisher_node,  # Joint State Publisher
        robot_state_publisher_node,  # Robot State Publisher
        rviz_node,  # RViz2 for visualization
        start_robot_localization_cmd,  # EKF for localization
        odometry_node,  # Start the odometry node
        start_nav2_cmd  # Start Navigation2 (Nav2) stack
    ])
