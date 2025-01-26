cmake_minimum_required(VERSION 3.8)
project(smart_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smartcar_msgs REQUIRED)  # Ensure this is included if needed
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# For testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter which checks for copyrights if needed
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install auxiliary files
install(DIRECTORY 
  urdf
  launch
  world
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/odom.py
  scripts/keyboard_control.py
  scripts/joint_state_pub.py
  scripts/vehicle_status.py  
# Ensure this script is here
  DESTINATION lib/${PROJECT_NAME}
)




ament_package()
