cmake_minimum_required(VERSION 3.8)
project(spot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ROS 2 control specific dependencies
find_package(controller_manager REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(controller_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# Messages
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Spot URDF files
find_package(spot_description REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(noarm_squat src/noarm_squat.cpp)
ament_target_dependencies(noarm_squat
  rclcpp
  sensor_msgs
  std_msgs
)
add_executable(wiggle_arm src/wiggle_arm.cpp)
ament_target_dependencies(wiggle_arm
  rclcpp
  sensor_msgs
  std_msgs
)
install(TARGETS noarm_squat wiggle_arm
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY xacro launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
