cmake_minimum_required(VERSION 3.8)
project(spot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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

add_executable(robot_squat src/robot_squat.cpp)
ament_target_dependencies(robot_squat
  rclcpp
  std_msgs
)
add_executable(arm_wiggle src/arm_wiggle.cpp)
ament_target_dependencies(arm_wiggle
  rclcpp
  std_msgs
)
install(TARGETS robot_squat arm_wiggle
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY xacro launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
