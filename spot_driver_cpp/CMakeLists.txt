cmake_minimum_required(VERSION 3.22)
project(spot_driver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(bosdyn REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tl_expected REQUIRED)

add_library(spot_image_publisher src/spot_image_publisher.cpp src/spot_image_sources.cpp src/spot_interface.cpp)
target_include_directories(spot_image_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_image_publisher PUBLIC bosdyn::bosdyn_client_static)
ament_target_dependencies(spot_image_publisher PUBLIC rclcpp cv_bridge sensor_msgs tl_expected)

add_executable(spot_image_publisher_node src/spot_image_publisher_node.cpp)
target_link_libraries(spot_image_publisher_node PUBLIC spot_image_publisher)

# TODO: Fix linking against shared SDK libraries to allow loading as a component node
# add_library(spot_image_publisher_node_component SHARED src/spot_image_publisher_node_component.cpp)
# target_link_libraries(spot_image_publisher_node_component PUBLIC spot_image_publisher)
# ament_target_dependencies(spot_image_publisher_node_component PUBLIC rclcpp_components)
# rclcpp_components_register_nodes(spot_image_publisher spot_ros2::SpotImagePublisherNode)

install(
  TARGETS
    spot_image_publisher
    spot_image_publisher_node
    # spot_image_publisher_node_component
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files

  # set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files

  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
