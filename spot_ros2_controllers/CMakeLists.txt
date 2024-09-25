cmake_minimum_required(VERSION 3.16)
project(spot_ros2_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  forward_command_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)


find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  spot_passthrough_controller_parameters
  include/spot_ros2_controllers/spot_passthrough_controller_parameters.yaml
)

add_library(spot_ros2_controllers SHARED
  src/spot_passthrough_controller.cpp
)
target_compile_features(spot_ros2_controllers PUBLIC cxx_std_17)
target_include_directories(spot_ros2_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/spot_ros2_controllers>
)
target_link_libraries(spot_ros2_controllers PUBLIC
  spot_passthrough_controller_parameters forward_command_controller::forward_command_controller
)
ament_target_dependencies(spot_ros2_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(spot_ros2_controllers PRIVATE "SPOT_PASSTHROUGH_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface spot_passthrough_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/spot_ros2_controllers
)
install(
  TARGETS
    spot_ros2_controllers
    spot_passthrough_controller_parameters
  EXPORT export_spot_ros2_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_spot_ros2_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
