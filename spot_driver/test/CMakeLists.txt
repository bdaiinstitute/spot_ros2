find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

#rclcpp_test

add_library(rclcpp_test src/rclcpp_test.cpp)
target_include_directories(rclcpp_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rclcpp_test PUBLIC rclcpp::rclcpp)

ament_add_gmock(test_clock_skew
  src/test_clock_skew.cpp
)
target_include_directories(test_clock_skew
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_clock_skew spot_api)

# test_spot_image_publisher

ament_add_gmock(test_spot_image_publisher
  src/images/test_spot_image_publisher.cpp
)
target_include_directories(test_spot_image_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_spot_image_publisher spot_api)

# test_spot_image_publisher_node

ament_add_gmock(test_spot_image_publisher_node
  src/images/test_spot_image_publisher_node.cpp
)
target_include_directories(test_spot_image_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_spot_image_publisher_node spot_api)

# test_spot_image_sources

ament_add_gmock(test_spot_image_sources
  src/images/test_spot_image_sources.cpp
)
target_link_libraries(test_spot_image_sources spot_api)

# test_parameter_interface

ament_add_gmock(test_parameter_interface
  src/test_parameter_interface.cpp
)
target_link_libraries(test_parameter_interface spot_api rclcpp_test)

# test_kinematic_conversions

ament_add_gmock(test_kinematic_conversions
    src/conversions/test_kinematic_conversions.cpp
)
target_link_libraries(test_kinematic_conversions spot_api)

# test_common_conversions

ament_add_gmock(test_common_conversions
    src/conversions/test_common_conversions.cpp
)
target_link_libraries(test_common_conversions spot_api)

# test_kinematic_service

ament_add_gmock(test_kinematic_service
    src/kinematic/test_kinematic_service.cpp
)
target_include_directories(test_kinematic_service
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_kinematic_service spot_api)

#rclpy_test

set(_pytest_tests
  pytests/test_copyright.py
  pytests/test_dock.py
  pytests/test_execute_dance.py
  pytests/test_get_choreography_status.py
  pytests/test_list_all_dances.py
  pytests/test_list_all_moves.py
  pytests/test_locomotion_mode.py
  pytests/test_max_velocity.py
  pytests/test_pep257.py
  pytests/test_power_off.py
  pytests/test_recorded_state_to_animation.py
  pytests/test_release.py
  pytests/test_robot_command.py
  pytests/test_rollover.py
  pytests/test_ros_interfaces.py
  pytests/test_self_right.py
  pytests/test_sit.py
  pytests/test_stair_mode.py
  pytests/test_stand.py
  pytests/test_start_recording_state.py
  pytests/test_stop.py
  pytests/test_stop_recording_state.py
  pytests/test_undock.py
  pytests/test_upload_animation.py
)
foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()
