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

# test_spot_image_publisher

ament_add_gmock(test_spot_image_publisher
  src/images/test_spot_image_publisher.cpp
)
target_include_directories(test_spot_image_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_spot_image_publisher spot_image_publisher)

# test_spot_image_publisher_node

ament_add_gmock(test_spot_image_publisher_node
  src/images/test_spot_image_publisher_node.cpp
)
target_include_directories(test_spot_image_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_spot_image_publisher_node spot_image_publisher)

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

# test_spot_robot_state_publisher

ament_add_gmock(test_state_publisher
  src/robot_state/test_state_publisher.cpp
)
target_include_directories(test_state_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_state_publisher state_publisher)

# test_spot_robot_state_publisher_node

ament_add_gmock(test_state_publisher_node
  src/robot_state/test_state_publisher_node.cpp
)
target_include_directories(test_state_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_state_publisher_node state_publisher)

ament_add_gmock(test_conversions_geometry
  src/conversions/test_geometry.cpp
)
target_include_directories(test_conversions_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_conversions_geometry spot_api)

ament_add_gmock(test_conversions_robot_state
  src/conversions/test_robot_state.cpp
)
target_include_directories(test_conversions_robot_state
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_conversions_robot_state spot_api)
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
target_link_libraries(test_kinematic_service spot_inverse_kinematics)

#rclpy_test

set(_pytest_tests
  pytests/test_copyright.py 
  pytests/test_pep257.py 
  pytests/test_power_management.py
  pytests/test_ros_interfaces.py
  )
foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()
