find_package(ament_cmake_gmock REQUIRED)

add_library(rclcpp_test src/rclcpp_test.cpp)
target_include_directories(rclcpp_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rclcpp_test PUBLIC rclcpp::rclcpp)

ament_add_gmock(test_spot_image_publisher
  src/images/test_spot_image_publisher.cpp
)
target_link_libraries(test_spot_image_publisher spot_image_publisher)

ament_add_gmock(test_spot_image_publisher_node
  src/images/test_spot_image_publisher_node.cpp
)
target_link_libraries(test_spot_image_publisher_node spot_image_publisher)

ament_add_gmock(test_spot_image_sources
  src/images/test_spot_image_sources.cpp
)
target_link_libraries(test_spot_image_sources spot_image_publisher)

ament_add_gmock(test_parameter_interface
  test_parameter_interface.cpp
)
target_link_libraries(test_parameter_interface spot_image_publisher rclcpp_test)

###############################################################################
# test_kinematic_conversions

ament_add_gmock(test_kinematic_conversions
    src/conversions/test_kinematic_conversions.cpp
)
target_link_libraries(test_kinematic_conversions spot_image_publisher)

###############################################################################
# test_common_conversions

ament_add_gmock(test_common_conversions
    src/conversions/test_common_conversions.cpp
)
target_link_libraries(test_common_conversions spot_image_publisher)