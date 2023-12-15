find_package(ament_cmake_gmock REQUIRED)

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
target_link_libraries(test_spot_image_sources spot_image_publisher)

# test_parameter_interface

ament_add_gmock(test_parameter_interface
  src/test_parameter_interface.cpp
)
target_link_libraries(test_parameter_interface spot_image_publisher rclcpp_test)
