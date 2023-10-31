find_package(ament_cmake_gmock REQUIRED)
# find_package(gmock REQUIRED)
# find_package(gtest REQUIRED)

add_library(rclcpp_test src/rclcpp_test.cpp)
target_include_directories(rclcpp_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rclcpp_test PUBLIC rclcpp::rclcpp)

ament_add_gmock(test_spot_image_publisher
  test_spot_image_publisher.cpp
)
target_link_libraries(test_spot_image_publisher spot_image_publisher)

ament_add_gmock(test_spot_image_sources
  test_spot_image_sources.cpp
)
target_link_libraries(test_spot_image_sources spot_image_publisher)

ament_add_gmock(test_parameter_interface
  test_parameter_interface.cpp
)
target_link_libraries(test_parameter_interface spot_image_publisher rclcpp_test)
