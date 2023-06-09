cmake_minimum_required(VERSION 3.5)
project(spot_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(bosdyn_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStateArray.msg"
  "msg/BehaviorFault.msg"
  "msg/EStopStateArray.msg"
  "msg/FootStateArray.msg"
  "msg/LeaseArray.msg"
  "msg/LeaseOwner.msg"
  "msg/Metrics.msg"
  "msg/MobilityParams.msg"
  "msg/SystemFault.msg"
  "msg/WiFiState.msg"
  "msg/BatteryState.msg"
  "msg/BehaviorFaultState.msg"
  "msg/EStopState.msg"
  "msg/Feedback.msg"
  "msg/FootState.msg"
  "msg/Lease.msg"
  "msg/LeaseResource.msg"
  "msg/PowerState.msg"
  "msg/SystemFaultState.msg"
  "srv/ListGraph.srv"
  "srv/ListWorldObjects.srv"
  "srv/SetLocomotion.srv"
  "srv/SetVelocity.srv"
  "srv/ExecuteDance.srv"
  "srv/ListAllDances.srv"
  "srv/ListAllMoves.srv"
  "srv/UploadAnimation.srv"
  "srv/ClearBehaviorFault.srv"
  "srv/ListSounds.srv"
  "srv/LoadSound.srv"
  "srv/PlaySound.srv"
  "srv/DeleteSound.srv"
  "srv/GetVolume.srv"
  "srv/SetVolume.srv"
  "srv/GraphNavUploadGraph.srv"
  "srv/GraphNavClearGraph.srv"
  "srv/GraphNavSetLocalization.srv"
  "srv/GraphNavGetLocalizationPose.srv"
  "action/NavigateTo.action"
  "action/RobotCommand.action"
  "action/Trajectory.action"
  "action/Manipulation.action"
  DEPENDENCIES bosdyn_msgs builtin_interfaces common_interfaces geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
