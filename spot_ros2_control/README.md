# spot_ros2_control

TODO: Description

Testing is currently done on a robot with an arm with
`ros2 launch spot_ros2_control spot_ros2_control.launch.py has_arm:=true controllers_config:=spot_controllers_with_arm.yaml hardware_interface:=spot-sdk`
with the robot login info specified as environment variables. 

To use a mock hardware interface, run with `hardware_interface:=mock`.
