# spot_ros2_control

TODO: Description

## notes

Testing is currently done on a robot with an arm with
`ros2 launch spot_ros2_control spot_ros2_control.launch.py has_arm:=true controllers_config:=spot_controllers_with_arm.yaml hardware_interface:=spot-sdk`
with the robot login info specified as environment variables. 

If you `CTRL+C`, the hardware interface won't get shut down correctly and the lease will still be held. To get around this you will have to take control from the tablet then release control. The workaround to this for now is to call `ros2 control set_hardware_component_state SpotSystem inactive` in another terminal then `CTRL+C`
