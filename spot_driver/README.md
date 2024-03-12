# spot_driver

This package provides the central ROS 2 interface to interact with Spot. 

The primary launchfile in this package is `spot_driver.launch.py`.

`ros2 launch spot_driver spot_driver.launch.py`

* Various driver parameters can be customized using a configuration file -- see [spot_ros_example.yaml](config/spot_ros_example.yaml) for reference. To launch the driver with a configuration file, add the launch argument `config_file:=path/to/config.yaml`. Note that you can specify the hostname, username, and password for login to the robot in this file, or they can alternatively be set in the environment variables `SPOT_IP`, `BOSDYN_CLIENT_USERNAME`, and `BOSDYN_CLIENT_PASSWORD`, respectively. 
* To launch the process within a namespace, add the launch argument `spot_name:={name}`
* To launch RViz, add the launch argument `launch_rviz:=True`. This will automatically generate the appropriate RViz config file for your robot's name using `rviz.launch.py`.
* To publish point clouds, add the launch argument `publish_point_clouds:=True`. This is disabled by default.


The Spot driver contains both Python and C++ nodes. Spot's Python SDK is used for many operations. For example, `spot_ros2` is the primary node that connects with Spot and creates the ROS 2 action servers and services. Spot's C++ SDK is used in nodes like `spot_image_publisher_node` to retrieve images from Spot's RGB and depth cameras at close to their native refresh rate of 15 Hz -- something that is not possible using the Python SDK. 

## Examples
For some examples of using the Spot ROS 2 driver, check out [`spot_examples`](../spot_examples/).


## `spot_ros2` interfaces

### Subscriptions
|Interface Name                                    |Type                                              |Description                                       |
|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|
|/body_pose                                        |geometry_msgs/msg/Pose                            |    | 
|/cmd_vel                                          |geometry_msgs/msg/Twist                           |    | 


### Publishers
|Interface Name                                    |Type                                              |Description                                       |
|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|
|/status/feedback                                  |spot_msgs/msg/Feedback                            |    | 
|/status/leases                                    |spot_msgs/msg/LeaseArray                          |    | 
|/status/metrics                                   |spot_msgs/msg/Metrics                             |    | 
|/status/mobility_params                           |spot_msgs/msg/MobilityParams                      |    | 


### Services

|Interface Name                                    |Type                                              |Description                                       |
|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|
|/claim                                            |std_srvs/srv/Trigger                              |    | 
|/clear_behavior_fault                             |spot_msgs/srv/ClearBehaviorFault                  |    | 
|/delete_logpoint                                  |spot_msgs/srv/DeleteLogpoint                      |    | 
|/delete_sound                                     |spot_msgs/srv/DeleteSound                         |    | 
|/dock                                             |spot_msgs/srv/Dock                                |    | 
|/estop/gentle                                     |std_srvs/srv/Trigger                              |    | 
|/estop/hard                                       |std_srvs/srv/Trigger                              |    | 
|/estop/release                                    |std_srvs/srv/Trigger                              |    | 
|/execute_dance                                    |spot_msgs/srv/ExecuteDance                        |    | 
|/get_choreography_status                          |spot_msgs/srv/GetChoreographyStatus               |    | 
|/get_gripper_camera_parameters                    |spot_msgs/srv/GetGripperCameraParameters          |    | 
|/get_led_brightness                               |spot_msgs/srv/GetLEDBrightness                    |    | 
|/get_logpoint_status                              |spot_msgs/srv/GetLogpointStatus                   |    | 
|/get_ptz_position                                 |spot_msgs/srv/GetPtzPosition                      |    | 
|/get_volume                                       |spot_msgs/srv/GetVolume                           |    | 
|/graph_nav_clear_graph                            |spot_msgs/srv/GraphNavClearGraph                  |    | 
|/graph_nav_get_localization_pose                  |spot_msgs/srv/GraphNavGetLocalizationPose         |    | 
|/graph_nav_set_localization                       |spot_msgs/srv/GraphNavSetLocalization             |    | 
|/graph_nav_upload_graph                           |spot_msgs/srv/GraphNavUploadGraph                 |    | 
|/initialize_lens                                  |spot_msgs/srv/InitializeLens                      |    | 
|/list_all_dances                                  |spot_msgs/srv/ListAllDances                       |    | 
|/list_all_moves                                   |spot_msgs/srv/ListAllMoves                        |    | 
|/list_cameras                                     |spot_msgs/srv/ListCameras                         |    | 
|/list_graph                                       |spot_msgs/srv/ListGraph                           |    | 
|/list_logpoints                                   |spot_msgs/srv/ListLogpoints                       |    | 
|/list_ptz                                         |spot_msgs/srv/ListPtz                             |    | 
|/list_sounds                                      |spot_msgs/srv/ListSounds                          |    | 
|/list_world_objects                               |spot_msgs/srv/ListWorldObjects                    |    | 
|/load_sound                                       |spot_msgs/srv/LoadSound                           |    | 
|/locomotion_mode                                  |spot_msgs/srv/SetLocomotion                       |    | 
|/max_velocity                                     |spot_msgs/srv/SetVelocity                         |    | 
|/play_sound                                       |spot_msgs/srv/PlaySound                           |    | 
|/power_off                                        |std_srvs/srv/Trigger                              |    | 
|/power_on                                         |std_srvs/srv/Trigger                              |    | 
|/recorded_state_to_animation                      |spot_msgs/srv/ChoreographyRecordedStateToAnimation|    | 
|/release                                          |std_srvs/srv/Trigger                              |    | 
|/retrieve_logpoint                                |spot_msgs/srv/RetrieveLogpoint                    |    | 
|/rollover                                         |std_srvs/srv/Trigger                              |    | 
|/self_right                                       |std_srvs/srv/Trigger                              |    | 
|/set_gripper_camera_parameters                    |spot_msgs/srv/SetGripperCameraParameters          |    | 
|/set_led_brightness                               |spot_msgs/srv/SetLEDBrightness                    |    | 
|/set_ptz_position                                 |spot_msgs/srv/SetPtzPosition                      |    | 
|/set_volume                                       |spot_msgs/srv/SetVolume                           |    | 
|/sit                                              |std_srvs/srv/Trigger                              |    | 
|/stair_mode                                       |std_srvs/srv/SetBool                              |    | 
|/stand                                            |std_srvs/srv/Trigger                              |    | 
|/start_recording_state                            |spot_msgs/srv/ChoreographyStartRecordingState     |    | 
|/stop                                             |std_srvs/srv/Trigger                              |    | 
|/stop_recording_state                             |spot_msgs/srv/ChoreographyStopRecordingState      |    | 
|/store_logpoint                                   |spot_msgs/srv/StoreLogpoint                       |    | 
|/tag_logpoint                                     |spot_msgs/srv/TagLogpoint                         |    | 
|/take_lease                                       |std_srvs/srv/Trigger                              |    | 
|/undock                                           |std_srvs/srv/Trigger                              |    | 
|/upload_animation                                 |spot_msgs/srv/UploadAnimation                     |    | 


### Actions
|Interface Name                                    |Type                                              |Description                                       |
|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|
|/manipulation                                     |spot_msgs/action/Manipulation                     |    | 
|/navigate_to                                      |spot_msgs/action/NavigateTo                       |    | 
|/robot_command                                    |spot_msgs/action/RobotCommand                     |    | 
|/trajectory                                       |spot_msgs/action/Trajectory                       |    | 


## Other Interfaces
