
## Optional Automatic Eye-in-Hand Stereo Calibration Routine for Manipulator (Arm) Payload
#### Collect Calibration
An optional custom Automatic Eye-in-Hand Stereo Calibration Routine for the arm is available for use in the ```spot_wrapper``` submodule, where the
output results can be used with ROS 2 for improved Depth to RGB correspondence for the hand cameras.
See the readme at [```/spot_wrapper/spot_wrapper/calibration/README.md```](https://github.com/bdaiinstitute/spot_wrapper/tree/main/spot_wrapper/calibration/README.md) for 
target setup and relevant information.

First, collect a calibration with ```spot_wrapper/spot_wrapper/calibrate_spot_hand_camera_cli.py```.
Make sure to use the default ```--stereo_pairs``` configuration, and the default tag configuration (```--tag default```).

For the robot and target setup described in [```/spot_wrapper/spot_wrapper/calibration/README.md```](https://github.com/bdaiinstitute/spot_wrapper/tree/main/spot_wrapper/calibration/README.md), the default viewpoint ranges should suffice.

```
python3 spot_wrapper/spot_wrapper/calibrate_spot_hand_camera_cli.py --ip <IP> -u user -pw <SECRET> --data_path ~/my_collection/ \
--save_data --result_path ~/my_collection/calibrated.yaml --photo_utilization_ratio 1 --stereo_pairs "[(1,0)]" \
--spot_rgb_photo_width=640 --spot_rgb_photo_height=480 --tag default --legacy_charuco_pattern True
```

Then, you can run a publisher to transform the depth image into the rgb images frame with the same image
dimensions, so that finding the 3D location of a feature found in rgb can be as easy as passing
the image feature pixel coordinates to the registered depth image, and extracting the 3D location.
For all possible command line arguments, run ```ros2 run spot_driver calibated_reregistered_hand_camera_depth_publisher.py -h```
  
#### Run the Calibrated Re-Publisher
```
ros2 run spot_driver calibrated_reregistered_hand_camera_depth_publisher.py --tag=default --calibration_path <SAVED_CAL> --robot_name <ROBOT_NAMESPACE> --topic depth_registered/hand_custom_cal/image
```

You can treat the reregistered topic, (in the above example, ```<ROBOT_NAME>/depth_registered/hand_custom_cal/image```)
as a drop in replacement by the registered image published by the default spot driver
(```<ROBOT_NAME>/depth_registered/hand/image```). The registered depth can be easily used in tools 
like downstream, like Open3d, (see [creating RGBD Images](https://www.open3d.org/docs/release/python_api/open3d.geometry.RGBDImage.html) and [creating color point clouds from RGBD Images](https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.create_from_rgbd_image)), due to matching image dimensions and registration
to a shared frame.

#### Comparing Calibration Results Quick Sanity Check
You can compare the new calibration to the old calibration through comparing visualizing 
the colored point cloud from a bag in RViz. See RViz setup below the bagging instructions.


First, collect a bag where there is a an object of a clearly different color in the foreground then
that of the background.

```
ROBOT_NAME=<ROBOT_NAME> && \ 
ros2 bag record --output drop_in_test --topics /tf /tf_static \
/${ROBOT_NAME}/depth/hand/image /${ROBOT_NAME}/camera/hand/camera_info \
/${ROBOT_NAME}/joint_states /${ROBOT_NAME}/camera/hand/image \
/${ROBOT_NAME}/depth_registered/hand/image 
```

To see what the default calibration looks like:
```
# In seperate terminals

ros2 bag play drop_in_test --loop
ROBOT_NAME=<ROBOT_NAME> && \
ros2 launch spot_driver point_cloud_xyzrgb.launch.py spot_name:=${ROBOT_NAME} camera:=hand
```

To see what the new calibration looks like:
```
# In seperate terminals
ROBOT_NAME=<ROBOT_NAME> && \
ros2 bag play drop_in_test --loop --topics /${ROBOT_NAME}/depth/hand/image \
/${ROBOT_NAME}/camera/hand/camera_info /${ROBOT_NAME}/joint_states \
/${ROBOT_NAME}/camera/hand/image /tf /tf_static

ROBOT_NAME=<ROBOT_NAME> && \
CALIBRATION_PATH=<CALIBRATION_PATH> && \
ros2 run spot_driver calibrated_reregistered_hand_camera_depth_publisher.py --robot_name ${ROBOT_NAME} \
--calibration_path ${CALIBRATION_PATH} --topic depth_registered/hand/image

ROBOT_NAME=<ROBOT_NAME> && \
ros2 launch spot_driver point_cloud_xyzrgb.launch.py spot_name:=${ROBOT_NAME} camera:=hand
```

#### RVIZ Setup for Sanity Check:
Set global frame to be ```/<ROBOT_NAME>/hand```

Add (bottom left) -> by topic ->
```/<ROBOT_NAME>/depth_registered/hand/points``` -> ok

On the left pane, expand the PointCloud2 message. Expand Topic. Set History
Policy to be Keep Last, Reliability Policy to be Best Effort, and Durability policy to be
Volatile (select these from the dropdowns).
