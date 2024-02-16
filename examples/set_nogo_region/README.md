This uses ROS 2 to set "no-go" regions for Spot.

## Running the Example
1.  Make sure the robot is in an open space
2.  Make sure you've built and sourced your workspace:
    ```bash
    cd <ros2 workspace>
    colcon build --symlink-install
    source /opt/ros/humble/setup.bash
    source ./install/local_setup.bash
    ```

3.  Define the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP` appropriately for your robot.

4.  Start the driver:
```bash
ros2 launch spot_driver spot_driver.launch.py
```
If you want to launch with a namespace,
```bash
ros2 launch spot_driver.launch.py spot_name:=<spot_name> 
```

5.  Run the example:
```bash
ros2 run set_nogo_region set_nogo
```
If you are launching spot_ros2 with a namespace, use the following command instead:
```bash
ros2 run set_nogo_region set_nogo --robot <spot_name>
```

## Understanding the Code
Todo
