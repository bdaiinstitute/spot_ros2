# nogo_regions

This uses ROS 2 to set no-go regions for Spot via the `mutate_world_objects` service from the Spot driver. It demonstrates how to add and then delete a custom world object with a no-go region via this service. Optionally, you can also test how Spot walks when there is a no-go region present vs when the space is cleared. 

## Running the Example
1.  Make sure the robot is in an open space with at least 2m of space in front of it if you plan to test that Spot will avoid the no-go region.
2.  Build and source the ROS 2 workspace.
    ```bash
    cd <ros2 workspace>
    colcon build
    source install/local_setup.bash
    ```

3.  Define the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP` appropriately for your robot, or put the login credentials into a configuration file yaml ([example](../../spot_driver/config/spot_ros_example.yaml))

4.  Start the driver:
```bash
ros2 launch spot_driver spot_driver.launch.py
```
* If you are defining your login credentials in a configuration yaml instead of as environment variables, add the launch argument `config_file:=path/to/config.yaml`
* If you want to launch with a namespace, use the launch argument `spot_name:=<spot_name>`

5.  Run the example
```bash
ros2 run nogo_regions nogo_example
```
* If you launched the driver with a namespace, add the argument ` --robot <spot_name>`
* By default, this node will not move the robot. If you want to test that Spot will not walk through the no-go region, add the argument `--walk True`.

## Understanding the Code

Listed below are the high level steps that this example implements. 

1. List the current world objects that Spot sees. 
2. Add a custom world object in front of Spot using the `mutate_world_objects` service that contains a no-go region corresponding to just in front of the current robot position.
3. List the current world objects again. There should be a new world object this time that corresponds to the one that was just added.
4. If run with the `--walk True` argument, attempt to walk Spot forward. Spot should not be able to move forward it would involve going through the no-go region. 
5. Delete the custom world object that was just added, again using the `mutate_world_objects` service. 
6. List the current world objects. This should look the same as the initial list, and the custom world object should be gone. 
7. If run with the `--walk True` argument, again attempt to walk Spot forward. This time, Spot should reach the desired end goal. 
