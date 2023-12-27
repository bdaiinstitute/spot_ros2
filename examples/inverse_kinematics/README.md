# IK solutions service

## Introduction

In this example, we will explore the utilization of ROS2 for deriving Inverse Kinematics (IK) solutions for specified tool poses. The process involves the following steps:

**Goal Visualization:** For each requested tool pose, the system publishes a Tranform (TF) to RViz, to clearly visualize the goal.

**Solution Identification:** When the system successfully computes an IK solution for a given tool pose, the Transform (TF) is broadcasted to RViz tagged with a "YES" label.

**No Solution Handling:** In scenarios where the IK solution is not found, the TF is still published to RViz but is distinctly tagged with a "NO" label.

**Robot Movement Execution:** Upon finding a viable IK solution, the robot is commanded to relocate the tool to the requested pose.

**Alternative Strategy for Unsolvable Positions:** If an IK solution is unachievable, the system adopts a contingency strategy. It instructs the robot to approximate the target position as closely as feasible. This is done by allowing modifications in the robot's yaw (rotational movement) and height, thus optimizing the robot's position relative to the desired pose.

This approach ensures a robust and adaptive response to varying tool pose requests, enhancing the flexibility and efficiency of the robotic system in dynamic environments.

The given Python script will:
- undock Spot;
- move it forward approximatively 1m;
- rotate it 90 degrees counter-clockwise;
- search and test IK solutions for random tool poses;
- dock Spot back to the given docking station.

## Running the Example

1. Make sure the robot is docked, and there is approximately 2 meters free in front of the docking station.

2. Make sure you've built and sourced your workspace:
   ```bash
   cd <ros2 workspace>
   colcon build --symlink-install
   source /opt/ros/humble/setup.bash
   source ./install/local_setup.bash
   ```
3.  Start the driver:
    ```bash
    bdai spot driver spot_name --launch-rviz
    ```
4.  Run the script:
    ```bash
    ros2 run inverse_kinematics send_inverse_kinematics_requests --robot spot_name --dock docking_station_id --poses n
    ```
    For example:
    ```bash
    ros2 run inverse_kinematics send_inverse_kinematics_requests --robot Opal --dock 527 --poses 10
    ```

    The following parameters are mandatory:

    ```
    --robot robot_name
      The name of the Spot robot to use, e.g. "Opal".

    --dock docking_station_id
      The docking station where spot is docked, e.g. 527.
    ```
    The following parameter is optional:

    ```
    --poses n
      The number of tool poses to check (default 1).
    ```

    The same command can be run/debugged in VSCode by adding the following to your launch configuration file `launch.json`.

    ```json
    {
        "name": "send_inverse_kinematics_requests",
        "type": "python",
        "request": "launch",
        "program": "${workspaceFolder}/ws/src/external/spot_ros2/examples/inverse_kinematics/inverse_kinematics/send_inverse_kinematics_requests.py",
        "args": [
            "--robot",
            "Opal",
            "--dock",
            "527",
            "--poses",
            "10"
        ],
        "console": "integratedTerminal",
        "justMyCode": true
    }
    ```

## Converting Direct API Calls to ROS2

While the Spot driver provides plenty of helper services and topics, direct Spot API calls can usually be replaced by ROS2 calls. This is done by replacing the protobuf messages with ROS messages and using the `robot_command` action.

The [ROS2 example](inverse_kinematics/send_inverse_kinematics_requests.py) is closely related to the [Spot SDK example](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/inverse_kinematics/reachability.py) which is documented [here](https://dev.bostondynamics.com/python/examples/inverse_kinematics/readme).

The ROS2 example publishes realtime TFs to RViz instead of plotting the result. It also executes some additional steps like undocking the robot, moving it to a specified location, and docking it back.

The first substantive lines of the original code are:

```python
def reachability_queries(config):
    """
    Making reachability requests using the InverseKinematics service
    """

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk("ReachabilitySDK")
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
```

If you want to ensure you only communicate with the robot via the ROS2 driver, the best way to do so is to ensure that you never call `authenticate` in any other program.  That is done by the Spot driver, which should be the only piece of code that communicates with the robot.  All other programs communicate with the Spot driver via ROS2. Therefore, we replace the pieces of code that talks directly to Spot with their ROS counterparts:

```python
    def __init__(self, node: Node, args: argparse.Namespace):
        self._node = node
        self._robot_name: str = args.robot
        self._dock_id: int = args.dock
        self._poses: int = args.poses
        self._logger = node.get_logger()
        self._tf_broadcaster = TransformBroadcaster(node)
        self._tf_listener = TFListenerWrapper(node)
        self._robot = SimpleSpotCommander(self._robot_name)

        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )

        self._timer = node.create_timer(0.1, self._timer_callback)
        self._transforms: List[geometry_msgs.msg.TransformStamped] = []

        self._dock_client = node.create_client(Dock, namespace_with(self._robot_name, "dock"))
        self._ik_client = node.create_client(
            GetInverseKinematicSolutions,
            namespace_with(self._robot_name, "get_inverse_kinematic_solutions"),
        )
```

This gives us four components, which we'll use in many ROS2 programs:

* **A node:** [ROS2 nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) are the objects that interact with [ROS2 topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) and almost all ROS programs require them. Here we fetch the one in scope, already instantiated and serviced in the background. Had we instantiated a node of our own, we would have had to spin it ourselves.

* **A TF listener:** This handles computing transforms.  As we'll see later, it can be used in place of Spot API `RobotStateClient` to get information about where frames on the robot are.  For more information about ROS2 TF see [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html). For more information about our [TF wrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/bdai_ros2_wrappers/bdai_ros2_wrappers/tf_listener_wrapper.py) and how we use it in these examples, see the [simple_walk_forward example](../simple_walk_forward/).

* **A spot commander:** This is a [wrapper](../utilities/utilities/simple_spot_commander.py) around service clients that calls the Spot driver to do simple things like get the lease and stand.  This is used in place of calls like `blocking_stand`.

* **A robot command action client:** This is the ROS2 action client that sends goals to the ROS2 action server (for more information about ROS2 actions see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)).  This is used in place of the Spot API `RobotCommandClient`.  We use a [wrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/bdai_ros2_wrappers/bdai_ros2_wrappers/action_client.py) around the built in ROS2 action client that allows us to wait for the goal to return without risk of deadlock.

The original code uses direct calls to the API to power on and stand the robot:

```python
    # Now, we are ready to power on the robot.
    # This call will block until the power is on. 
    # Commands would fail if this did not happen.
    # We can also check that the robot is powered at any point.
    robot.logger.info("Powering on robot... This may take a several seconds.")
    robot.power_on(timeout_sec=20)
    assert robot.is_powered_on(), "Robot power on failed."
    robot.logger.info("Robot powered on.")

    # The command service is used to issue commands to a robot.
    # The set of valid commands for a robot depends on hardware configuration. 
    # See RobotCommandBuilder for more detailed examples on command building.
    # The robot command service requires timesync between the robot and the client.
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    # Tell the robot to stand up
    robot.logger.info("Commanding robot to stand...")
    blocking_stand(command_client, timeout_sec=10)
    robot.logger.info("Robot standing.")
```

The ROS2 code uses service calls to do the same thing:

```python
    # Claim robot
    self._logger.info("Claiming robot")
    result = self._robot.command("claim")
    if not result:
        self._logger.error("Unable to claim robot")
        return False
    self._logger.info("Claimed robot")

    # Power on robot.
    self._logger.info("Powering robot on")
    result = self._robot.command("power_on")
    if not result:
        self._logger.error("Unable to power on robot")
        return False

    # Stand up robot.
    self._logger.info("Standing robot up")
    result = self._robot.command("stand")
    if not result:
        self._logger.error("Robot did not stand")
        return False
    self._logger.info("Successfully stood up.")
```

Note that we also use ROS's text logger instead of the Spot API logger.

Once the robot is initialized and standing, the next change to the code comes when we need to get the robot's current position. With direct API calls, this uses the robot state:

```python
    # Define the task frame to be in front of the robot and near the ground
    robot_state = robot_state_client.get_robot_state()

    odom_T_flat_body = get_a_tform_b(
        robot_state.kinematic_state.transforms_snapshot,
        ODOM_FRAME_NAME,
        GRAV_ALIGNED_BODY_FRAME_NAME,
    )
```
With ROS2, we use TF:

```python
    # Look for known transforms published by the robot.
    odom_T_flat_body: SE3Pose = self._tf_listener.lookup_a_tform_b(odom_frame_name, flat_body_frame_name)

```

The ROS2 example adds some additional instructions to move the robot to a position in front of the docking station and rotate it by 90 degrees counter-clockwise.

The commands are built the same way after this, but we change how we send them. 

With the Spot API:

```python
    # Query the IK service for the reachability of the desired tool pose.
    # Construct the IK request for this reachability problem.
    # Note that since `root_tform_scene` is unset, the "scene" frame is the same as the "root" frame in this case.
    ik_request = InverseKinematicsRequest(
        root_frame_name=ODOM_FRAME_NAME,
        scene_tform_task=odom_T_task.to_proto(),
        wrist_mounted_tool=InverseKinematicsRequest.WristMountedTool(
            wrist_tform_tool=wr1_T_tool.to_proto()
        ),
        tool_pose_task=InverseKinematicsRequest.ToolPoseTask(
            task_tform_desired_tool=task_T_desired_tool.to_proto()
        ),
    )
    ik_responses.append(ik_client.inverse_kinematics(ik_request))
```

In ROS2, we convert the created protobuf ROS2 request and call a ROS2 service.

```python
    def _send_ik_request(
        self, odom_T_task: SE3Pose, wr1_T_tool: SE3Pose, task_T_desired_tool: SE3Pose
    ) -> GetInverseKinematicSolutions.Request:
        """
        Send an IK request for a given task frame, tool and tool desired pose.
        Returns:
            An IK solution, if any.
        """
        ik_request = inverse_kinematics_pb2.InverseKinematicsRequest(
            root_frame_name=ODOM_FRAME_NAME,
            scene_tform_task=odom_T_task.to_proto(),
            wrist_mounted_tool=inverse_kinematics_pb2.InverseKinematicsRequest.WristMountedTool(
                wrist_tform_tool=wr1_T_tool.to_proto()
            ),
            tool_pose_task=inverse_kinematics_pb2.InverseKinematicsRequest.ToolPoseTask(
                task_tform_desired_tool=task_T_desired_tool.to_proto()
            ),
        )
        request = GetInverseKinematicSolutions.Request()
        conv.convert_proto_to_bosdyn_msgs_inverse_kinematics_request(ik_request, request.request)
        ik_reponse = self._ik_client.call(request)

        proto = inverse_kinematics_pb2.InverseKinematicsResponse()
        conv.convert_bosdyn_msgs_inverse_kinematics_response_to_proto(ik_reponse.response, proto)
        return proto
```
