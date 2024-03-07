# IK solutions service

## Introduction

In this example, we will explore the utilization of ROS 2 for deriving Inverse Kinematics (IK) solutions for specified tool poses. The process involves the following steps:

**Goal Visualization:** For each requested tool pose, the system publishes a Tranform (TF) to RViz, to clearly visualize the goal.

**Solution Identification:** When the system successfully computes an IK solution for a given tool pose, the Transform (TF) is broadcasted to RViz tagged with a "YES" label.

**No Solution Handling:** In scenarios where the IK solution is not found, the TF is still published to RViz but is distinctly tagged with a "NO" label.

**Robot Movement Execution:** Upon finding a viable IK solution, the robot is commanded to relocate the tool to the requested pose.

**Alternative Strategy for Unsolvable Positions:** If an IK solution is unachievable, the system adopts a contingency strategy. It instructs the robot to approximate the target position as closely as feasible. This is done by allowing modifications in the robot's yaw (rotational movement) and height, thus optimizing the robot's position relative to the desired pose.

This approach ensures a robust and adaptive response to varying tool pose requests, enhancing the flexibility and efficiency of the robotic system in dynamic environments.

The given Python script will:
- power on and stand up Spot;
- search and test IK solutions for random tool poses;

## Running the Example

Make sure there is approximately 1 meter free around the robot in all directions. Once the Spot driver is running, you can start the example with:
```bash
ros2 run spot_examples send_inverse_kinematics_requests --robot spot_name --poses n
```
For example:
```bash
ros2 run spot_examples send_inverse_kinematics_requests --robot Opal --poses 10
```

The following parameter is mandatory.

```
--robot robot_name
    The name of the Spot robot to use, e.g. "Opal".
```
The following parameter is optional.

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
    "program": "${workspaceFolder}/ws/src/external/spot_ros2/spot_examples/spot_examples/send_inverse_kinematics_requests.py",
    "args": [
        "--robot",
        "Opal",
        "--poses",
        "10"
    ],
    "console": "integratedTerminal",
    "justMyCode": true
}
```

## Converting Direct API Calls to ROS 2

While the Spot driver provides plenty of helper services and topics, direct Spot API calls can usually be replaced by ROS 2 calls. This is done by replacing the protobuf messages with ROS messages and using the `robot_command` action.

The [ROS 2 example](../spot_examples/send_inverse_kinematics_requests.py) is closely related to the [Spot SDK example](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/inverse_kinematics/reachability.py) which is documented [here](https://dev.bostondynamics.com/python/examples/inverse_kinematics/readme).

The ROS 2 example publishes realtime TFs to RViz instead of plotting the result.

The commands to claim, power on and stand up Spot are already well described in the example [Simple Arm Motion](simple_arm_motion/README.md). We will focus on the command to request an IK solution for a tool pose.

Using the Spot SDK, an IK request can be written as follows:

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

In ROS 2, we convert the created protobuf ROS 2 request and call a ROS 2 service.

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
        convert(ik_request, request.request)
        ik_reponse = self._ik_client.call(request)

        proto = inverse_kinematics_pb2.InverseKinematicsResponse()
        convert(ik_reponse.response, proto)
        return proto
```
