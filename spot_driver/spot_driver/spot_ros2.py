### Debug
# from ros_helpers import *
import sys
import threading
import time
import traceback
import typing
from dataclasses import dataclass
from enum import Enum
from functools import partial
from typing import Any, Callable, Dict, List, Optional, Union

import builtin_interfaces.msg
import rclpy
import rclpy.time
import tf2_ros
from bdai_ros2_wrappers.single_goal_action_server import (
    SingleGoalActionServer,
)
from bdai_ros2_wrappers.single_goal_multiple_action_servers import (
    SingleGoalMultipleActionServers,
)
from bosdyn.api import (
    geometry_pb2,
    image_pb2,
    manipulation_api_pb2,
    robot_command_pb2,
    trajectory_pb2,
    world_object_pb2,
)
from bosdyn.api.geometry_pb2 import Quaternion, SE2VelocityLimit
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.exceptions import InternalServerError
from bosdyn_msgs.msg import ManipulationApiFeedbackResponse, RobotCommandFeedback
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist, TwistWithCovarianceStamped, Vector3Stamped
from google.protobuf.timestamp_pb2 import Timestamp
from nav_msgs.msg import Odometry
from rclpy import Parameter
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.impl import rcutils_logger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Rate
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import SetBool, Trigger

import spot_driver.conversions as conv
from spot_msgs.action import Manipulation, NavigateTo, RobotCommand, Trajectory  # type: ignore
from spot_msgs.msg import (  # type: ignore
    BatteryStateArray,
    BehaviorFaultState,
    EStopStateArray,
    Feedback,
    FootStateArray,
    LeaseArray,
    LeaseResource,
    Metrics,
    MobilityParams,
    PowerState,
    SystemFaultState,
    WiFiState,
)
from spot_msgs.srv import (  # type: ignore
    ClearBehaviorFault,
    DeleteSound,
    Dock,
    ExecuteDance,
    GetVolume,
    GraphNavClearGraph,
    GraphNavGetLocalizationPose,
    GraphNavSetLocalization,
    GraphNavUploadGraph,
    ListAllDances,
    ListAllMoves,
    ListGraph,
    ListSounds,
    ListWorldObjects,
    LoadSound,
    PlaySound,
    SetLocomotion,
    SetVelocity,
    SetVolume,
    UploadAnimation,
)
from spot_wrapper.cam_wrapper import SpotCamWrapper
from spot_wrapper.spot_images import CameraSource
from spot_wrapper.wrapper import SpotWrapper

#####DEBUG/RELEASE: RELATIVE PATH NOT WORKING IN DEBUG
# Release
from .ros_helpers import (
    bosdyn_data_to_image_and_camera_info_msgs,
    get_battery_states_from_state,
    get_behavior_faults_from_state,
    get_end_effector_force_from_state,
    get_estop_state_from_state,
    get_feet_from_state,
    get_from_env_and_fall_back_to_param,
    get_joint_states_from_state,
    get_odom_from_state,
    get_odom_twist_from_state,
    get_power_states_from_state,
    get_system_faults_from_state,
    get_tf_from_state,
    get_tf_from_world_objects,
    get_wifi_from_state,
    populate_transform_stamped,
)

MAX_DURATION = 1e6
MOCK_HOSTNAME = "Mock_spot"
COLOR_END = "\33[0m"
COLOR_GREEN = "\33[32m"
COLOR_YELLOW = "\33[33m"


@dataclass
class Request:
    id: str
    data: Any


@dataclass
class Response:
    message: str
    success: bool


class GoalResponse(Enum):
    FAILED = -1
    IN_PROGRESS = False
    SUCCESS = True


class WaitForGoal(object):
    def __init__(self, clock: Clock, _time: Union[float, rclpy.time.Time], callback: Optional[Callable] = None) -> None:
        self._at_goal: bool = False
        self._callback: Optional[Callable] = callback
        self._clock: Clock = clock
        self._time: rclpy.time.Duration = rclpy.time.Duration(seconds=_time)
        self._thread: threading.Thread = threading.Thread(target=self._run)
        self._thread.start()

    @property
    def at_goal(self) -> bool:
        return self._at_goal

    def _run(self) -> None:
        start_time = self._clock.now()
        while self._clock.now() - start_time < self._time:
            time.sleep(0.05)
        self._at_goal = True


class SpotImageType(str, Enum):
    RGB = "visual"
    Depth = "depth"
    RegDepth = "depth_registered"


class SpotROS(Node):
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self, parameter_list: Optional[typing.List[Parameter]] = None) -> None:
        """
        Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.
        Holds lease from wrapper and updates all async tasks at the ROS rate
        """
        super().__init__("spot_ros2")
        self.run_navigate_to: Optional[bool] = None
        self._printed_once: bool = False

        self.get_logger().info(COLOR_GREEN + "Hi from spot_driver." + COLOR_END)

        self.callbacks: Dict[str, Callable] = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["robot_state"] = self.robot_state_callback
        self.callbacks["metrics"] = self.metrics_callback
        self.callbacks["lease"] = self.lease_callback
        self.callbacks["world_objects"] = self.world_objects_callback

        self.group: CallbackGroup = ReentrantCallbackGroup()
        self.rgb_callback_group: CallbackGroup = MutuallyExclusiveCallbackGroup()
        self.depth_callback_group: CallbackGroup = MutuallyExclusiveCallbackGroup()
        self.depth_registered_callback_group: CallbackGroup = MutuallyExclusiveCallbackGroup()
        self.graph_nav_callback_group: CallbackGroup = MutuallyExclusiveCallbackGroup()
        rate = self.create_rate(100)
        self.node_rate: Rate = rate

        # spot_ros.yaml
        self.rates = {
            "robot_state": 50.0,
            "metrics": 0.04,
            "lease": 1.0,
            "world_objects": 20.0,
            "front_image": 10.0,
            "side_image": 10.0,
            "rear_image": 10.0,
            "graph_nav_pose": 10.0,
        }
        max_task_rate = float(max(self.rates.values()))

        self.declare_parameter("auto_claim", False)
        self.declare_parameter("auto_power_on", False)
        self.declare_parameter("auto_stand", False)

        self.declare_parameter("use_take_lease", True)
        self.declare_parameter("get_lease_on_action", True)
        self.declare_parameter("continually_try_stand", False)

        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("estop_timeout", 9.0)
        self.declare_parameter("async_tasks_rate", max_task_rate)
        self.declare_parameter("cmd_duration", 0.125)
        self.declare_parameter("start_estop", False)
        self.declare_parameter("publish_rgb", True)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("publish_depth_registered", False)
        self.declare_parameter("rgb_cameras", True)

        self.declare_parameter("publish_graph_nav_pose", False)
        self.declare_parameter("graph_nav_seed_frame", "graph_nav_map")

        self.declare_parameter("spot_name", "")

        # used for setting when not using launch file
        if parameter_list is not None:
            self.set_parameters(parameter_list)

        self.auto_claim: Parameter = self.get_parameter("auto_claim")
        self.auto_power_on: Parameter = self.get_parameter("auto_power_on")
        self.auto_stand: Parameter = self.get_parameter("auto_stand")
        self.start_estop: Parameter = self.get_parameter("start_estop")

        self.use_take_lease: Parameter = self.get_parameter("use_take_lease")
        self.get_lease_on_action: Parameter = self.get_parameter("get_lease_on_action")
        self.continually_try_stand: Parameter = self.get_parameter("continually_try_stand")

        self.publish_rgb: Parameter = self.get_parameter("publish_rgb")
        self.publish_depth: Parameter = self.get_parameter("publish_depth")
        self.publish_depth_registered: Parameter = self.get_parameter("publish_depth_registered")
        self.rgb_cameras: Parameter = self.get_parameter("rgb_cameras")

        self.publish_graph_nav_pose: Parameter = self.get_parameter("publish_graph_nav_pose")
        self.graph_nav_seed_frame: str = self.get_parameter("graph_nav_seed_frame").value

        self._wait_for_goal: Optional[WaitForGoal] = None
        self.goal_handle: Optional[ServerGoalHandle] = None

        # This is only done from parameter because it should be passed by the launch file
        self.name: Optional[str] = self.get_parameter("spot_name").value
        if not self.name:
            self.name = None

        self.motion_deadzone: Parameter = self.get_parameter("deadzone")
        self.estop_timeout: Parameter = self.get_parameter("estop_timeout")
        self.async_tasks_rate: float = self.get_parameter("async_tasks_rate").value
        if self.async_tasks_rate < max_task_rate:
            self.get_logger().warn(
                COLOR_YELLOW
                + f"The maximum individual task rate is {max_task_rate} Hz. You have manually set the async_tasks_rate"
                f" to {self.async_tasks_rate} which is lower and will decrease the frequency of one of the periodic"
                " tasks being run."
                + COLOR_END
            )

        self.cmd_duration: float = self.get_parameter("cmd_duration").value

        self.username: Optional[str] = get_from_env_and_fall_back_to_param(
            "BOSDYN_CLIENT_USERNAME", self, "username", "user"
        )
        self.password: Optional[str] = get_from_env_and_fall_back_to_param(
            "BOSDYN_CLIENT_PASSWORD", self, "password", "password"
        )
        self.ip: Optional[str] = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")

        self.camera_static_transform_broadcaster: tf2_ros.StaticTransformBroadcaster = (
            tf2_ros.StaticTransformBroadcaster(self)
        )
        # Static transform broadcaster is super simple and just a latched publisher. Every time we add a new static
        # transform we must republish all static transforms from this source, otherwise the tree will be incomplete.
        # We keep a list of all the static transforms we already have, so they can be republished, and so we can check
        # which ones we already have
        self.camera_static_transforms: List[TransformStamped] = []

        # Spot has 2 types of odometries: 'odom' and 'vision'
        # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
        # These params enables to change which odometry frame is a parent of body frame and to change tf names of each
        # odometry frames.
        frame_prefix = ""
        if self.name is not None:
            frame_prefix = self.name + "/"
        self.frame_prefix: str = frame_prefix
        self.preferred_odom_frame: Parameter = self.declare_parameter(
            "preferred_odom_frame", frame_prefix + "odom"
        )  # 'vision' or 'odom'
        self.tf_name_kinematic_odom: Parameter = self.declare_parameter("tf_name_kinematic_odom", frame_prefix + "odom")
        self.tf_name_raw_kinematic: str = frame_prefix + "odom"
        self.tf_name_vision_odom: Parameter = self.declare_parameter("tf_name_vision_odom", frame_prefix + "vision")
        self.tf_name_raw_vision: str = frame_prefix + "vision"

        if (
            self.preferred_odom_frame.value != self.tf_name_raw_kinematic
            and self.preferred_odom_frame.value != self.tf_name_raw_vision
        ):
            error_msg = f'rosparam "preferred_odom_frame" should be "{frame_prefix}odom" or "{frame_prefix}vision".'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)

        self.tf_name_graph_nav_body: str = frame_prefix + "body"

        # logger for spot wrapper
        name_with_dot = ""
        if self.name is not None:
            name_with_dot = self.name + "."
        self.wrapper_logger = rcutils_logger.RcutilsLogger(name=f"{name_with_dot}spot_wrapper")
        self.cam_logger = rcutils_logger.RcutilsLogger(name=f"{name_with_dot}spot_cam_wrapper")

        name_str = ""
        if self.name is not None:
            name_str = " for " + self.name
        self.get_logger().info("Starting ROS driver for Spot" + name_str)
        ############## testing with Robot

        if self.name == MOCK_HOSTNAME:
            self.spot_wrapper: Optional[SpotWrapper] = None
            self.cam_wrapper: Optional[SpotCamWrapper] = None
        else:
            # create SpotWrapper if not mocking
            self.spot_wrapper = SpotWrapper(
                self.username,
                self.password,
                self.ip,
                self.name,
                self.wrapper_logger,
                self.start_estop.value,
                self.estop_timeout.value,
                self.rates,
                self.callbacks,
                self.use_take_lease.value,
                self.get_lease_on_action.value,
                self.continually_try_stand.value,
                self.rgb_cameras.value,
            )
            if not self.spot_wrapper.is_valid:
                return

            try:
                self.spot_cam_wrapper = SpotCamWrapper(self.ip, self.username, self.password, self.cam_logger)
            except SystemError:
                self.spot_cam_wrapper = None

        all_cameras = ["frontleft", "frontright", "left", "right", "back"]
        has_arm = False
        if self.spot_wrapper is not None:
            has_arm = self.spot_wrapper.has_arm()
        if has_arm:
            all_cameras.append("hand")
        self.declare_parameter("cameras_used", all_cameras)
        self.cameras_used = self.get_parameter("cameras_used")

        # Create the necessary publishers and timers
        # if enable set up publisher for rgb images
        if self.publish_rgb.value:
            self.create_image_publisher(SpotImageType.RGB, self.rgb_callback_group)
        # if enabled set up publisher for depth images
        if self.publish_depth.value:
            self.create_image_publisher(SpotImageType.Depth, self.depth_callback_group)
        # if enable publish registered depth
        if self.publish_depth_registered.value:
            self.create_image_publisher(SpotImageType.RegDepth, self.depth_registered_callback_group)

        if self.publish_graph_nav_pose.value:
            # graph nav pose will be published both on a topic
            # and as a TF transform from graph_nav_map to body.
            self.graph_nav_pose_pub = self.create_publisher(PoseStamped, "graph_nav/body_pose", 1)
            self.graph_nav_pose_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

            self.create_timer(
                1 / self.rates["graph_nav_pose"],
                self.publish_graph_nav_pose_callback,
                callback_group=self.graph_nav_callback_group,
            )

        self.declare_parameter("has_arm", has_arm)

        # Status Publishers #
        self.joint_state_pub: Publisher = self.create_publisher(JointState, "joint_states", 1)
        self.dynamic_broadcaster: tf2_ros.TransformBroadcaster = tf2_ros.TransformBroadcaster(self)
        self.metrics_pub: Publisher = self.create_publisher(Metrics, "status/metrics", 1)
        self.lease_pub: Publisher = self.create_publisher(LeaseArray, "status/leases", 1)
        self.odom_twist_pub: Publisher = self.create_publisher(TwistWithCovarianceStamped, "odometry/twist", 1)
        self.odom_pub: Publisher = self.create_publisher(Odometry, "odometry", 1)
        self.feet_pub: Publisher = self.create_publisher(FootStateArray, "status/feet", 1)
        self.estop_pub: Publisher = self.create_publisher(EStopStateArray, "status/estop", 1)
        self.wifi_pub: Publisher = self.create_publisher(WiFiState, "status/wifi", 1)
        self.power_pub: Publisher = self.create_publisher(PowerState, "status/power_state", 1)
        self.battery_pub: Publisher = self.create_publisher(BatteryStateArray, "status/battery_states", 1)
        self.behavior_faults_pub: Publisher = self.create_publisher(BehaviorFaultState, "status/behavior_faults", 1)
        self.system_faults_pub: Publisher = self.create_publisher(SystemFaultState, "status/system_faults", 1)
        self.feedback_pub: Publisher = self.create_publisher(Feedback, "status/feedback", 1)
        self.mobility_params_pub: Publisher = self.create_publisher(MobilityParams, "status/mobility_params", 1)
        if has_arm:
            self.end_effector_force_pub: Publisher = self.create_publisher(
                Vector3Stamped, "status/end_effector_force", 1
            )

        self.create_subscription(Twist, "cmd_vel", self.cmd_velocity_callback, 1, callback_group=self.group)
        self.create_subscription(Pose, "body_pose", self.body_pose_callback, 1, callback_group=self.group)
        self.create_service(
            Trigger,
            "claim",
            lambda request, response: self.service_wrapper("claim", self.handle_claim, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "release",
            lambda request, response: self.service_wrapper("release", self.handle_release, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "stop",
            lambda request, response: self.service_wrapper("stop", self.handle_stop, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "self_right",
            lambda request, response: self.service_wrapper("self_right", self.handle_self_right, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "sit",
            lambda request, response: self.service_wrapper("sit", self.handle_sit, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "stand",
            lambda request, response: self.service_wrapper("stand", self.handle_stand, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "rollover",
            lambda request, response: self.service_wrapper("rollover", self.handle_rollover, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "power_on",
            lambda request, response: self.service_wrapper("power_on", self.handle_power_on, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "power_off",
            lambda request, response: self.service_wrapper("power_off", self.handle_safe_power_off, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "estop/hard",
            lambda request, response: self.service_wrapper("estop/hard", self.handle_estop_hard, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "estop/gentle",
            lambda request, response: self.service_wrapper("estop/gentle", self.handle_estop_soft, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "estop/release",
            lambda request, response: self.service_wrapper(
                "estop/release", self.handle_estop_disengage, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "undock",
            lambda request, response: self.service_wrapper("undock", self.handle_undock, request, response),
            callback_group=self.group,
        )

        self.create_service(
            SetBool,
            "stair_mode",
            lambda request, response: self.service_wrapper("stair_mode", self.handle_stair_mode, request, response),
            callback_group=self.group,
        )
        self.create_service(
            SetLocomotion,
            "locomotion_mode",
            lambda request, response: self.service_wrapper(
                "locomotion_mode", self.handle_locomotion_mode, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            SetVelocity,
            "max_velocity",
            lambda request, response: self.service_wrapper("max_velocity", self.handle_max_vel, request, response),
            callback_group=self.group,
        )
        self.create_service(
            ClearBehaviorFault,
            "clear_behavior_fault",
            lambda request, response: self.service_wrapper(
                "clear_behavior_fault", self.handle_clear_behavior_fault, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ExecuteDance,
            "execute_dance",
            lambda request, response: self.service_wrapper(
                "execute_dance", self.handle_execute_dance, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            UploadAnimation,
            "upload_animation",
            lambda request, response: self.service_wrapper(
                "upload_animation", self.handle_upload_animation, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListAllDances,
            "list_all_dances",
            lambda request, response: self.service_wrapper(
                "list_all_dances", self.handle_list_all_dances, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListAllMoves,
            "list_all_moves",
            lambda request, response: self.service_wrapper(
                "list_all_moves", self.handle_list_all_moves, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListSounds,
            "list_sounds",
            lambda request, response: self.service_wrapper("list_sounds", self.handle_list_sounds, request, response),
            callback_group=self.group,
        )
        self.create_service(
            LoadSound,
            "load_sound",
            lambda request, response: self.service_wrapper("load_sound", self.handle_load_sound, request, response),
            callback_group=self.group,
        )
        self.create_service(
            PlaySound,
            "play_sound",
            lambda request, response: self.service_wrapper("play_sound", self.handle_play_sound, request, response),
            callback_group=self.group,
        )
        self.create_service(
            DeleteSound,
            "delete_sound",
            lambda request, response: self.service_wrapper("delete_sound", self.handle_delete_sound, request, response),
            callback_group=self.group,
        )
        self.create_service(
            GetVolume,
            "get_volume",
            lambda request, response: self.service_wrapper("get_volume", self.handle_get_volume, request, response),
            callback_group=self.group,
        )
        self.create_service(
            SetVolume,
            "set_volume",
            lambda request, response: self.service_wrapper("set_volume", self.handle_set_volume, request, response),
            callback_group=self.group,
        )
        self.create_service(
            ListGraph,
            "list_graph",
            lambda request, response: self.service_wrapper("list_graph", self.handle_list_graph, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Dock,
            "dock",
            lambda request, response: self.service_wrapper("dock", self.handle_dock, request, response),
            callback_group=self.group,
        )

        # This doesn't use the service wrapper because it's not a trigger, and we want different mock responses
        self.create_service(ListWorldObjects, "list_world_objects", self.handle_list_world_objects)

        self.create_service(
            GraphNavUploadGraph,
            "graph_nav_upload_graph",
            self.handle_graph_nav_upload_graph,
            callback_group=self.group,
        )

        self.create_service(
            GraphNavClearGraph,
            "graph_nav_clear_graph",
            self.handle_graph_nav_clear_graph,
            callback_group=self.group,
        )

        self.create_service(
            GraphNavGetLocalizationPose,
            "graph_nav_get_localization_pose",
            self.handle_graph_nav_get_localization_pose,
            callback_group=self.group,
        )

        self.create_service(
            GraphNavSetLocalization,
            "graph_nav_set_localization",
            self.handle_graph_nav_set_localization,
            callback_group=self.group,
        )

        self.navigate_as = ActionServer(
            self, NavigateTo, "navigate_to", self.handle_navigate_to, callback_group=self.group
        )
        # spot_ros.navigate_as.start() # As is online

        self.trajectory_server = ActionServer(
            self, Trajectory, "trajectory", self.handle_trajectory, callback_group=self.group
        )
        # spot_ros.trajectory_server.start()

        if has_arm:
            # Allows both the "robot command" and the "manipulation" action goal to preempt each other
            self.robot_command_and_manipulation_servers = SingleGoalMultipleActionServers(
                self,
                [
                    (
                        RobotCommand,
                        "robot_command",
                        self.handle_robot_command,
                        self.group,
                    ),
                    (
                        Manipulation,
                        "manipulation",
                        self.handle_manipulation_command,
                        self.group,
                    ),
                ],
            )
        else:
            self.robot_command_server = SingleGoalActionServer(
                self,
                RobotCommand,
                "robot_command",
                self.handle_robot_command,
                callback_group=self.group,
            )

        # Register Shutdown Handle
        # rclpy.on_shutdown(spot_ros.shutdown) ############## Shutdown Handle

        # Wait for an estop to be connected
        if self.spot_wrapper is not None and not self.start_estop.value:
            printed = False
            while self.spot_wrapper.is_estopped():
                if not printed:
                    self.get_logger().warn(
                        COLOR_YELLOW
                        + "Waiting for estop to be released.  Make sure you have an active estop."
                        '  You can acquire an estop on the tablet by choosing "Acquire Cut Motor Power Authority"'
                        " in the dropdown menu from the power icon.  (This will not power the motors or take the"
                        " lease.)"
                        + COLOR_END,
                    )
                    printed = True
                time.sleep(0.5)
            self.get_logger().info("Found estop!")

        self.create_timer(1 / self.async_tasks_rate, self.step, callback_group=self.group)

        self.mt_executor = MultiThreadedExecutor(num_threads=8)
        self.mt_executor.add_node(self)

        if self.spot_wrapper is not None and self.auto_claim.value:
            self.spot_wrapper.claim()
            if self.auto_power_on.value:
                self.spot_wrapper.power_on()
                if self.auto_stand.value:
                    self.spot_wrapper.stand()

    def spin(self) -> None:
        self.get_logger().info("Spinning ros2_driver")
        sys.stdout.flush()
        try:
            self.mt_executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

        self.mt_executor.shutdown()

    def robot_state_callback(self, results: Any) -> None:
        """Callback for when the Spot Wrapper gets new robot state data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        if self.spot_wrapper is None:
            return

        state = self.spot_wrapper.robot_state

        if state is not None:
            # Joint states
            joint_state = get_joint_states_from_state(state, self.spot_wrapper)
            if self.joint_state_pub is not None:
                self.joint_state_pub.publish(joint_state)

            # TF
            tf_msg = get_tf_from_state(state, self.spot_wrapper, self.preferred_odom_frame.value)
            if len(tf_msg.transforms) > 0:
                self.dynamic_broadcaster.sendTransform(tf_msg.transforms)

            # Odom Twist #
            twist_odom_msg = get_odom_twist_from_state(state, self.spot_wrapper)
            self.odom_twist_pub.publish(twist_odom_msg)

            # Odom #
            if self.preferred_odom_frame.value == self.spot_wrapper.frame_prefix + "vision":
                odom_msg = get_odom_from_state(state, self.spot_wrapper, use_vision=True)
            else:
                odom_msg = get_odom_from_state(state, self.spot_wrapper, use_vision=False)
            self.odom_pub.publish(odom_msg)

            # Feet #
            foot_array_msg = get_feet_from_state(state, self.spot_wrapper)
            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = get_estop_state_from_state(state, self.spot_wrapper)
            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = get_wifi_from_state(state, self.spot_wrapper)
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = get_battery_states_from_state(state, self.spot_wrapper)
            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = get_power_states_from_state(state, self.spot_wrapper)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = get_system_faults_from_state(state, self.spot_wrapper)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = get_behavior_faults_from_state(state, self.spot_wrapper)
            self.behavior_faults_pub.publish(behavior_fault_state_msg)

            if self.spot_wrapper.has_arm():
                end_effector_force_msg = get_end_effector_force_from_state(state, self.spot_wrapper)
                self.end_effector_force_pub.publish(end_effector_force_msg)

    def metrics_callback(self, results: Any) -> None:
        """Callback for when the Spot Wrapper gets new metrics data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        if self.spot_wrapper is None:
            return

        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            local_time = self.spot_wrapper.robotToLocalTime(metrics.timestamp)
            metrics_msg.header.stamp = builtin_interfaces.msg.Time(sec=local_time.seconds, nanosec=local_time.nanos)

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                if metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                if metric.label == "time moving":
                    # metrics_msg.time_moving = Time(metric.duration.seconds, metric.duration.nanos)
                    duration = builtin_interfaces.msg.Duration(
                        sec=metric.duration.seconds, nanosec=metric.duration.nanos
                    )
                    metrics_msg.time_moving = duration
                if metric.label == "electric power":
                    # metrics_msg.electric_power = Time(metric.duration.seconds, metric.duration.nanos)
                    duration = builtin_interfaces.msg.Duration(
                        sec=metric.duration.seconds, nanosec=metric.duration.nanos
                    )
                    metrics_msg.electric_power = duration
            self.metrics_pub.publish(metrics_msg)

    def lease_callback(self, results: Any) -> None:
        """Callback for when the Spot Wrapper gets new lease data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        if self.spot_wrapper is None:
            return

        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease
        if lease_list:
            for resource in lease_list:
                new_resource = LeaseResource()
                new_resource.resource = resource.resource
                new_resource.lease.resource = resource.lease.resource
                new_resource.lease.epoch = resource.lease.epoch

                for seq in resource.lease.sequence:
                    new_resource.lease.sequence.append(seq)

                new_resource.lease_owner.client_name = resource.lease_owner.client_name
                new_resource.lease_owner.user_name = resource.lease_owner.user_name

                lease_array_msg.resources.append(new_resource)

            self.lease_pub.publish(lease_array_msg)

    def world_objects_callback(self, results: Any) -> None:
        if self.spot_wrapper is None:
            return

        world_objects = self.spot_wrapper.world_objects
        if world_objects:
            # TF
            tf_msg = get_tf_from_world_objects(
                world_objects.world_objects, self.spot_wrapper, self.preferred_odom_frame.value
            )
            if len(tf_msg.transforms) > 0:
                self.dynamic_broadcaster.sendTransform(tf_msg.transforms)

    def publish_graph_nav_pose_callback(self) -> None:
        if self.spot_wrapper is None:
            return

        try:
            # noinspection PyProtectedMember
            state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if not state.localization.waypoint_id:
                self.get_logger().warning("Robot is not localized; Please upload graph and localize.")
                return

            seed_t_body_msg, seed_t_body_trans_msg = conv.bosdyn_localization_to_pose_msg(
                state.localization,
                self.spot_wrapper.robotToLocalTime,
                in_seed_frame=True,
                seed_frame=self.graph_nav_seed_frame,
                body_frame=self.tf_name_graph_nav_body,
                return_tf=True,
            )
            self.graph_nav_pose_pub.publish(seed_t_body_msg)
            self.graph_nav_pose_transform_broadcaster.sendTransform(seed_t_body_trans_msg)
        except Exception as e:
            self.get_logger().error(f"Exception: {e} \n {traceback.format_exc()}")

    def create_image_publisher(self, image_type: SpotImageType, callback_group: CallbackGroup) -> None:
        topic_name = image_type.value
        publisher_name = image_type.value
        # RGB is the only type with different naming scheme
        if image_type == SpotImageType.RGB:
            topic_name = "camera"
            publisher_name = "image"
        for camera_name in self.cameras_used.value:
            setattr(
                self,
                f"{camera_name}_{publisher_name}_pub",
                self.create_publisher(Image, f"{topic_name}/{camera_name}/image", 1),
            )
            setattr(
                self,
                f"{camera_name}_{publisher_name}_info_pub",
                self.create_publisher(CameraInfo, f"{topic_name}/{camera_name}/camera_info", 1),
            )
        # create a timer for publishing
        self.create_timer(
            1 / self.rates["front_image"],
            partial(self.publish_camera_images_callback, image_type),
            callback_group=callback_group,
        )

    def publish_camera_images_callback(self, image_type: SpotImageType) -> None:
        """
        Publishes the camera images from a specific image type
        """
        if self.spot_wrapper is None:
            return

        publisher_name = image_type.value
        # RGB is the only type with different naming scheme
        if image_type == SpotImageType.RGB:
            publisher_name = "image"

        result = self.spot_wrapper.spot_images.get_images_by_cameras(
            [CameraSource(camera_name, [image_type]) for camera_name in self.cameras_used.value]
        )
        for image_entry in result:
            image_msg, camera_info = bosdyn_data_to_image_and_camera_info_msgs(
                image_entry.image_response, self.spot_wrapper.robotToLocalTime, self.spot_wrapper.frame_prefix
            )
            image_pub = getattr(self, f"{image_entry.camera_name}_{publisher_name}_pub")
            image_info_pub = getattr(self, f"{image_entry.camera_name}_{publisher_name}_info_pub")
            image_pub.publish(image_msg)
            image_info_pub.publish(camera_info)
            self.populate_camera_static_transforms(image_entry.image_response)

    def service_wrapper(
        self,
        name: str,
        handler: Callable[[Request, Response], Response],
        request: Request,
        response: Response,
    ) -> Response:
        if self.spot_wrapper is None:
            self.get_logger().info(f"Mock mode: service {name} successfully called with request {request}")
            response.success = True
            return response
        return handler(request, response)

    def handle_claim(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the claim service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.claim()
        return response

    def handle_release(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the release service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.release()
        return response

    def handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stop service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.stop()
        return response

    def handle_self_right(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the self-right service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.self_right()
        return response

    def handle_sit(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the sit service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.sit()
        return response

    def handle_stand(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stand service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.stand()
        return response

    def handle_rollover(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the rollover service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.battery_change_pose()
        return response

    def handle_power_on(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the power-on service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.power_on()
        return response

    def handle_safe_power_off(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the safe-power-off service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.safe_power_off()
        return response

    def handle_estop_hard(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.assertEStop(True)
        return response

    def handle_estop_soft(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.assertEStop(False)
        return response

    def handle_estop_disengage(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to disengage the eStop on the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.disengageEStop()
        return response

    def handle_undock(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to undock the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_docking.undock()
        return response

    def handle_clear_behavior_fault(
        self, request: ClearBehaviorFault.Request, response: ClearBehaviorFault.Response
    ) -> ClearBehaviorFault.Response:
        """ROS service handler for clearing behavior faults"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.clear_behavior_fault(request.id)
        return response

    def handle_execute_dance(
        self, request: ExecuteDance.Request, response: ExecuteDance.Response
    ) -> ExecuteDance.Response:
        """ROS service handler for uploading and executing dance."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.execute_dance(request.choreo_file_content)
        return response

    def handle_list_all_dances(
        self, request: ListAllDances.Request, response: ListAllDances.Response
    ) -> ListAllDances.Response:
        """ROS service handler for getting list of already uploaded dances."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message, response.dances = self.spot_wrapper.list_all_dances()
        return response

    def handle_list_all_moves(
        self, request: ListAllMoves.Request, response: ListAllMoves.Response
    ) -> ListAllMoves.Response:
        """ROS service handler for getting list of already uploaded moves."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message, response.moves = self.spot_wrapper.list_all_moves()
        return response

    def handle_upload_animation(
        self, request: UploadAnimation.Request, response: UploadAnimation.Response
    ) -> UploadAnimation.Response:
        """ROS service handler for uploading an animation."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.upload_animation(
            request.animation_name, request.animation_file_content
        )
        return response

    def handle_list_sounds(self, request: ListSounds.Request, response: ListSounds.Response) -> ListSounds.Response:
        """ROS service handler for listing sounds loaded on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            names = self.spot_cam_wrapper.audio.list_sounds()
            response.names = names
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_load_sound(self, request: LoadSound.Request, response: LoadSound.Response) -> LoadSound.Response:
        """ROS service handler for loading a wav file sound on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.load_sound(request.wav_path, request.name)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_play_sound(self, request: PlaySound.Request, response: PlaySound.Response) -> PlaySound.Response:
        """ROS service handler for playing a sound loaded on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.play_sound(request.name, request.volume_multiplier)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_delete_sound(self, request: DeleteSound.Request, response: DeleteSound.Response) -> DeleteSound.Response:
        """ROS service handler for deleting a sound loaded on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.delete_sound(request.name)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_get_volume(self, request: GetVolume.Request, response: GetVolume.Response) -> GetVolume.Response:
        """ROS service handler for getting the volume on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            response.volume = self.spot_cam_wrapper.audio.get_volume()
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_set_volume(self, request: SetVolume.Request, response: SetVolume.Response) -> SetVolume.Response:
        """ROS service handler for setting the volume on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.set_volume(request.volume)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_stair_mode(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """ROS service handler to set a stair mode to the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = request.data
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = "Error:{}".format(e)
            return response

    def handle_locomotion_mode(
        self, request: SetLocomotion.Request, response: SetLocomotion.Response
    ) -> SetLocomotion.Response:
        """ROS service handler to set locomotion mode"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = request.locomotion_mode
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = "Error:{}".format(e)
            return response

    def handle_dock(self, request: Dock.Request, response: Dock.Response) -> Dock.Response:
        """ROS service handler to dock the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_docking.dock(request.dock_id)
        return response

    def handle_max_vel(self, request: SetVelocity.Request, response: SetVelocity.Response) -> SetVelocity.Response:
        """
        Handle a max_velocity service call. This will modify the mobility params to have a limit on the maximum
        velocity that the robot can move during motion commands. This affects trajectory commands and velocity
        commands
        Args:
            response: SetVelocity.Response containing response
            request: SetVelocity.Request containing requested maximum velocity
        Returns: SetVelocity.Response
        """
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(
                SE2VelocityLimit(
                    max_vel=math_helpers.SE2Velocity(
                        request.velocity_limit.linear.x,
                        request.velocity_limit.linear.y,
                        request.velocity_limit.angular.z,
                    ).to_proto()
                )
            )
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def _robot_command_goal_complete(self, feedback: RobotCommandFeedback) -> GoalResponse:
        if feedback is None:
            # NOTE: it takes an iteration for the feedback to get set.
            return GoalResponse.IN_PROGRESS

        if feedback.command.command_choice == feedback.command.COMMAND_FULL_BODY_FEEDBACK_SET:
            full_body_feedback = feedback.command.full_body_feedback
            if full_body_feedback.status.value != full_body_feedback.status.STATUS_PROCESSING:
                return GoalResponse.IN_PROGRESS
            if full_body_feedback.feedback.feedback_choice == full_body_feedback.feedback.FEEDBACK_STOP_FEEDBACK_SET:
                return GoalResponse.SUCCESS
            elif (
                full_body_feedback.feedback.feedback_choice == full_body_feedback.feedback.FEEDBACK_FREEZE_FEEDBACK_SET
            ):
                return GoalResponse.SUCCESS
            elif (
                full_body_feedback.feedback.feedback_choice
                == full_body_feedback.feedback.FEEDBACK_SELFRIGHT_FEEDBACK_SET
            ):
                if (
                    full_body_feedback.feedback.selfright_feedback.status.value
                    == full_body_feedback.feedback.selfright_feedback.status.STATUS_COMPLETED
                ):
                    return GoalResponse.SUCCESS
                else:
                    return GoalResponse.IN_PROGRESS
            elif (
                full_body_feedback.feedback.feedback_choice
                == full_body_feedback.feedback.FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET
            ):
                if (
                    full_body_feedback.feedback.safe_power_off_feedback.status.value
                    == full_body_feedback.feedback.safe_power_off_feedback.status.STATUS_POWERED_OFF
                ):
                    return GoalResponse.SUCCESS
                else:
                    return GoalResponse.IN_PROGRESS
            elif (
                full_body_feedback.feedback.feedback_choice
                == full_body_feedback.feedback.FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET
            ):
                if (
                    full_body_feedback.feedback.battery_change_pose_feedback.status.value
                    == full_body_feedback.feedback.battery_change_pose_feedback.status.STATUS_COMPLETED
                ):
                    return GoalResponse.SUCCESS
                if (
                    full_body_feedback.feedback.battery_change_pose_feedback.status.value
                    == full_body_feedback.feedback.battery_change_pose_feedback.status.STATUS_FAILED
                ):
                    return GoalResponse.FAILED
                return GoalResponse.IN_PROGRESS
            elif (
                full_body_feedback.feedback.feedback_choice
                == full_body_feedback.feedback.FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET
            ):
                if (
                    full_body_feedback.feedback.payload_estimation_feedback.status.value
                    == full_body_feedback.feedback.payload_estimation_feedback.status.STATUS_COMPLETED
                ):
                    return GoalResponse.SUCCESS
                if (
                    full_body_feedback.feedback.payload_estimation_feedback.status.value
                    == full_body_feedback.feedback.payload_estimation_feedback.status.STATUS_SMALL_MASS
                ):
                    return GoalResponse.SUCCESS
                if (
                    full_body_feedback.feedback.payload_estimation_feedback.status.value
                    == full_body_feedback.feedback.payload_estimation_feedback.status.STATUS_ERROR
                ):
                    return GoalResponse.FAILED
                return GoalResponse.IN_PROGRESS
            elif (
                full_body_feedback.feedback.feedback_choice
                == full_body_feedback.feedback.FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET
            ):
                if (
                    full_body_feedback.feedback.constrained_manipulation_feedback.status.value
                    == full_body_feedback.feedback.constrained_manipulation_feedback.status.STATUS_RUNNING
                ):
                    return GoalResponse.IN_PROGRESS
                return GoalResponse.FAILED
            else:
                return GoalResponse.IN_PROGRESS

        elif feedback.command.command_choice == feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET:
            # The idea here is that a synchronized command can have arm, mobility, and/or gripper
            # sub-commands in it.  The total command isn't done until all sub-commands are satisfied.
            # So if any one of the sub-commands is still in progress, it short-circuits out as
            # IN_PROGRESS.  And if it makes it to the bottom of the function, then all components
            # must be satisfied, and it returns SUCCESS.
            # One corner case to know about is that the commands that don't provide feedback, such
            # as a velocity command will return SUCCESS.  This allows you to use them more effectively
            # with other commands.  For example if you wanted to move the arm with some velocity
            # while the mobility is going to some SE2 trajectory then that will work.

            sync_feedback = feedback.command.synchronized_feedback
            if sync_feedback.arm_command_feedback_is_set is True:
                arm_feedback = sync_feedback.arm_command_feedback
                if (
                    arm_feedback.status.value == arm_feedback.status.STATUS_COMMAND_OVERRIDDEN
                    or arm_feedback.status.value == arm_feedback.status.STATUS_COMMAND_TIMED_OUT
                    or arm_feedback.status.value == arm_feedback.status.STATUS_ROBOT_FROZEN
                    or arm_feedback.status.value == arm_feedback.status.STATUS_INCOMPATIBLE_HARDWARE
                ):
                    return GoalResponse.FAILED
                if arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET:
                    if (
                        arm_feedback.feedback.arm_cartesian_feedback.status.value
                        != arm_feedback.feedback.arm_cartesian_feedback.status.STATUS_TRAJECTORY_COMPLETE
                    ):
                        return GoalResponse.IN_PROGRESS
                elif (
                    arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET
                ):
                    if (
                        arm_feedback.feedback.arm_joint_move_feedback.status.value
                        != arm_feedback.feedback.arm_joint_move_feedback.status.STATUS_COMPLETE
                    ):
                        return GoalResponse.IN_PROGRESS
                elif (
                    arm_feedback.feedback.feedback_choice
                    == arm_feedback.feedback.FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET
                ):
                    if (
                        arm_feedback.feedback.named_arm_position_feedback.status.value
                        != arm_feedback.feedback.named_arm_position_feedback.status.STATUS_COMPLETE
                    ):
                        return GoalResponse.IN_PROGRESS
                elif arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_VELOCITY_FEEDBACK_SET:
                    self.get_logger().warn("WARNING: ArmVelocityCommand provides no feedback")
                    pass  # May return SUCCESS below
                elif arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_GAZE_FEEDBACK_SET:
                    if (
                        arm_feedback.feedback.arm_gaze_feedback.status.value
                        != arm_feedback.feedback.arm_gaze_feedback.status.STATUS_TRAJECTORY_COMPLETE
                    ):
                        return GoalResponse.IN_PROGRESS
                elif arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_STOP_FEEDBACK_SET:
                    self.get_logger().warn("WARNING: Stop command provides no feedback")
                    pass  # May return SUCCESS below
                elif arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_DRAG_FEEDBACK_SET:
                    if (
                        arm_feedback.feedback.arm_drag_feedback.status.value
                        != arm_feedback.feedback.arm_drag_feedback.status.STATUS_DRAGGING
                    ):
                        return GoalResponse.FAILED
                elif arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET:
                    self.get_logger().warn("WARNING: ArmImpedanceCommand provides no feedback")
                    pass  # May return SUCCESS below
                else:
                    self.get_logger().error("ERROR: unknown arm command type")
                    return GoalResponse.IN_PROGRESS

            if sync_feedback.mobility_command_feedback_is_set is True:
                mob_feedback = sync_feedback.mobility_command_feedback
                if (
                    mob_feedback.status.value == mob_feedback.status.STATUS_COMMAND_OVERRIDDEN
                    or mob_feedback.status.value == mob_feedback.status.STATUS_COMMAND_TIMED_OUT
                    or mob_feedback.status.value == mob_feedback.status.STATUS_ROBOT_FROZEN
                    or mob_feedback.status.value == mob_feedback.status.STATUS_INCOMPATIBLE_HARDWARE
                ):
                    return GoalResponse.FAILED
                if mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET:
                    if (
                        mob_feedback.feedback.se2_trajectory_feedback.status.value
                        != mob_feedback.feedback.se2_trajectory_feedback.status.STATUS_AT_GOAL
                    ):
                        return GoalResponse.IN_PROGRESS
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_SE2_VELOCITY_FEEDBACK_SET:
                    self.get_logger().warn("WARNING: Planar velocity commands provide no feedback")
                    pass  # May return SUCCESS below
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_SIT_FEEDBACK_SET:
                    if (
                        mob_feedback.feedback.sit_feedback.status.value
                        != mob_feedback.feedback.sit_feedback.status.STATUS_IS_SITTING
                    ):
                        return GoalResponse.IN_PROGRESS
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_STAND_FEEDBACK_SET:
                    if (
                        mob_feedback.feedback.stand_feedback.status.value
                        != mob_feedback.feedback.stand_feedback.status.STATUS_IS_STANDING
                    ):
                        return GoalResponse.IN_PROGRESS
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_STANCE_FEEDBACK_SET:
                    if (
                        mob_feedback.feedback.stance_feedback.status.value
                        == mob_feedback.feedback.stance_feedback.status.STATUS_TOO_FAR_AWAY
                    ):
                        return GoalResponse.FAILED
                    if (
                        mob_feedback.feedback.stance_feedback.status.value
                        != mob_feedback.feedback.stance_feedback.status.STATUS_STANCED
                    ):
                        return GoalResponse.IN_PROGRESS
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_STOP_FEEDBACK_SET:
                    self.get_logger().warn("WARNING: Stop command provides no feedback")
                    pass  # May return SUCCESS below
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_FOLLOW_ARM_FEEDBACK_SET:
                    self.get_logger().warn("WARNING: FollowArmCommand provides no feedback")
                    pass  # May return SUCCESS below
                elif mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_NOT_SET:
                    # sync_feedback.mobility_command_feedback_is_set, feedback_choice is actually not set.
                    # This may happen when a command finishes, which means we may return SUCCESS below.
                    self.get_logger().info("mobility command feedback indicates goal has reached")
                    pass
                else:
                    self.get_logger().error("ERROR: unknown mobility command type")
                    return GoalResponse.IN_PROGRESS

            if sync_feedback.gripper_command_feedback_is_set is True:
                grip_feedback = sync_feedback.gripper_command_feedback
                if (
                    grip_feedback.status.value == grip_feedback.status.STATUS_COMMAND_OVERRIDDEN
                    or grip_feedback.status.value == grip_feedback.status.STATUS_COMMAND_TIMED_OUT
                    or grip_feedback.status.value == grip_feedback.status.STATUS_ROBOT_FROZEN
                    or grip_feedback.status.value == grip_feedback.status.STATUS_INCOMPATIBLE_HARDWARE
                ):
                    return GoalResponse.FAILED
                if grip_feedback.command.command_choice == grip_feedback.command.COMMAND_CLAW_GRIPPER_FEEDBACK_SET:
                    if (
                        grip_feedback.command.claw_gripper_feedback.status.value
                        != grip_feedback.command.claw_gripper_feedback.status.STATUS_AT_GOAL
                    ):
                        return GoalResponse.IN_PROGRESS
                else:
                    self.get_logger().error("ERROR: unknown gripper command type")
                    return GoalResponse.IN_PROGRESS

            return GoalResponse.SUCCESS

        else:
            self.get_logger().error("ERROR: unknown robot command type")
            return GoalResponse.IN_PROGRESS

    def _get_robot_command_feedback(self, goal_id: Optional[str]) -> RobotCommandFeedback:
        feedback = RobotCommandFeedback()
        if goal_id is None:
            mobility_command_feedback = feedback.command.synchronized_feedback.mobility_command_feedback
            feedback.command.command_choice = feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET
            mobility_command_feedback.status.value = mobility_command_feedback.status.STATUS_PROCESSING
            mobility_command_feedback.feedback.feedback_choice = (
                mobility_command_feedback.feedback.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET
            )
            if self._wait_for_goal is not None and self._wait_for_goal.at_goal:
                mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
                    mobility_command_feedback.feedback.se2_trajectory_feedback.status.STATUS_AT_GOAL
                )
            else:
                mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
                    mobility_command_feedback.feedback.se2_trajectory_feedback.status.STATUS_GOING_TO_GOAL
                )
        else:
            if self.spot_wrapper is not None:
                conv.convert_proto_to_bosdyn_msgs_robot_command_feedback(
                    self.spot_wrapper.get_robot_command_feedback(goal_id).feedback, feedback
                )
        return feedback

    def handle_robot_command(self, goal_handle: ServerGoalHandle) -> RobotCommand.Result:
        ros_command = goal_handle.request.command
        proto_command = robot_command_pb2.RobotCommand()
        conv.convert_bosdyn_msgs_robot_command_to_proto(ros_command, proto_command)
        self._wait_for_goal = None
        if self.spot_wrapper is None:
            self._wait_for_goal = WaitForGoal(self.get_clock(), 2.0)
            goal_id = None
        else:
            success, err_msg, goal_id = self.spot_wrapper.robot_command(proto_command)
            if not success:
                raise Exception(err_msg)

        self.get_logger().info("Robot now executing goal " + str(goal_id))
        # The command is non-blocking, but we need to keep this function up in order to interrupt if a
        # preempt is requested and to return success if/when the robot reaches the goal. Also check the is_active to
        # monitor whether the timeout_cb has already aborted the command
        feedback: Optional[RobotCommandFeedback] = None
        feedback_msg: Optional[RobotCommand.Feedback] = None
        while (
            rclpy.ok()
            and not goal_handle.is_cancel_requested
            and self._robot_command_goal_complete(feedback) == GoalResponse.IN_PROGRESS
            and goal_handle.is_active
        ):
            feedback = self._get_robot_command_feedback(goal_id)
            feedback_msg = RobotCommand.Feedback(feedback=feedback)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # don't use rate here because we're already in a single thread

        # publish a final feedback
        result = RobotCommand.Result()
        if feedback is not None:
            goal_handle.publish_feedback(feedback_msg)
            result.result = feedback

        result.success = self._robot_command_goal_complete(feedback) == GoalResponse.SUCCESS

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
        elif not goal_handle.is_active:
            result.success = False
            result.message = "Cancelled"
            # Don't abort because that's already happened
        elif result.success:
            result.message = "Successfully completed command"
            goal_handle.succeed()
        else:
            result.message = "Failed to complete command"
            goal_handle.abort()
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self.get_logger().info("Returning action result " + str(result))
        return result

    def _manipulation_goal_complete(self, feedback: Optional[ManipulationApiFeedbackResponse]) -> GoalResponse:
        if feedback is None:
            # NOTE: it takes an iteration for the feedback to get set.
            return GoalResponse.IN_PROGRESS

        if feedback.current_state.value == feedback.current_state.MANIP_STATE_UNKNOWN:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_DONE:
            return GoalResponse.SUCCESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_SEARCHING_FOR_GRASP:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_MOVING_TO_GRASP:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASPING_OBJECT:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACING_OBJECT:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_SUCCEEDED:
            return GoalResponse.SUCCESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_FAILED:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_PLANNING_SUCCEEDED:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_WALKING_TO_OBJECT:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_ATTEMPTING_RAYCASTING:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_MOVING_TO_PLACE:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACE_FAILED_TO_RAYCAST_INTO_MAP:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACE_SUCCEEDED:
            return GoalResponse.SUCCESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACE_FAILED:
            return GoalResponse.FAILED
        else:
            raise Exception("Unknown manipulation state type")

    def _get_manipulation_command_feedback(self, goal_id: str) -> ManipulationApiFeedbackResponse:
        feedback = ManipulationApiFeedbackResponse()
        if self.spot_wrapper is not None:
            conv.convert_proto_to_bosdyn_msgs_manipulation_api_feedback_response(
                self.spot_wrapper.get_manipulation_command_feedback(goal_id), feedback
            )
        return feedback

    def handle_manipulation_command(self, goal_handle: ServerGoalHandle) -> Manipulation.Result:
        # Most of the logic here copied from handle_robot_command
        self.get_logger().debug("I'm a function that handles request to the manipulation api!")

        ros_command = goal_handle.request.command
        proto_command = manipulation_api_pb2.ManipulationApiRequest()
        conv.convert_bosdyn_msgs_manipulation_api_request_to_proto(ros_command, proto_command)
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self._wait_for_goal = WaitForGoal(self.get_clock(), 2.0)
            goal_id: Optional[str] = None
        else:
            success, err_msg, goal_id = self.spot_wrapper.manipulation_command(proto_command)
            if not success:
                raise Exception(err_msg)

        self.get_logger().info("Robot now executing goal " + str(goal_id))
        # The command is non-blocking, but we need to keep this function up in order to interrupt if a
        # preempt is requested and to return success if/when the robot reaches the goal. Also check the is_active to
        # monitor whether the timeout_cb has already aborted the command
        feedback: Optional[ManipulationApiFeedbackResponse] = None
        feedback_msg: Optional[Manipulation.Feedback] = None
        while (
            rclpy.ok()
            and not goal_handle.is_cancel_requested
            and self._manipulation_goal_complete(feedback) == GoalResponse.IN_PROGRESS
            and goal_handle.is_active
        ):
            try:
                if goal_id is not None:
                    feedback = self._get_manipulation_command_feedback(goal_id)
            except InternalServerError as e:
                self.get_logger().error(e)

            feedback_msg = Manipulation.Feedback(feedback=feedback)
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)  # don't use rate here because we're already in a single thread

        # publish a final feedback
        result = Manipulation.Result()
        if feedback is not None:
            goal_handle.publish_feedback(feedback_msg)
            result.result = feedback
        result.success = self._manipulation_goal_complete(feedback) == GoalResponse.SUCCESS

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
        elif not goal_handle.is_active:
            result.success = False
            result.message = "Cancelled"
            # Don't abort because that's already happened
        elif result.success:
            result.message = "Successfully completed manipulation"
            goal_handle.succeed()
        else:
            result.message = "Failed to complete manipulation"
            goal_handle.abort()
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self.get_logger().info("Returning action result " + str(result))
        return result

    def handle_trajectory(self, goal_handle: ServerGoalHandle) -> Optional[Trajectory.Result]:
        """ROS actionserver execution handler to handle receiving a request to move to a location"""
        result: Optional[Trajectory.Result] = None

        if goal_handle.request.target_pose.header.frame_id != "body":
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = "frame_id of target_pose must be 'body'"
            return result

        if goal_handle.request.duration.sec <= 0:
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = "duration must be larger than 0"
            return result

        if self.spot_wrapper is None:
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = "Spot wrapper is None"
            return result

        cmd_duration_secs = goal_handle.request.duration.sec * 1.0

        self.spot_wrapper.trajectory_cmd(
            goal_x=goal_handle.request.target_pose.pose.position.x,
            goal_y=goal_handle.request.target_pose.pose.position.y,
            goal_heading=math_helpers.Quat(
                w=goal_handle.request.target_pose.pose.orientation.w,
                x=goal_handle.request.target_pose.pose.orientation.x,
                y=goal_handle.request.target_pose.pose.orientation.y,
                z=goal_handle.request.target_pose.pose.orientation.z,
            ).to_yaw(),
            cmd_duration=cmd_duration_secs,
            precise_position=goal_handle.request.precise_positioning,
        )

        command_start_time = self.get_clock().now()

        # Abort the action server if cmd_duration is exceeded - the driver stops but does not provide
        # feedback to indicate this, so we monitor it ourselves
        # The trajectory command is non-blocking, but we need to keep this function up in order to
        # interrupt if a preempt is requested and to return success if/when the robot reaches the goal.
        # Also check the is_active to
        # monitor whether the timeout has already aborted the command

        #
        # Pre-emp missing in port to ROS2 (ros1: self.trajectory_server.is_preempt_requested())
        #

        # rate = rclpy.Rate(10)

        try:
            while rclpy.ok() and not self.spot_wrapper.at_goal and goal_handle.is_active:
                feedback = Trajectory.Feedback()
                if self.spot_wrapper.near_goal:
                    if self.spot_wrapper._last_trajectory_command_precise:
                        feedback.feedback = "Near goal, performing final adjustments"
                    else:
                        feedback.feedback = "Near goal"
                else:
                    feedback.feedback = "Moving to goal"

                # rate.sleep()
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

                # check for timeout
                com_dur = self.get_clock().now() - command_start_time

                if com_dur.nanoseconds / 1e9 > cmd_duration_secs:
                    # timeout, quit with failure
                    self.get_logger().error("TIMEOUT")
                    feedback = Trajectory.Feedback()
                    feedback.feedback = "Failed to reach goal, timed out"
                    goal_handle.publish_feedback(feedback)
                    goal_handle.abort()

            result = Trajectory.Result()
            result.success = False
            result.message = "timeout"

            # If still active after exiting the loop, the command did not time out
            if goal_handle.is_active:
                #            if self.trajectory_server.is_preempt_requested():
                #                self.trajectory_server.publish_feedback(TrajectoryFeedback("Preempted"))
                #                self.trajectory_server.set_preempted()
                #                self.spot_wrapper.stop()
                #                result.success = False
                #                result.message = 'preempt'

                feedback = Trajectory.Feedback()
                if self.spot_wrapper.at_goal:
                    # self.get_logger().error("SUCCESS")
                    feedback.feedback = "Reached goal"
                    goal_handle.publish_feedback(feedback)
                    result.success = True
                    result.message = ""
                    goal_handle.succeed()
                else:
                    # self.get_logger().error("FAIL")
                    feedback.feedback = "Failed to reach goal"
                    goal_handle.publish_feedback(feedback)
                    result.success = False
                    result.message = "not at goal"
                    goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f"Exception: {type(e)} - {e}")
            if result is not None:
                result.success = False
            result.message = f"Exception: {type(e)} - {e}"
        # self.get_logger().error(f"RETURN FROM HANDLE: {result}")
        return result

    def cmd_velocity_callback(self, data: Twist) -> None:
        """Callback for cmd_vel command"""
        if not self.spot_wrapper:
            self.get_logger().info(f"Mock mode, received command vel {data}")
            return
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z, self.cmd_duration)

    def body_pose_callback(self, data: Pose) -> None:
        """Callback for cmd_vel command"""
        if self.spot_wrapper is None:
            self.get_logger().info("Mock mode, received command vel " + str(data))
            return
        q = Quaternion()
        q.x = data.orientation.x
        q.y = data.orientation.y
        q.z = data.orientation.z
        q.w = data.orientation.w
        position = geometry_pb2.Vec3(z=data.position.z)
        pose = geometry_pb2.SE3Pose(position=position, rotation=q)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

        mobility_params = self.spot_wrapper.get_mobility_params()
        mobility_params.body_control.CopyFrom(body_control)
        self.spot_wrapper.set_mobility_params(mobility_params)

    def handle_graph_nav_get_localization_pose(
        self, request: GraphNavGetLocalizationPose.Response, response: GraphNavGetLocalizationPose.Response
    ) -> GraphNavGetLocalizationPose.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if not state.localization.waypoint_id:
                response.success = False
                response.message = "The robot is currently not localized to the map; Please localize."
                self.get_logger().warning(response.message)
                return response
            else:
                seed_t_body_msg = conv.bosdyn_localization_to_pose_msg(
                    state.localization,
                    self.spot_wrapper.robotToLocalTime,
                    in_seed_frame=True,
                    seed_frame=self.graph_nav_seed_frame,
                    body_frame=self.tf_name_graph_nav_body,
                    return_tf=False,
                )
                response.success = True
                response.message = "Success"
                response.pose = seed_t_body_msg
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        if response.success:
            self.get_logger().info("GraphNav localization pose received")
        return response

    def handle_graph_nav_set_localization(
        self, request: GraphNavSetLocalization.Request, response: GraphNavSetLocalization.Response
    ) -> GraphNavSetLocalization.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            if request.method == "fiducial":
                self.spot_wrapper._set_initial_localization_fiducial()
                response.success = True
                response.message = "Success"
            elif request.method == "waypoint":
                self.spot_wrapper._set_initial_localization_waypoint([request.waypoint_id])
                response.success = True
                response.message = "Success"
            else:
                response.success = False
                response.message = f"Invalid localization method {request.method}.Must be 'fiducial' or 'waypoint'"
                raise Exception(response.message)
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        if response.success:
            self.get_logger().info(f"Successfully set GraphNav localization. Method: {request.method}")
        return response

    def handle_graph_nav_upload_graph(
        self, request: GraphNavUploadGraph.Request, response: GraphNavUploadGraph.Response
    ) -> GraphNavUploadGraph.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            self.get_logger().info(f"Uploading GraphNav map: {request.upload_filepath}")
            self.spot_wrapper._upload_graph_and_snapshots(request.upload_filepath)
            self.get_logger().info("Uploaded")
            response.success = True
            response.message = "Success"
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        return response

    def handle_graph_nav_clear_graph(
        self, request: GraphNavClearGraph.Request, response: GraphNavClearGraph.Response
    ) -> GraphNavClearGraph.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            self.get_logger().info("Clearing graph")
            self.spot_wrapper._clear_graph()
            self.get_logger().info("Cleared")
            response.success = True
            response.message = "Success"
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        return response

    def handle_list_graph(self, request: ListGraph.Request, response: ListGraph.Response) -> ListGraph.Response:
        """ROS service handler for listing graph_nav waypoint_ids"""
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            self.get_logger().error(f"handle_list_graph: {request}")
            self.spot_wrapper._clear_graph()
            self.spot_wrapper._upload_graph_and_snapshots(request.upload_filepath)
            response.waypoint_ids = self.spot_wrapper.list_graph(request.upload_filepath)
            self.get_logger().error(f"handle_list_graph RESPONSE: {response}")
        except Exception as e:
            self.get_logger().error("Exception Error:{}".format(e))
        return response

    def handle_list_world_objects(
        self, request: ListWorldObjects.Request, response: ListWorldObjects.Response
    ) -> Optional[ListWorldObjects.Response]:
        # For some reason exceptions in service callbacks don't print which makes debugging difficult!
        try:
            return self._handle_list_world_objects(request, response)
        except Exception:
            self.get_logger().error(f"In handling list world objects, exception was {traceback.format_exc()}")
        return None

    def _handle_list_world_objects(
        self, request: ListWorldObjects.Request, response: ListWorldObjects.Response
    ) -> ListWorldObjects.Response:
        object_types = [ot.value for ot in request.request.object_type]
        time_start_point = None
        if request.request.timestamp_filter_is_set:
            time_start_point = (
                request.request.timestamp_filter.sec + float(request.request.timestamp_filter.nanosec) / 1e9
            )
        if self.spot_wrapper is None:
            self.get_logger().info(f"Mock return for {object_types} after time {time_start_point}")
            # return a fake list
            proto_response = world_object_pb2.ListWorldObjectResponse()
            world_object = proto_response.world_objects.add()
            world_object.name = "my_fiducial_3"
            world_object.apriltag_properties.tag_id = 3
            world_object.apriltag_properties.frame_name_fiducial = "fiducial_3"
            world_object.apriltag_properties.frame_name_fiducial_filtered = "filtered_fiducial_3"
        else:
            proto_response = self.spot_wrapper.list_world_objects(object_types, time_start_point)
        conv.convert_proto_to_bosdyn_msgs_list_world_object_response(proto_response, response.response)
        return response

    def handle_navigate_to_feedback(self) -> None:
        """Thread function to send navigate_to feedback"""
        if self.spot_wrapper is None:
            return

        while rclpy.ok() and self.run_navigate_to:
            localization_state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if localization_state.localization.waypoint_id:
                feedback = NavigateTo.Feedback()
                feedback.waypoint_id = localization_state.localization.waypoint_id
                if self.goal_handle is not None:
                    self.goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
            # rclpy.Rate(10).sleep()

    def handle_navigate_to(self, goal_handle: ServerGoalHandle) -> NavigateTo.Result:
        """ROS service handler to run mission of the robot.  The robot will replay a mission"""
        # create thread to periodically publish feedback

        self.goal_handle = goal_handle
        feedback_thread = threading.Thread(target=self.handle_navigate_to_feedback, args=())
        self.run_navigate_to = True
        feedback_thread.start()
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response = NavigateTo.Result()
            response.success = False
            response.message = "Spot wrapper is None"
            goal_handle.abort()
            return response

        # run navigate_to
        resp = self.spot_wrapper.navigate_to(
            upload_path=goal_handle.request.upload_path,
            navigate_to=goal_handle.request.navigate_to,
            initial_localization_fiducial=goal_handle.request.initial_localization_fiducial,
            initial_localization_waypoint=goal_handle.request.initial_localization_waypoint,
        )
        self.run_navigate_to = False
        feedback_thread.join()

        result = NavigateTo.Result()
        result.success = resp[0]
        result.message = resp[1]
        # check status
        if resp[0]:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def populate_camera_static_transforms(self, image_data: image_pb2.Image) -> None:
        """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
        transforms. This is the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot, so we need to generate the
        transforms at runtime.
        Args:
        image_data: Image protobuf data from the wrapper
        """
        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the preferred_odom_frame, and will be published
        # by the non-static transform publishing that is done by the state callback
        frame_prefix = MOCK_HOSTNAME + "/"
        if self.spot_wrapper is not None:
            frame_prefix = self.spot_wrapper.frame_prefix
        excluded_frames = [self.tf_name_vision_odom.value, self.tf_name_kinematic_odom.value, frame_prefix + "body"]
        excluded_frames = [f[f.rfind("/") + 1 :] for f in excluded_frames]

        # Special case handling for hand camera frames that reference the link "arm0.link_wr1" in their
        # transform snapshots. This name only appears in hand camera transform snapshots and appears to
        # be a bug in this particular image callback path.
        #
        # 1. We exclude publishing a static transform from arm0.link_wr1 -> body here because it depends
        #    on the arm's position and a static transform would fix it to its initial position.
        #
        # 2. Below we rename the parent link "arm0.link_wr1" to "link_wr1" as it appears in robot state
        #    which is used for publishing dynamic tfs elsewhere. Without this, the hand camera frame
        #    positions would never properly update as no other pipelines reference "arm0.link_wr1".
        #
        # We save an RPC call to self.spot_wrapper.has_arm() and any extra complexity here as the link
        # will not exist if the spot does not have an arm and the special case code will have no effect.
        excluded_frames.append("arm0.link_wr1")

        for frame_name in image_data.shot.transforms_snapshot.child_to_parent_edge_map:
            if frame_name in excluded_frames:
                continue

            transform = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            parent_frame = transform.parent_frame_name

            # special case handling of parent frame to sync with robot state naming, see above
            if parent_frame == "arm0.link_wr1":
                parent_frame = "link_wr1"

            existing_transforms = [
                (transform.header.frame_id, transform.child_frame_id) for transform in self.camera_static_transforms
            ]
            if (frame_prefix + parent_frame, frame_prefix + frame_name) in existing_transforms:
                # We already extracted this transform
                continue

            if self.spot_wrapper is not None:
                local_time = self.spot_wrapper.robotToLocalTime(image_data.shot.acquisition_time)
            else:
                local_time = Timestamp()
            tf_time = builtin_interfaces.msg.Time(sec=local_time.seconds, nanosec=local_time.nanos)
            static_tf = populate_transform_stamped(
                tf_time, parent_frame, frame_name, transform.parent_tform_child, frame_prefix
            )
            self.camera_static_transforms.append(static_tf)
            self.camera_static_transform_broadcaster.sendTransform(self.camera_static_transforms)

    def shutdown(self, sig: Optional[Any] = None, frame: Optional[str] = None) -> None:
        self.get_logger().info("Shutting down ROS driver for Spot")
        if self.spot_wrapper is not None:
            self.spot_wrapper.sit()
        self.node_rate.sleep()
        if self.spot_wrapper is not None:
            self.spot_wrapper.disconnect()
        self.destroy_node()

    def step(self) -> None:
        """Update spot sensors"""
        if not self._printed_once:
            self.get_logger().info("Driver successfully started!")
            self._printed_once = True
        self.get_logger().debug("Step/Update")
        if rclpy.ok():
            if self.spot_wrapper is not None:
                self.spot_wrapper.updateTasks()  # Testing with Robot
            self.get_logger().debug("UPDATE TASKS")
            feedback_msg = Feedback()
            if self.spot_wrapper:
                feedback_msg.standing = self.spot_wrapper.is_standing
                feedback_msg.sitting = self.spot_wrapper.is_sitting
                feedback_msg.moving = self.spot_wrapper.is_moving
                _id = self.spot_wrapper.id
                try:
                    feedback_msg.serial_number = _id.serial_number
                    feedback_msg.species = _id.species
                    feedback_msg.version = _id.version
                    feedback_msg.nickname = _id.nickname
                    feedback_msg.computer_serial_number = _id.computer_serial_number
                except AttributeError:
                    pass
            self.feedback_pub.publish(feedback_msg)
            mobility_params_msg = MobilityParams()
            if self.spot_wrapper is not None:
                try:
                    mobility_params = self.spot_wrapper.get_mobility_params()
                    mobility_params_msg.body_control.position.x = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.x
                    )
                    mobility_params_msg.body_control.position.y = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.y
                    )
                    mobility_params_msg.body_control.position.z = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.z
                    )
                    mobility_params_msg.body_control.orientation.x = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.x
                    )
                    mobility_params_msg.body_control.orientation.y = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.y
                    )
                    mobility_params_msg.body_control.orientation.z = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.z
                    )
                    mobility_params_msg.body_control.orientation.w = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.w
                    )
                    mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint
                    mobility_params_msg.stair_hint = mobility_params.stair_hint
                except Exception as e:
                    self.get_logger().error("Error:{}".format(e))
                    pass
            self.mobility_params_pub.publish(mobility_params_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    spot_ros = SpotROS()
    try:
        spot_ros.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if spot_ros.spot_wrapper is not None:
        spot_ros.spot_wrapper.disconnect()
    spot_ros.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
