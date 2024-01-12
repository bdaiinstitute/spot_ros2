
    GRAV_ALIGNED_BODY_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from rclpy.node import Node

import spot_driver.conversions as conv
from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.srv import Dock, GetInverseKinematicSolutions  # type: ignore



    def __init__(self, node: Node, args: argparse.Namespace):
        self._node = node
        self._robot_name: str = args.robot
        self._dock_id: int = args.dock
        self._poses: int = args.poses
        self._logger = node.get_logger()
        self._tf_broadcaster = TransformBroadcaster(node)

        self._timer = node.create_timer(0.1, self._timer_callback)
        self._transforms: List[geometry_msgs.msg.TransformStamped] = []

        self._dock_client = node.create_client(Dock, namespace_with(self._robot_name, "dock"))
        self._ik_client = node.create_client(
            GetInverseKinematicSolutions,
            namespace_with(self._robot_name, "get_inverse_kinematic_solutions"),

            tf.header.stamp = self._node.get_clock().now().to_msg()
            self._tf_broadcaster.sendTransform(tf)

    def _walk_to(self, pose: SE2Pose) -> bool:
        """
        Walk the robot to a given pose.
        """
        frame_t_goal = pose
        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=frame_t_goal.x,
            goal_y=frame_t_goal.y,
            goal_heading=frame_t_goal.angle,
            frame_name=VISION_FRAME_NAME,
        )
        action_goal = RobotCommand.Goal()
        conv.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("walk_forward", action_goal)

    def _ready_arm(self) -> bool:
        """
        Unstow the robot arm.

        return self._robot_command_client.send_goal_and_wait("ready_arm", action_goal)

    def _dock(self, dock_id: int) -> bool:
        """
        Dock the robot.
        """
        request = Dock.Request()
        request.dock_id = dock_id
        response = self._dock_client.call(request)
        return response.success

    def _send_ik_request(

            return False
        self._logger.info("Successfully stood up.")

        # Walk forward 1.2m.
        self._logger.info("Walking forward")
        result = self._walk_to(SE2Pose(1.2, 0, 0))
        if not result:
            self._logger.error("Cannot walk forward")
            return False
        self._logger.info("Successfully walked forward.")

        # Rotate 90 degrees left.
        self._logger.info("Rotate 90 degrees")
        result = self._walk_to(SE2Pose(1.2, 0, 1.5708))
        if not result:
            self._logger.error("Cannot rotate")
            return False
        self._logger.info("Successfully rotated 90 degrees.")

        # Unstow the arm.
        self._logger.info("Unstow the arm")
        result = self._ready_arm()

            )

        # Dock robot.
        self._logger.info("Docking the robot")
        result = self._dock(self._dock_id)
        if not result:
            self._logger.error("Unable to dock the robot")
            return False

        # Power off robot.
        self._logger.info("Powering robot off")
        result = self._robot.command("power_off")
        if not result:
            self._logger.error("Unable to power off robot")
            return False

        return True

    Parse all arguments.
    --robot [string]
        The robot name e.g. Opal.
    --dock [int]
        The docking station number (e.g. 527 for Spot Opal).
    -n --poses [int]
        Number of desired tool poses to query.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="The robot name.")
    parser.add_argument("--dock", type=int, required=True, help="The docking station number (527 for Spot Opal).")
    parser.add_argument("-n", "--poses", type=int, default=50, help="Number of desired tool poses to query.")
    return parser

