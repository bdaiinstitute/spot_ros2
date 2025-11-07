import argparse
from typing import Optional

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.api.world_object_pb2 import ListWorldObjectRequest, MutateWorldObjectRequest, WorldObjectType
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_api_msgs.msg import ListWorldObjectRequest as ListRequestRos
from bosdyn_api_msgs.msg import MutateWorldObjectRequest as MutateRequestRos
from bosdyn_msgs.conversions import convert
from synchros2.action import Actionable
from synchros2.service import Serviced
from synchros2.utilities import namespace_with

from spot_examples.simple_spot_commander import SimpleSpotCommander
from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.srv import ListWorldObjects, MutateWorldObject  # type: ignore

# relevant example from the spot SDK:
# https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/user_nogo_regions/user_nogo_regions.py


class NoGoRegion:
    def __init__(self, robot_name: Optional[str] = None, walk: bool = False) -> None:
        self._node = ros_scope.ensure_node()
        self._logger = self._node.get_logger()
        self._robot_name = robot_name
        self._walk = walk

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        tf_listener = ros_scope.tf_listener()
        assert tf_listener is not None  # Makes mypy happy
        tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        world_t_robot = tf_listener.lookup_a_tform_b(self._vision_frame_name, self._body_frame_name)
        self.world_t_robot_init = SE3Pose(
            world_t_robot.transform.translation.x,
            world_t_robot.transform.translation.y,
            world_t_robot.transform.translation.z,
            Quat(
                world_t_robot.transform.rotation.w,
                world_t_robot.transform.rotation.x,
                world_t_robot.transform.rotation.y,
                world_t_robot.transform.rotation.z,
            ),
        )

        self._robot = SimpleSpotCommander(self._robot_name, self._node)
        self._robot_command: Actionable = Actionable(RobotCommand, namespace_with(self._robot_name, "robot_command"))
        self._logger.info("Waiting for robot command action server")
        self._robot_command.wait_for_server()
        self._logger.info("Found robot command action server")
        self._list_wo: Serviced = Serviced(ListWorldObjects, namespace_with(self._robot_name, "list_world_objects"))
        self._logger.info("Waiting for list world objects service")
        self._list_wo.wait_for_service()
        self._logger.info("Found list world objects service")
        self._mutate_wo: Serviced = Serviced(
            MutateWorldObject, namespace_with(self._robot_name, "mutate_world_objects")
        )
        self._logger.info("Waiting for mutate world objects service")
        self._mutate_wo.wait_for_service()
        self._logger.info("Found mutate world objects service")

    def initialize_robot(self) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        # if we are testing with walking, stand the robot up
        if self._walk:
            result = self._robot.command("stand")
            if not result.success:
                self._logger.error("Robot did not stand message was " + result.message)
                return False
            self._logger.info("Successfully stood up.")
            return True
        return True

    def list_current_world_objects(self) -> None:
        request = ListWorldObjects.Request()
        response = self._list_wo.synchronously(request)
        print("Current world objects:")
        for wo in response.response.world_objects:
            print(f"\tID: {wo.id} name: {wo.name}")

    def add_nogo_region(
        self,
        box_x: float,
        box_y: float,
        frame_name: str,
        frame_t_box: SE3Pose,
        region_name: str = "nogo_region",
        lifetime: int = 300,
    ) -> Optional[int]:
        """Set a nogo region for Spot using the MutateWorldObject service

        Args:
            box_x, box_y: Dimensions of the nogo box, in meters
            frame_name: Frame that the nogo box is located relative to
            frame_t_box: The transform from frame_name to the nogo box
            region_name: Name of the new world object created
            lifetime: How long the nogo region will persist, in seconds
        Returns:
            the id of the newly created world object, or None if the request failed
        """
        request = MutateWorldObjectRequest()
        request.mutation.action = MutateWorldObjectRequest.Action.ACTION_ADD
        # create the fake world object
        request.mutation.object.name = region_name
        # There is no "infinite lifetime" option. If it is unset, there is a default.
        # see https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#bosdyn-api-WorldObject
        request.mutation.object.object_lifetime.seconds = lifetime
        # set no-go box
        request.mutation.object.nogo_region_properties.box.box.size.x = box_x
        request.mutation.object.nogo_region_properties.box.box.size.y = box_y
        request.mutation.object.nogo_region_properties.box.frame_name = frame_name
        request.mutation.object.nogo_region_properties.box.frame_name_tform_box.CopyFrom(frame_t_box.to_proto())
        ros_msg = MutateRequestRos()
        convert(request, ros_msg)
        response = self._mutate_wo.synchronously(MutateWorldObject.Request(request=ros_msg), nothrow=True)
        print(f"Added a world object with ID {response.response.mutated_object_id}")
        # TODO check if the world object was added successfully
        if response.response.status.value == response.response.status.STATUS_OK:
            return response.response.mutated_object_id
        else:
            print(f"Adding the nogo region failed with response status {response.response.status.value}")
            return None

    def delete_all_user_nogo_regions(self) -> None:
        print("Deleting all user added nogo regions!")
        request = ListWorldObjectRequest()
        request.object_type.append(WorldObjectType.WORLD_OBJECT_USER_NOGO)
        ros_msg = ListRequestRos()
        convert(request, ros_msg)
        response = self._list_wo.synchronously(ListWorldObjects.Request(request=ros_msg))
        for wo in response.response.world_objects:
            self.delete_nogo_region(wo.id)

    def delete_nogo_region(self, nogo_id: int) -> bool:
        print(f"Deleting world object with ID {nogo_id}")
        request = MutateWorldObjectRequest()
        request.mutation.action = MutateWorldObjectRequest.Action.ACTION_DELETE
        # tell it to delete the world object with the given ID
        request.mutation.object.id = nogo_id
        # call service
        ros_msg = MutateRequestRos()
        convert(request, ros_msg)
        response = self._mutate_wo.synchronously(MutateWorldObject.Request(request=ros_msg))
        if response.response.status.value == response.response.status.STATUS_OK:
            return True
        else:
            print(f"Deleting the nogo region failed with response status {response.response.status.value}")
            return False

    def try_walking_forward(self, x_offset: float) -> None:
        if not self._walk:
            print("Not attempting to walk forward")
            return None
        print("Try walking forward")
        world_t_goal = self.world_t_robot_init.get_closest_se2_transform() * SE2Pose(x=x_offset, y=0.0, angle=0.0)
        print(f"World t goal {world_t_goal}")
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=world_t_goal.x,
            goal_y=world_t_goal.y,
            goal_heading=world_t_goal.angle,
            frame_name=VISION_FRAME_NAME,
        )
        action_goal = RobotCommand.Goal()
        convert(proto_goal, action_goal.command)
        self._robot_command.synchronously(action_goal)
        self._logger.info("Successfully walked forward")


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    parser.add_argument("--walk", type=bool, default=False)
    parser.add_argument("--delete-first", type=bool, default=False)
    return parser


@ros_process.main(cli(), uses_tf=True)
def main(args: argparse.Namespace) -> int:
    print("Use the top down view on the tablet to visualize nogo regions!")
    nogo = NoGoRegion(args.robot, args.walk)
    nogo.initialize_robot()
    # list the world objects before we make any modification
    nogo.list_current_world_objects()
    if args.delete_first:
        nogo.delete_all_user_nogo_regions()
        nogo.list_current_world_objects()
    # add nogo region that is a 0.2mx2m box that is 0.5m in front of spot
    nogo_id = nogo.add_nogo_region(
        box_x=0.2,
        box_y=2.0,
        frame_name=VISION_FRAME_NAME,
        frame_t_box=nogo.world_t_robot_init * SE3Pose(x=1.0, y=0.0, z=0.0, rot=Quat()),
    )
    if not nogo_id:
        return 0
    # list objects to see if it was added successfully
    nogo.list_current_world_objects()
    input("Press enter when ready to walk forward (if requested) and then delete")
    # try walking forward
    nogo.try_walking_forward(x_offset=1.0)
    # if the nogo region was set successfully, delete it
    nogo.delete_nogo_region(nogo_id)
    # list objects one final time to make sure that it was deleted
    nogo.list_current_world_objects()
    # try walking forward again
    nogo.try_walking_forward(x_offset=2.0)
    return 0


if __name__ == "__main__":
    exit(main())
