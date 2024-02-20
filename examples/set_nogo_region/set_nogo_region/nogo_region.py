import argparse
import logging
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.utilities import fqn, namespace_with
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn_msgs.msg import (
    MutateWorldObjectRequestAction,
    MutateWorldObjectResponseStatus,
    NoGoRegionPropertiesOneOfRegion,
    WorldObject,
)
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from utilities.simple_spot_commander import SimpleSpotCommander
from utilities.tf_listener_wrapper import TFListenerWrapper

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.srv import ListWorldObjects, MutateWorldObject  # type: ignore

# Where we are placing the NOGO region relative to the starting pose of the robot
ROBOT_T_NOGO = SE2Pose(1.0, 0.0, 0.0)

# relevant example from the spot SDK:
# https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/user_nogo_regions/user_nogo_regions.py


class NoGoRegion:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self.node = node
        self._robot_name = robot_name

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)

        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )
        self._list_wo_client = node.create_client(
            ListWorldObjects, namespace_with(self._robot_name, "list_world_objects")
        )

        self._mutuate_wo_client = node.create_client(
            MutateWorldObject, namespace_with(self._robot_name, "mutate_world_object")
        )

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
        return True

    def list_current_world_objects(self) -> None:
        request = ListWorldObjects.Request()
        print("Current world objects:")
        future = self._list_wo_client.call_async(request)
        if not wait_for_future(future, context=self.node.context):
            return
        response = future.result()
        for wo in response.response.world_objects:
            print(f"\tID: {wo.id} name: {wo.name}")
            if wo.nogo_region_properties_is_set:
                print(wo.nogo_region_properties)

    def add_nogo_region(
        self,
        box_x: float,
        box_y: float,
        frame_name: str,
        frame_t_box: Pose,
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
        request = MutateWorldObject.Request()
        request.request.mutation.action.value = MutateWorldObjectRequestAction.ACTION_ADD
        request.request.mutation_is_set = True  # TODO If this is false, driver will crash. should handle in driver?
        # create the fake world object
        wo = WorldObject()
        wo.name = region_name
        # There is no "infinite lifetime" option. If it is unset, there is a default.
        # see https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#bosdyn-api-WorldObject
        wo.object_lifetime.sec = lifetime
        wo.object_lifetime_is_set = True
        # set no-go box
        wo.nogo_region_properties.region.box.box.size.x = box_x
        wo.nogo_region_properties.region.box.box.size.y = box_y
        wo.nogo_region_properties.region.box.box.size_is_set = True
        wo.nogo_region_properties.region.box.frame_name = frame_name
        wo.nogo_region_properties.region.box.frame_name_tform_box = frame_t_box
        wo.nogo_region_properties.region.box.frame_name_tform_box_is_set = True
        wo.nogo_region_properties.region.box.box_is_set = True
        # final flags
        wo.nogo_region_properties.region.region_choice = NoGoRegionPropertiesOneOfRegion.REGION_BOX_SET
        wo.nogo_region_properties_is_set = True
        # attach the object to the request
        request.request.mutation.object = wo
        request.request.mutation.object_is_set = True
        # call request
        future = self._mutuate_wo_client.call_async(request)
        if not wait_for_future(future, context=self.node.context):
            return None
        response = future.result()
        print(f"Added a world object with ID {response.response.mutated_object_id}")
        # TODO check if the world object was added successfully
        if response.response.status.value == MutateWorldObjectResponseStatus.STATUS_OK:
            return response.response.mutated_object_id
        else:
            print(f"Adding the nogo region failed with response status {response.response.status.value}")
            return None

    def delete_nogo_region(self, nogo_id: int) -> bool:
        print(f"Deleting world object with ID {nogo_id}")
        request = MutateWorldObject.Request()
        request.request.mutation.action.value = MutateWorldObjectRequestAction.ACTION_DELETE
        request.request.mutation_is_set = True  # TODO If this is false, driver will crash. should handle in driver
        # tell it to delete the world object with the given ID
        request.request.mutation.object.id = nogo_id
        request.request.mutation.object_is_set = True
        # call service
        future = self._mutuate_wo_client.call_async(request)
        if not wait_for_future(future, context=self.node.context):
            return False
        response = future.result()
        if response.response.status.value == MutateWorldObjectResponseStatus.STATUS_OK:
            return True
        else:
            print(f"Deleting the nogo region failed with response status {response.response.status.value}")
            return False


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    nogo = NoGoRegion(args.robot, main.node)
    nogo.initialize_robot()
    # list the world objects before we make any modification
    nogo.list_current_world_objects()
    # add nogo region that is a 1x1 box that is 2m in front of Spot's vision frame.
    nogo_id = nogo.add_nogo_region(
        box_x=1.0, box_y=1.0, frame_name=VISION_FRAME_NAME, frame_t_box=Pose(position=Point(x=2.0))
    )
    # list objects to see if it was added successfully
    nogo.list_current_world_objects()
    if nogo_id:
        # if the nogo region was set successfully, delete it
        nogo.delete_nogo_region(nogo_id)
        # list objects one final time to make sure that it was deleted
        nogo.list_current_world_objects()
    return 0


if __name__ == "__main__":
    exit(main())
