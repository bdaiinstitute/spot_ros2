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
from bosdyn_msgs.msg import MutateWorldObjectRequestAction, MutateWorldObjectResponseStatus
from rclpy.node import Node
from utilities.simple_spot_commander import SimpleSpotCommander
from utilities.tf_listener_wrapper import TFListenerWrapper

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.srv import ListWorldObjects, MutateWorldObject  # type: ignore

# Where we want the robot to walk to relative to itself
ROBOT_T_GOAL = SE2Pose(1.0, 0.0, 0.0)

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
            print(f"ID: {wo.id} name: {wo.name}")
            # print out nogo properties
            # if wo.nogo_region_properties_is_set:
            #     print(f"\tNogo properties set to: {wo.nogo_region_properties}")

    def add_nogo_region(self, name: str = "nogo_region", lifetime: int = 10) -> Optional[int]:
        request = MutateWorldObject.Request()
        request.request.mutation.action.value = MutateWorldObjectRequestAction.ACTION_ADD
        request.request.mutation_is_set = True  # TODO If this is false, driver will crash. should handle in driver
        request.request.mutation.object.name = name
        # TODO figure out how to make it persist infinitely
        request.request.mutation.object.object_lifetime.sec = lifetime
        request.request.mutation.object.object_lifetime_is_set = True
        # set no-go box, here it is a 1x1 region 2m in front of Spot
        request.request.mutation.object.nogo_region_properties.region.box.box.size.x = 1.0
        request.request.mutation.object.nogo_region_properties.region.box.box.size.y = 1.0
        request.request.mutation.object.nogo_region_properties.region.box.box.size_is_set = True
        # set box's offset from Spot
        request.request.mutation.object.nogo_region_properties.region.box.frame_name = VISION_FRAME_NAME
        request.request.mutation.object.nogo_region_properties.region.box.frame_name_tform_box.position.x = 2.0
        request.request.mutation.object.nogo_region_properties.region.box.frame_name_tform_box_is_set = True
        request.request.mutation.object.nogo_region_properties.region.box.box_is_set = True
        # final flags
        request.request.mutation.object.nogo_region_properties.region.region_choice = 1  # TODO 1=set 0=unset
        request.request.mutation.object.nogo_region_properties_is_set = True
        request.request.mutation.object_is_set = True
        request.request.mutation_is_set = True
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
            return False


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    nogo = NoGoRegion(args.robot, main.node)
    nogo.initialize_robot()
    nogo.list_current_world_objects()
    nogo_id = nogo.add_nogo_region()
    nogo.list_current_world_objects()
    if nogo_id:
        nogo.delete_nogo_region(nogo_id)
        nogo.list_current_world_objects()
    return 0


if __name__ == "__main__":
    exit(main())
