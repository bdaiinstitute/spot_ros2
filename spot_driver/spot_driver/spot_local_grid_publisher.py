#!/usr/bin/env python3
from typing import List, Optional

import numpy as np
import rclpy
import ros2_numpy as rnp
from bosdyn.api.local_grid_pb2 import LocalGrid
from bosdyn.client import create_standard_sdk
from bosdyn.client.common import FutureWrapper
from bosdyn.client.frame_helpers import GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.robot_state import RobotStateClient
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from spot_driver.manual_conversions import se3_pose_to_ros_pose
from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param

VALID_GRIDS = ["terrain", "terrain_valid", "intensity", "no_step", "obstacle_distance"]


class LocalGridPublisher(Node):
    def __init__(self) -> None:
        super().__init__("local_grid_publisher")
        self.get_logger().debug("Initializing LocalGridPublisher Node...")

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("local_grid_name", "obstacle_distance", read_only)
        self.grid_name = self.get_parameter("local_grid_name").value

        # Verify the requested grid name is an actual grid name
        if self.grid_name not in VALID_GRIDS:
            self.get_logger().error(f'Requested grid "{self.grid_name}" is not a valid local_grid type!')
            raise ValueError("Invalid local_grid name")

        # Get robot Credentials
        self.username: str = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_USERNAME", self, "username", "user")
        self.password: str = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_PASSWORD", self, "password", "password")
        self.ip: str = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")

        if not self.ip or not self.username or not self.password:
            self.get_logger().error("Robot credentials not found")
            raise ValueError("Robot credentials not found")

        # Verify the credentials work
        self.get_logger().debug("🧰 Creating SDK objects...")
        self.sdk = create_standard_sdk("local_grid_publisher")
        self.robot = self.sdk.create_robot(self.ip)
        self.get_logger().debug("🧰 Created SDK objects successfully!")

        self.get_logger().debug("🔐 Attempting authentication...")
        self.robot.authenticate(self.username, self.password)  # an exception will be raised if authentication fails
        self.get_logger().debug("🔐 Robot authenticated successfully!")

        self.get_logger().debug("🕰 Waiting for time sync...")
        self.robot.time_sync.wait_for_sync()
        self.get_logger().debug("🕰 Time sync successful!")

        # Create LocalGridClient
        self.get_logger().debug("Creating LocalGridClient...")
        self.local_grid_client = self.robot.ensure_client(LocalGridClient.default_service_name)
        self.get_logger().debug("LocalGridClient created successfully!")

        # Create RobotStateClient
        self.get_logger().debug("Creating RobotStateClient...")
        self.robot_State_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.get_logger().debug("RobotStateClient created successfully!")

        # Create ROS2 publisher
        self.get_logger().debug("📡 Creating OccupancyGrid publisher...")
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, "grid_topic_REMAP_ME", 10)
        self.get_logger().debug("📡 OccupancyGrid publisher created successfully!")

        # Indicate successful initialization
        self.get_logger().debug("[✓] Spot Local Grid Publisher Node initialized")

        # Set runtime variables
        self.first_draw_done = False
        self.im = None
        self.fig = None
        self.ax = None

        self.fetch_next_grid_data()

    def fetch_next_grid_data(self) -> None:
        future = self.local_grid_client.get_local_grids_async([self.grid_name])
        future.add_done_callback(self.publish_grid)

    def publish_grid(self, future: FutureWrapper) -> None:
        """
        Converts the local grid protobuf into a ROS occupancy grid message

        Depending on the requested grid, some conversions may be done, as ROS OccupancyGrid data must be in int8 format

        Code in this function is adapted from the Boston Dynamics Spot SDK basic_streaming_visualizer example
        """
        proto = future.result()
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == self.grid_name:
                local_grid_proto = local_grid_found

        local_grid_proto.local_grid.extent.num_cells_x * local_grid_proto.local_grid.extent.num_cells_y

        # Populate Grid data and convert datatype if necessary

        raw_cells = self.unpack_grid(local_grid_proto)

        # "terrain_valid" and "intensity" grid protos are uint8
        if raw_cells.dtype == np.uint8:
            converted_cells = (raw_cells.astype(np.int16) - 128).astype(
                np.int8
            )  # Subtract a bias value so values remain correctly relative to each other

        # "terrain", "no_step", and "obstacle_distance" grid protos are int16
        elif raw_cells.dtype == np.int16:
            # Not much we can do to avoid losing some information - the "no_step" grid is booleans so it
            # shouldn't be an issue, but "terrain" and "obstacle_distance" values might be clipped
            converted_cells = raw_cells.astype(np.int8)

        grid = converted_cells.reshape(
            local_grid_proto.local_grid.extent.num_cells_y, local_grid_proto.local_grid.extent.num_cells_x
        )

        grid_msg = rnp.msgify(OccupancyGrid, grid)  # Grid data converted using ros2_numpy
        grid_msg.header.frame_id = VISION_FRAME_NAME

        # Timestamp data from protobuf
        grid_msg.header.stamp.sec = local_grid_proto.local_grid.acquisition_time.seconds
        grid_msg.header.stamp.nanosec = local_grid_proto.local_grid.acquisition_time.nanos
        grid_msg.info.map_load_time.sec = local_grid_proto.local_grid.acquisition_time.seconds
        grid_msg.info.map_load_time.nanosec = local_grid_proto.local_grid.acquisition_time.nanos

        # Spatial information
        grid_msg.info.resolution = local_grid_proto.local_grid.extent.cell_size
        transform = get_a_tform_b(
            local_grid_proto.local_grid.transforms_snapshot,
            VISION_FRAME_NAME,
            local_grid_proto.local_grid.frame_name_local_grid_data,
        )

        # Set grid Z-position to ground plane's Z-position
        vision_frame_ground_z = self.compute_ground_height_in_vision_frame()
        transform.z = vision_frame_ground_z

        grid_msg.info.origin = se3_pose_to_ros_pose(transform)

        # Publish and begin the next fetch
        self.occupancy_grid_pub.publish(grid_msg)
        self.fetch_next_grid_data()

    # Helper functions for local grid processing - functions taken from Bosdyn Dynamics Spot SDK visualizer example
    def unpack_grid(self, local_grid_proto: LocalGrid) -> np.array:
        """Unpack the local grid proto."""
        # Determine the data type for the bytes data.
        data_type = self.get_numpy_data_type(local_grid_proto.local_grid)
        if data_type is None:
            print("Cannot determine the dataformat for the local grid.")
            return None
        # Decode the local grid.
        if local_grid_proto.local_grid.encoding == LocalGrid.ENCODING_RAW:
            full_grid = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        elif local_grid_proto.local_grid.encoding == LocalGrid.ENCODING_RLE:
            full_grid = self.expand_data_by_rle_count(local_grid_proto, data_type=data_type)
        else:
            # Return nothing if there is no encoding type set.
            return None
        # Apply the offset and scaling to the local grid.
        if local_grid_proto.local_grid.cell_value_scale == 0:
            return full_grid
        full_grid_float = full_grid.astype(np.float64)
        full_grid_float *= local_grid_proto.local_grid.cell_value_scale
        full_grid_float += local_grid_proto.local_grid.cell_value_offset
        return full_grid_float

    def get_numpy_data_type(self, local_grid_proto: LocalGrid) -> np.dtype:
        """Convert the cell format of the local grid proto to a numpy data type."""
        if local_grid_proto.cell_format == LocalGrid.CELL_FORMAT_UINT16:
            return np.uint16
        elif local_grid_proto.cell_format == LocalGrid.CELL_FORMAT_INT16:
            return np.int16
        elif local_grid_proto.cell_format == LocalGrid.CELL_FORMAT_UINT8:
            return np.uint8
        elif local_grid_proto.cell_format == LocalGrid.CELL_FORMAT_INT8:
            return np.int8
        elif local_grid_proto.cell_format == LocalGrid.CELL_FORMAT_FLOAT64:
            return np.float64
        elif local_grid_proto.cell_format == LocalGrid.CELL_FORMAT_FLOAT32:
            return np.float32
        else:
            return None

    def expand_data_by_rle_count(self, local_grid_proto: LocalGrid, data_type: np.dtype = np.int16) -> np.array:
        """Expand local grid data to full bytes data using the RLE count."""
        cells_pz = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        cells_pz_full = []
        # For each value of rle_counts, we expand the cell data at the matching index
        # to have that many repeated, consecutive values.
        for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
            for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
                cells_pz_full.append(cells_pz[i])
        return np.array(cells_pz_full)

    def compute_ground_height_in_vision_frame(
        self,
    ) -> float:  # technically a double because python's float is double-precision
        """Get the z-height of the ground plane in vision frame from the current robot state."""
        robot_state = self.robot_state_client.get_robot_state()
        vision_tform_ground_plane = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot, VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME
        )
        return vision_tform_ground_plane.position.z


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    try:
        node = LocalGridPublisher()
    except Exception:
        rclpy.shutdown()
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
