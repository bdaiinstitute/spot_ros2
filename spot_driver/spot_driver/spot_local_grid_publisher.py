#!/usr/bin/env python3
import os
import rclpy
import numpy as np
import ros2_numpy as rnp
from bosdyn.api import local_grid_pb2
from bosdyn.client import create_standard_sdk
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.math_helpers import SE3Pose
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from spot_driver.manual_conversions import se3_pose_to_ros_pose
from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param

VALID_GRIDS = [
    "terrain",
    "terrain_valid",
    "intensity",
    "no_step",
    "obstacle_distance"
]

class LocalGridPublisher(Node):

    def __init__(self):
        super().__init__('local_grid_publisher')
        self.get_logger().debug("Initializing LocalGridPublisher Node...")

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter('local_grid_name', 'obstacle_distance', read_only)
        self.grid_name = self.get_parameter('local_grid_name').value
        

        # Verify the requested grid name is a real one

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
        self.sdk = create_standard_sdk('local_grid_publisher')
        self.robot = self.sdk.create_robot(self.ip)
        self.get_logger().debug("🧰 Created SDK objects successfully!")

        self.get_logger().debug("🔐 Attempting authentication...")
        self.robot.authenticate(self.username, self.password)   # an exception will be raised if authentication fails
        self.get_logger().debug("🔐 Robot authenticated successfully!")
        
        self.get_logger().debug("🕰 Waiting for time sync...")
        self.robot.time_sync.wait_for_sync()
        self.get_logger().debug("🕰 Time sync successful!")


        # Create LocalGridClient

        self.get_logger().debug("Creating LocalGridClient...")
        self.local_grid_client = self.robot.ensure_client(LocalGridClient.default_service_name)
        self.get_logger().debug("LocalGridClient created successfully!")


        # Create ROS2 publisher

        self.get_logger().debug("📡 Creating OccupancyGrid publisher...")
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, 'grid_topic_REMAP_ME', 10)
        self.get_logger().debug("📡 OccupancyGrid publisher created successfully!")


        # Indicate successful initialization

        self.get_logger().debug('[✓] Spot Local Grid Publisher Node initialized')


        # Set runtime variables

        self.first_draw_done = False
        self.im = None
        self.fig = None
        self.ax = None

        self.fetch_next_grid_data()

    
    def fetch_next_grid_data(self):

        future = self.local_grid_client.get_local_grids_async([self.grid_name])
        future.add_done_callback(self.publish_grid)


    def publish_grid(self, future):
        """
        Converts the local grid protobuf into a ROS occupancy grid message

        Depending on the requested grid, some conversions may be done, as ROS OccupancyGrid data must be in int8 format

        Code in this function is adapted from the Boston Dynamics Spot SDK basic_streaming_visualizer example
        """
        proto = future.result()
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == self.grid_name:
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        cell_count = local_grid_proto.local_grid.extent.num_cells_x * local_grid_proto.local_grid.extent.num_cells_y
        

        # Populate Grid data and convert datatype if necessary

        raw_cells = self.unpack_grid(local_grid_proto)

        # "terrain_valid" and "intensity" grid protos are uint8
        if raw_cells.dtype == np.uint8:
            converted_cells = (raw_cells.astype(np.int16) - 128).astype(np.int8)    # Subtract a bias value so values remain correctly relative to each other

        # "terrain", "no_step", and "obstacle_distance" grid protos are int16
        elif raw_cells.dtype == np.int16:
            # Not much we can do to avoid losing some information - the "no_step" grid is booleans so it shouldn't be an issue, but "terrain" and "obstacle_distance" values might be clipped 
            converted_cells = raw_ells.astype(np.int8)

        grid = converted_cells.reshape(local_grid_proto.local_grid.extent.num_cells_y, local_grid_proto.local_grid.extent.num_cells_x)

        grid_msg = rnp.msgify(OccupancyGrid, grid)  # Grid data converted using ros2_numpy
        grid_msg.header.frame_id = VISION_FRAME_NAME

        # Timestamp data from protobuf
        grid_msg.header.stamp.sec = local_grid_proto.local_grid.acquisition_time.seconds
        grid_msg.header.stamp.nanosec = local_grid_proto.local_grid.acquisition_time.nanos
        grid_msg.info.map_load_time.sec = local_grid_proto.local_grid.acquisition_time.seconds
        grid_msg.info.map_load_time.nanosec = local_grid_proto.local_grid.acquisition_time.nanos

        # Spatial information
        grid_msg.info.resolution = local_grid_proto.local_grid.extent.cell_size
        transform = self.get_a_tform_b(local_grid_proto.local_grid.transforms_snapshot, VISION_FRAME_NAME,
                           local_grid_proto.local_grid.frame_name_local_grid_data)
        
        grid_msg.info.origin = se3_pose_to_ros_pose(transform)


        # Publish and begin the next fetch
        self.occupancy_grid_pub.publish(grid_msg)
        self.fetch_next_grid_data()

    
    # Helper functions for local grid processing - functions taken from Bosdyn Dynamics Spot SDK visualizer example
    def unpack_grid(self, local_grid_proto):
        """Unpack the local grid proto."""
        # Determine the data type for the bytes data.
        data_type = self.get_numpy_data_type(local_grid_proto.local_grid)
        if data_type is None:
            print('Cannot determine the dataformat for the local grid.')
            return None
        # Decode the local grid.
        if local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
            full_grid = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        elif local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
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


    def get_numpy_data_type(self, local_grid_proto):
        """Convert the cell format of the local grid proto to a numpy data type."""
        if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
            return np.uint16
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
            return np.int16
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
            return np.uint8
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
            return np.int8
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
            return np.float64
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
            return np.float32
        else:
            return None
        

    def expand_data_by_rle_count(self, local_grid_proto, data_type=np.int16):
        """Expand local grid data to full bytes data using the RLE count."""
        cells_pz = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        cells_pz_full = []
        # For each value of rle_counts, we expand the cell data at the matching index
        # to have that many repeated, consecutive values.
        for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
            for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
                cells_pz_full.append(cells_pz[i])
        return np.array(cells_pz_full)


    def get_a_tform_b(self, frame_tree_snapshot, frame_a, frame_b):
        """Get the SE(3) pose representing the transform between frame_a and frame_b.

        Using frame_tree_snapshot, find the math_helpers.SE3Pose to transform geometry from
        frame_a's representation to frame_b's.

        Args:
            frame_tree_snapshot (dict) dictionary representing the child_to_parent_edge_map
            frame_a (string)
            frame_b (string)
            validate (bool) if the FrameTreeSnapshot should be checked for a valid tree structure

        Returns:
            math_helpers.SE3Pose between frame_a and frame_b if they exist in the tree. None otherwise.
        """

        if frame_a not in frame_tree_snapshot.child_to_parent_edge_map:
            return None
        if frame_b not in frame_tree_snapshot.child_to_parent_edge_map:
            return None

        def _list_parent_edges(leaf_frame):
            parent_edges = []
            cur_frame = leaf_frame
            while True:
                parent_edge = frame_tree_snapshot.child_to_parent_edge_map.get(cur_frame)
                if not parent_edge.parent_frame_name:
                    break
                parent_edges.append(parent_edge)
                cur_frame = parent_edge.parent_frame_name
            return parent_edges

        inverse_edges = _list_parent_edges(frame_a)
        forward_edges = _list_parent_edges(frame_b)

        # Possible optimization: Nearest common ancestor pruning.

        def _accumulate_transforms(parent_edges):
            ret = SE3Pose.from_identity()
            for parent_edge in parent_edges:
                ret = SE3Pose.from_proto(parent_edge.parent_tform_child) * ret
            return ret

        frame_a_tform_root_frame = _accumulate_transforms(inverse_edges).inverse()
        root_frame_tform_frame_b = _accumulate_transforms(forward_edges)
        return frame_a_tform_root_frame * root_frame_tform_frame_b


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocalGridPublisher()
    except Exception as e:
        rclpy.shutdown()
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
