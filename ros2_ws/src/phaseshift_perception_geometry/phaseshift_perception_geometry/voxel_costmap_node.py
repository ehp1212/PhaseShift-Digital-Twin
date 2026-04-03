import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

import numpy as np
import open3d as o3d
from scipy.ndimage import distance_transform_edt
from rclpy.qos import QoSProfile, DurabilityPolicy

"""
[SLAM]
VoxelMapNode → voxel_map.pcd

[Interpretation Layer]
VoxelCostmapNode
  → /voxel_occupancy
  → /spatial_cost_field

[Navigation]
Nav2 StaticLayer ← /spatial_cost_field

✔ voxel-based spatial abstraction
✔ obstacle distance-aware cost shaping
✔ navigation behavior modification (wall avoidance)\
"""

class VoxelCostmapNode(LifecycleNode):

    def __init__(self):
        super().__init__('voxel_costmap_node')
        self._log_info_label('VOXEL COSTMAP NODE', "started...")
        
        # -------------------------
        # VOXEL MAP
        # -------------------------
        self.declare_parameter('voxel_map_path', '')
        self._voxel_map_path = None
        self.map_loaded = False

        # config
        self.declare_parameter("z_min", 0.1)
        self.declare_parameter("z_max", 1.5)
        self.declare_parameter("influence_distance", 1.0)
        self.declare_parameter("cost_power", 2.0)

        self._resolution = 0.05
        self.max_cost = 100

        self._voxel_points = []
        self._voxel_occupancy_cells = None
        self._voxel_occupancy_msg = None
        self._spatial_cost_field_msg = None

    # ==============================
    # LIFECYCLE
    # ==============================
    def on_configure(self, state):

        self._log_info_label('VOXEL COSTMAP NODE', "configured successfully")
        self.voxel_map_path = self.get_parameter('voxel_map_path').value

        if not self.voxel_map_path:
            self._log_info_label('VOXEL COSTMAP NODE', "No voxel_map_path provided...")
            return TransitionCallbackReturn.FAILURE

        self._log_info_label('VOXEL COSTMAP NODE', f"{self.voxel_map_path}")
        success = self._load_voxel_map(self.voxel_map_path)
        if not success:
            self._log_info_label('VOXEL COSTMAP NODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE

        self._z_min = self.get_parameter("z_min").value
        self._z_max = self.get_parameter("z_max").value
        self._influence_distance = self.get_parameter("influence_distance").value
        self._cost_power = self.get_parameter("cost_power").value

        self._log_info_label("SPATIAL COST FIELD", f"z_range = ({self._z_min}, {self._z_max})")

        self._voxel_occupancy_cells = self._build_2d_occupancy()
        if not self._voxel_occupancy_cells:
            self.get_logger().error("No occupied cells after projection")
            return TransitionCallbackReturn.FAILURE
        
        self._voxel_occupancy_msg = self._build_grid_msg(self._voxel_occupancy_cells)
        self._spatial_cost_field_msg = self._build_spatial_cost_field_msg(self._voxel_occupancy_cells)

        self.map_loaded = True
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state):

        if not self.map_loaded:
            self._log_info_label('VOXEL COSTMAP NODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE

        # transient_local qos
        grid_qos = QoSProfile(depth=1)
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._voxel_occupancy_pub = self.create_publisher(
            OccupancyGrid,
            '/voxel_occupancy',
            grid_qos
        )

        self._spatial_cost_field_pub = self.create_publisher(
            OccupancyGrid,
            '/spatial_cost_field',
            grid_qos
        )

        self._publish_once()

        self.timer = self.create_timer(
            3.0,
            self._publish_once
        )

        self._log_info_label('VOXEL COSTMAP NODE', "activated successfully")
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):

        self.get_logger().info("[VOXEL] deactivating...")

        if hasattr(self, 'timer'):
            self.timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        return super().on_cleanup(state)

    # ==============================
    # PUBLIC API
    # ==============================
    def set_voxel_map_path(self, map_path: str):
        if not map_path:
            self.get_logger().error("Map path is invalid.")
            return
        
        self._voxel_map_path = map_path
    
    # ==============================
    # INTERNAL
    # ==============================

    def _load_voxel_map(self, path):

        try:
            self._log_info_label('VOXEL COSTMAP NODE', f"Loading PCD: {path}")

            pcd = o3d.io.read_point_cloud(path)

            if len(pcd.points) == 0:
                self.get_logger().error("Empty PCD file")
                return False
            
            points = pcd.points

            # intentsity
            if pcd.has_colors():
                colors = pcd.colors
            else:
                colors = None

            self._voxel_points = []

            for i in range(len(points)):
                x, y, z = points[i]

                # confidence 추정
                if colors is not None:
                    # grayscale 기반 (R=G=B assumed)
                    conf = colors[i][0]
                else:
                    conf = 1.0  # fallback

                self._voxel_points.append((x, y, z, conf))

            self._log_info_label('VOXEL COSTMAP NODE', f"Loaded {len(self._voxel_points)} points")

            return True
        
        except Exception as e:
            self.get_logger().error(f"Load failed: {e}")
            return False
        
    def _build_2d_occupancy(self):
        occupied = set()

        for x, y, z, conf in self._voxel_points:
            if z < self._z_min or z > self._z_max:
                continue

            ix = int(x / self._resolution)
            iy = int(y / self._resolution)

            # dilation for near pixels
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    occupied.add((ix + dx, iy + dy))
            
        return occupied
    
    def _compute_bounds(self, occupied):

        xs = [ix for ix, _ in occupied]
        ys = [iy for _, iy in occupied]

        return min(xs), max(xs), min(ys), max(ys)
    
    def _build_grid_msg(self, occupied):

        min_x, max_x, min_y, max_y = self._compute_bounds(occupied)

        width = max_x - min_x + 1
        height = max_y - min_y + 1

        grid = [-1] * (width * height)

        for ix, iy in occupied:

            x = ix - min_x
            y = iy - min_y

            index = y * width + x
            grid[index] = 100

        msg = OccupancyGrid()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self._resolution
        msg.info.width = width
        msg.info.height = height

        msg.info.origin = Pose()
        msg.info.origin.position.x = min_x * self._resolution
        msg.info.origin.position.y = min_y * self._resolution

        msg.data = grid

        return msg

    def _build_spatial_cost_field_msg(self, occupied):
        min_x, max_x, min_y, max_y = self._compute_bounds(occupied)

        width = max_x - min_x + 1
        height = max_y - min_y + 1

        # 1 = free, 0 = occupied
        obstacle_mask = np.ones((height, width), dtype=np.uint8)

        for ix, iy in occupied:
            x = ix - min_x
            y = iy - min_y
            obstacle_mask[y, x] = 0

        # distance to obstacles
        distance_cells = distance_transform_edt(obstacle_mask)

        # to meter
        distance_m = distance_cells * self._resolution

        # limit influence range
        clipped_distance = np.clip(
            distance_m,
            0.0,
            self._influence_distance
        )

        # dist = 0    -> cost = 100
        # dist = max  -> cost = 0
        normalized = 1.0 - (clipped_distance / self._influence_distance)

        # non-linear influence from occupied cells
        cost_field = (normalized ** self._cost_power) * self.max_cost
        cost_field = cost_field.astype(np.int8)
        
        # occupied is always 100
        for ix, iy in occupied:
            x = ix - min_x
            y = iy - min_y
            cost_field[y, x] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self._resolution
        msg.info.width = width
        msg.info.height = height

        msg.info.origin = Pose()
        msg.info.origin.position.x = min_x * self._resolution
        msg.info.origin.position.y = min_y * self._resolution

        msg.data = cost_field.flatten().tolist()
        return msg
 
    # ==============================
    # UTILS
    # ==============================

    def _log_info_label(self, label: str, msg: str):
        self.get_logger().info(f"\033[93m[{label}]\033[0m - {msg}")  # Green

    def _publish_once(self):
        if self._voxel_occupancy_msg is not None:
            self._voxel_occupancy_msg.header.stamp = self.get_clock().now().to_msg()
            self._voxel_occupancy_pub.publish(self._voxel_occupancy_msg)

        if self._spatial_cost_field_msg is not None:
            self._spatial_cost_field_msg.header.stamp = self.get_clock().now().to_msg()
            self._spatial_cost_field_pub.publish(self._spatial_cost_field_msg)

    def _cleanup(self):
        pass

def main():
    rclpy.init()
    node = VoxelCostmapNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try: 
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        try:
            node._cleanup()
        except Exception as e:
            node.get_logger().warn(f"Lifecycle shutdown failed: {e}")

        executor.shutdown()
        node.destroy_node()