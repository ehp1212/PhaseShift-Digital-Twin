import struct
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from phaseshift_interfaces.msg import (
    DetectedObjectArray,
    EstimatedObject,
    EstimatedObjectArray,
)


class VoxelStateEstimatorNode(LifecycleNode):
    """
    VoxelStateEstimatorNode

    Role:
    - Subscribe projected 3D semantic objects from perception pipeline
    - Subscribe voxel change points from perception_geometry
    - Estimate object-level voxel state around each detected object
    - Publish EstimatedObjectArray for downstream memory / tracking / nav adapter

    Important semantic note:
    - `is_dynamic` here means "significant change observed around the object"
      rather than guaranteed physical motion truth.
    """

    MIN_DYNAMIC_POINTS = 10
    RADIUS = 1.0
    CONFIDENCE_SCALE = 20.0

    def __init__(self):
        super().__init__("voxel_state_estimator_node")

        # -----------------------------
        # Internal cache
        # -----------------------------
        self._change_points_np = None   # Nx3 float32 array
        self._last_header = None

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self._obj_sub = None
        self._change_points_sub = None
        self._estimated_objects_pub = None

        # -----------------------------
        # QoS
        # -----------------------------
        self._qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._qos_output = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.get_logger().info("VoxelStateEstimatorNode created")

    # -----------------------------
    # Lifecycle
    # -----------------------------
    def on_configure(self, state):
        self.get_logger().info("[Lifecycle] Configuring VoxelStateEstimatorNode...")

        try:
            self._obj_sub = self.create_subscription(
                DetectedObjectArray,
                "/perception/objects_3d",
                self._on_object_sub_callback,
                self._qos_output,
            )

            self._change_points_sub = self.create_subscription(
                PointCloud2,
                "/perception_geometry/change_points",
                self._on_change_points_sub_callback,
                self._qos_sensor,
            )

            self._estimated_objects_pub = self.create_publisher(
                EstimatedObjectArray,
                "/perception/estimated_objects",
                self._qos_output,
            )

            self.get_logger().info("[Lifecycle] Configured successfully")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"[Lifecycle] Configure failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info("[Lifecycle] Activating VoxelStateEstimatorNode...")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("[Lifecycle] Deactivating VoxelStateEstimatorNode...")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("[Lifecycle] Cleaning up VoxelStateEstimatorNode...")

        try:
            if self._obj_sub is not None:
                self.destroy_subscription(self._obj_sub)
                self._obj_sub = None

            if self._change_points_sub is not None:
                self.destroy_subscription(self._change_points_sub)
                self._change_points_sub = None

            if self._estimated_objects_pub is not None:
                self.destroy_publisher(self._estimated_objects_pub)
                self._estimated_objects_pub = None

            self._change_points_np = None
            self._last_header = None

            self.get_logger().info("[Lifecycle] Cleanup completed")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"[Lifecycle] Cleanup failed: {e}")
            return TransitionCallbackReturn.FAILURE

    # -----------------------------
    # Callbacks
    # -----------------------------
    def _on_object_sub_callback(self, msg: DetectedObjectArray):
        if self._estimated_objects_pub is None:
            return

        if self._change_points_np is None:
            return

        output = EstimatedObjectArray()
        output.header = msg.header

        for obj in msg.objects:
            center = obj.pose.position

            nearby_points = self._query_points_radius(
                center=center,
                radius=self.RADIUS,
            )

            state = self._estimate_state(center, nearby_points)

            estimated = EstimatedObject()

            # Base semantic detection
            estimated.class_id = obj.class_id
            estimated.confidence = obj.confidence
            estimated.pose = obj.pose

            # Voxel-based state
            estimated.is_dynamic = state["is_dynamic"]
            estimated.motion_confidence = state["motion_confidence"]
            estimated.dynamic_point_count = float(state["num_points"])
            estimated.estimated_distance = state["distance"]

            estimated.estimated_size.x = float(state["size"][0])
            estimated.estimated_size.y = float(state["size"][1])
            estimated.estimated_size.z = float(state["size"][2])

            output.objects.append(estimated)

        self._estimated_objects_pub.publish(output)

    def _on_change_points_sub_callback(self, msg: PointCloud2):
        self._last_header = msg.header
        self._change_points_np = self._pointcloud2_to_xyz_array(msg)

    # -----------------------------
    # Internal
    # -----------------------------
    def _pointcloud2_to_xyz_array(self, cloud_msg: PointCloud2) -> np.ndarray:
        """
        Convert PointCloud2 -> Nx3 numpy array.

        Assumption:
        - x, y, z are the first 3 float32 fields in the point step layout.

        For future optimization:
        - replace with numpy.frombuffer or structured dtype parsing.
        """
        points = []

        for offset in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x, y, z = struct.unpack_from("fff", cloud_msg.data, offset=offset)
            points.append([x, y, z])

        if not points:
            return np.empty((0, 3), dtype=np.float32)

        return np.array(points, dtype=np.float32)

    def _query_points_radius(self, center, radius: float) -> np.ndarray:
        """
        Return change points within a spherical radius around object center.
        """
        if self._change_points_np is None or len(self._change_points_np) == 0:
            return np.empty((0, 3), dtype=np.float32)

        center_np = np.array([center.x, center.y, center.z], dtype=np.float32)

        diff = self._change_points_np - center_np
        dist = np.linalg.norm(diff, axis=1)

        mask = dist < radius
        return self._change_points_np[mask]

    def _estimate_state(self, center, nearby_points: np.ndarray) -> dict:
        """
        Estimate voxel-based object state.

        Outputs:
        - num_points
        - is_dynamic
        - motion_confidence
        - distance
        - size

        Semantic note:
        - is_dynamic here means "significant change observed"
          rather than physically verified motion.
        """
        num_points = len(nearby_points)

        # 1) Significant change around object
        is_dynamic = num_points > self.MIN_DYNAMIC_POINTS

        # 2) Normalized confidence from local change density
        motion_confidence = float(
            min(1.0, num_points / self.CONFIDENCE_SCALE)
        )

        # 3) Distance estimate
        # Current definition:
        # mean distance from object center to nearby change points
        # This is more meaningful than global-origin distance for this node.
        if num_points > 0:
            center_np = np.array([center.x, center.y, center.z], dtype=np.float32)
            local_diff = nearby_points - center_np
            distances = np.linalg.norm(local_diff, axis=1)
            estimated_distance = float(np.mean(distances))
        else:
            estimated_distance = 0.0

        # 4) Axis-aligned bounding box size
        if num_points > 0:
            min_vals = np.min(nearby_points, axis=0)
            max_vals = np.max(nearby_points, axis=0)
            size = max_vals - min_vals
        else:
            size = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        return {
            "num_points": num_points,
            "is_dynamic": is_dynamic,
            "motion_confidence": motion_confidence,
            "distance": estimated_distance,
            "size": size,
        }


def main(args=None):
    rclpy.init(args=args)

    node = VoxelStateEstimatorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()