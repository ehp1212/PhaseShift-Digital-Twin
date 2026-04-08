import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from phaseshift_interfaces.msg import (
    DetectedObjectArray,
    EstimatedObject,
    EstimatedObjectArray,
    VoxelChangeArray,
)

class VoxelStateEstimatorNode(LifecycleNode):
    """
    This node enriches semantic detections using voxel-level geometric features.

    Pipeline:
    1. 3D object detections (from perception pipeline)
    2. VoxelChange feature input (dynamic score, temporal stability, confidence)
    3. Local voxel aggregation around each object
    4. Object-level state estimation (dynamic likelihood, size, density)

    Key Features:
    - Object-centric voxel feature aggregation
    - Fusion of geometry (voxel) and semantics (detection)
    - Robust dynamic estimation using temporal and spatial signals

    Design Philosophy:
    - Keep ROS2 as the authority for perception reasoning
    - Provide feature-rich outputs for downstream tracking and navigation
    - Separate geometric evidence from temporal state classification

    Output:
    - EstimatedObject with aggregated voxel features
    """

    RADIUS = 1.0
    DENSITY_THRESHOLD = 2.0

    def __init__(self):
        super().__init__("voxel_state_estimator_node")

        self._change_voxels = None
        self._last_header = None

        self._obj_sub = None
        self._change_info_sub = None
        self._estimated_objects_pub = None

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

        try:
            self._obj_sub = self.create_subscription(
                DetectedObjectArray,
                "/perception/objects_3d",
                self._on_object_sub_callback,
                self._qos_output,
            )

            self._change_info_sub = self.create_subscription(
                VoxelChangeArray,
                "/perception_geometry/change_points_info",
                self._on_change_info_sub_callback,
                self._qos_sensor,
            )

            self._estimated_objects_pub = self.create_publisher(
                EstimatedObjectArray,
                "/perception/estimated_objects",
                self._qos_output,
            )

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"[Configure failed] {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):

        try:
            if self._obj_sub:
                self.destroy_subscription(self._obj_sub)
                self._obj_sub = None

            if self._change_info_sub:
                self.destroy_subscription(self._change_info_sub)
                self._change_info_sub = None

            if self._estimated_objects_pub:
                self.destroy_publisher(self._estimated_objects_pub)
                self._estimated_objects_pub = None

            self._change_voxels = None

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"[Cleanup failed] {e}")
            return TransitionCallbackReturn.FAILURE

    # -----------------------------
    # Callbacks
    # -----------------------------
    def _on_change_info_sub_callback(self, msg: VoxelChangeArray):
        self._last_header = msg.header
        self._change_voxels = msg.voxels

    def _on_object_sub_callback(self, msg: DetectedObjectArray):

        if self._estimated_objects_pub is None:
            return

        if self._change_voxels is None:
            return

        output = EstimatedObjectArray()
        output.header = msg.header

        for obj in msg.objects:
            center = obj.pose.position

            nearby_voxels = self._query_voxels_radius(center)

            state = self._estimate_state(center, nearby_voxels)

            estimated = EstimatedObject()

            # -----------------------------
            # semantic base
            # -----------------------------
            estimated.class_id = obj.class_id
            estimated.confidence = obj.confidence
            estimated.pose = obj.pose

            estimated.is_dynamic = state["is_dynamic"]
            estimated.motion_confidence = state["motion_confidence"]
            estimated.dynamic_point_count = float(state["num_voxels"])
            estimated.density = float(state["density"])
            estimated.estimated_distance = state["distance"]

            estimated.estimated_size.x = float(state["size"][0])
            estimated.estimated_size.y = float(state["size"][1])
            estimated.estimated_size.z = float(state["size"][2])

            # -----------------------------
            # voxel info feature
            # -----------------------------
            estimated.support_voxel_count = float(state["num_voxels"])
            estimated.mean_dynamic_score = state["mean_dynamic_score"]
            estimated.max_dynamic_score = state["max_dynamic_score"]
            estimated.mean_temporal_stability = state["mean_temporal_stability"]
            estimated.mean_static_confidence = state["mean_static_confidence"]

            output.objects.append(estimated)

        self._estimated_objects_pub.publish(output)

    # -----------------------------
    # Internal
    # -----------------------------
    def _query_voxels_radius(self, center):

        if not self._change_voxels:
            return []

        cx, cy, cz = center.x, center.y, center.z
        nearby = []

        for v in self._change_voxels:
            dx = v.position.x - cx
            dy = v.position.y - cy
            dz = v.position.z - cz

            if dx*dx + dy*dy + dz*dz < self.RADIUS * self.RADIUS:
                nearby.append(v)

        return nearby

    def _estimate_state(self, center, nearby_voxels):

        num_voxels = len(nearby_voxels)

        volume = (self.RADIUS * 2.0) ** 3
        density = num_voxels / volume if volume > 0 else 0.0

        if num_voxels == 0:
            return {
                "num_voxels": 0,
                "density": 0.0,
                "is_dynamic": False,
                "motion_confidence": 0.0,
                "distance": 0.0,
                "size": np.array([0.0, 0.0, 0.0], dtype=np.float32),

                "mean_dynamic_score": 0.0,
                "max_dynamic_score": 0.0,
                "mean_temporal_stability": 0.0,
                "mean_static_confidence": 0.0,
            }

        positions = []
        dynamic_scores = []
        temporal_stabilities = []
        static_confidences = []

        cx, cy, cz = center.x, center.y, center.z

        for v in nearby_voxels:
            px = v.position.x
            py = v.position.y
            pz = v.position.z

            positions.append([px, py, pz])
            dynamic_scores.append(v.dynamic_score)
            temporal_stabilities.append(v.temporal_stability)
            static_confidences.append(v.static_confidence)

        positions = np.array(positions, dtype=np.float32)

        # -----------------------------
        # aggregation
        # -----------------------------
        mean_dynamic_score = float(np.mean(dynamic_scores))
        max_dynamic_score = float(np.max(dynamic_scores))
        mean_temporal = float(np.mean(temporal_stabilities))
        mean_static_conf = float(np.mean(static_confidences))

        # -----------------------------
        # dynamic
        # -----------------------------
        is_dynamic = (
            mean_dynamic_score > 0.4 and
            mean_temporal > 0.3 and
            num_voxels >= 3
        )

        motion_confidence = min(
            1.0,
            0.7 * mean_dynamic_score +
            0.3 * mean_temporal
        )

        # -----------------------------
        # distance
        # -----------------------------
        center_np = np.array([cx, cy, cz], dtype=np.float32)
        distances = np.linalg.norm(positions - center_np, axis=1)
        estimated_distance = float(np.mean(distances))

        # -----------------------------
        # size (AABB)
        # -----------------------------
        min_vals = np.min(positions, axis=0)
        max_vals = np.max(positions, axis=0)
        size = max_vals - min_vals

        return {
            "num_voxels": num_voxels,
            "density": density,
            "is_dynamic": is_dynamic,
            "motion_confidence": motion_confidence,
            "distance": estimated_distance,
            "size": size,

            "mean_dynamic_score": mean_dynamic_score,
            "max_dynamic_score": max_dynamic_score,
            "mean_temporal_stability": mean_temporal,
            "mean_static_confidence": mean_static_conf,
        }


def main(args=None):
    rclpy.init(args=args)

    node = VoxelStateEstimatorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()