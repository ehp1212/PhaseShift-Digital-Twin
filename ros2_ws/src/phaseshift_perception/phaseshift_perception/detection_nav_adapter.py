import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from phaseshift_interfaces.msg import DetectedObjectArray


class DetectionNavAdapter(Node):

    def __init__(self):
        super().__init__("detection_nav_adapter")

        # ---------------------------
        # Subscriber
        # ---------------------------
        self._sub = self.create_subscription(
            DetectedObjectArray,
            "/perception/objects_3d",
            self._callback,
            10
        )

        # ---------------------------
        # Publisher
        # ---------------------------
        self._pub = self.create_publisher(
            PointCloud2,
            "/perception/obstacles",
            10
        )

        # ---------------------------
        # Internal state
        # ---------------------------
        self._latest_objects = None

        # publish rate (10Hz)
        self.create_timer(0.1, self._publish_timer)

        self.get_logger().info("DetectionNavAdapter started")

    # --------------------------------------------------
    # Callback (store only)
    # --------------------------------------------------

    def _callback(self, msg: DetectedObjectArray):
        self._latest_objects = msg

    # --------------------------------------------------
    # Timer-based publish
    # --------------------------------------------------

    def _publish_timer(self):

        if self._latest_objects is None:
            return

        msg = self._latest_objects

        points = []

        for obj in msg.objects:

            cx = obj.pose.position.x
            cy = obj.pose.position.y

            # ---------------------------
            # Dynamic radius
            # ---------------------------
            radius = self._compute_dynamic_radius(obj)

            # ---------------------------
            # Circle sampling
            # ---------------------------
            circle_pts = self._sample_circle(cx, cy, radius)

            points.extend(circle_pts)

        if len(points) == 0:
            return

        # ---------------------------
        # PointCloud2 생성
        # ---------------------------
        header = msg.header
        header.frame_id = "map"  # 반드시 map or odom

        pc_msg = point_cloud2.create_cloud_xyz32(header, points)

        self._pub.publish(pc_msg)

    # --------------------------------------------------
    # Dynamic radius
    # --------------------------------------------------

    def _compute_dynamic_radius(self, obj):

        z = obj.pose.position.z
        class_id = obj.class_id

        # base size by class
        if class_id == "chair":
            base = 0.4
        elif class_id == "person":
            base = 0.3
        else:
            base = 0.25

        # depth scaling
        scale = np.clip(z * 0.15, 0.5, 1.5)

        return base * scale

    # --------------------------------------------------
    # Circle sampling (shape → points)
    # --------------------------------------------------

    def _sample_circle(self, cx, cy, radius):

        points = []

        # ---------------------------
        # Outer circle (boundary)
        # ---------------------------
        for theta in np.linspace(0, 2 * np.pi, 16):
            x = cx + radius * np.cos(theta)
            y = cy + radius * np.sin(theta)
            points.append([x, y, 0.0])

        # ---------------------------
        # Inner fill (grid-like)
        # ---------------------------
        for r in np.linspace(0, radius, 4):
            for theta in np.linspace(0, 2 * np.pi, 8):
                x = cx + r * np.cos(theta)
                y = cy + r * np.sin(theta)
                points.append([x, y, 0.0])

        return points


def main(args=None):
    rclpy.init(args=args)

    node = DetectionNavAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()