import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from phaseshift_interfaces.msg import TrackedObjectArray
class DetectionNavAdapter(Node):

    def __init__(self):
        super().__init__("detection_nav_adapter")

        # ---------------------------
        # Subscriber
        # ---------------------------

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self._sub = self.create_subscription(
            TrackedObjectArray,
            '/perception/tracked_objects',
            self._callback,
            qos_sensor
        )

        # ---------------------------
        # Publisher
        # ---------------------------
        self._pub = self.create_publisher(
            PointCloud2,
            "/perception/obstacles",
            qos_sensor
        )

        # ---------------------------
        # Internal state
        # ---------------------------
        self._latest_objects = None

        # publish rate (10Hz)
        self.create_timer(0.1, self._publish_timer)

        self.get_logger().info("Detection Nav Adapter started...")

    # --------------------------------------------------
    # Callback (store only)
    # --------------------------------------------------

    def _callback(self, msg: TrackedObjectArray):
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
            if obj.is_dynamic:
                radius *= 1.5
            else:
                radius *= 1.0
            
            radius = max(radius, 0.2)

            # ---------------------------
            # Circle sampling
            # ---------------------------
            circle_pts = self._sample_circle(cx, cy, radius)

            points.extend(circle_pts)

        # ---------------------------
        # PointCloud2
        # ---------------------------
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "map"

        pc_msg = point_cloud2.create_cloud_xyz32(header, points)

        self._pub.publish(pc_msg)

    # --------------------------------------------------
    # Dynamic radius
    # --------------------------------------------------

    def _compute_dynamic_radius(self, obj):

        x = obj.pose.position.x
        y = obj.pose.position.y
        dist = np.sqrt(x*x + y*y)

        class_id = obj.class_id

        # base size by class
        if class_id == "chair":
            base = 0.5
        elif class_id == "person":
            base = 0.3
        else:
            base = 0.25

        # depth scaling
        scale = np.clip(dist * 0.1, 0.5, 1.5)

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