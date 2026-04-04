import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from phaseshift_interfaces.msg import TrackedObjectArray
class DetectionNavAdapter(Node):

    MINIMUM_SPEED_THREDHOLD = 0.05

    def __init__(self):
        super().__init__("detection_nav_adapter")

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
    # Callback
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

            speed = np.sqrt(obj.velocity.x**2 + obj.velocity.y**2)
            radius += np.clip(speed * 0.3, 0.0, 0.5)

            # radius scaling
            risk = self._compute_risk(obj)
            radius *= (1.0 + risk)
            radius = max(radius, 0.2)

            # ---------------------------
            # Circle sampling
            # ---------------------------
            shape_pts = self._sample_motion_aware(obj, radius)
            points.extend(shape_pts)

        # pointCloud2
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
    
    def _sample_motion_aware(self, obj, radius):
        """
        calculate predicted paths
        apply the points to costmap
        """
        cx = obj.pose.position.x
        cy = obj.pose.position.y

        vx = obj.velocity.x
        vy = obj.velocity.y

        speed = np.sqrt(vx * vx + vy * vy)

        # static or very slow object
        if speed < self.MINIMUM_SPEED_THREDHOLD:
            return self._sample_circle(cx, cy, radius)
        
        points = []
        
        # direction
        dx = vx / speed
        dy = vy / speed

        # lookahead distance
        lookahead = np.clip(speed * 1.0, 0.3, 1.5)

        # future position
        num_steps = 4

        for t in np.linspace(0.0, lookahead, num_steps):

            px = cx + dx * t
            py = cy + dy * t

            pts = self._sample_circle(px, py, radius)
            points.extend(pts)
        
        return points
    
    def _compute_risk(self, obj):

        px = obj.pose.position.x
        py = obj.pose.position.y

        vx = obj.velocity.x
        vy = obj.velocity.y

        # distance
        dist = np.sqrt(px * px + py * py)

        # speed
        speed = np.sqrt(vx * vx + vy * vy)

        # confidence
        confidence = obj.motion_confidence

        # risk
        risk = (speed * confidence) / (dist + 0.1)

        # clamp
        risk = np.clip(risk, 0.0, 2.0)

        return risk        

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