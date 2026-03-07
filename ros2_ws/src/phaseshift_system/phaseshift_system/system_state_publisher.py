from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from slam_toolbox.srv import SaveMap
from phaseshift_interfaces.msg import SystemState

class SystemStatePublisher:

    def __init__(self, node):
        self.node = node

        # -----------------------------
        # QoS Design
        # -----------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._publisher = node.create_publisher(
            SystemState,
            'system_state',
            qos
        )

    # --------------------------------------------------
    # Publish System State
    # --------------------------------------------------

    def publish(self, phase: int,
                has_error: bool = False,
                error_message: str = ""):

        msg = SystemState()

        msg.stamp = self.node.get_clock().now().to_msg()
        msg.phase = phase
        msg.has_error = has_error
        msg.error_message = error_message

        self._publisher.publish(msg)
        self.node.get_logger().info(
            f"[SYSTEM_STATE] phase={phase}, "
            f"error={has_error}, msg='{error_message}'"
        )