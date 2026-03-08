import rclpy
from rclpy.node import Node
import time

from phaseshift_control.slam_controller import SlamController


class SlamTestNode(Node):

    def __init__(self):
        super().__init__("slam_test_node")

        self.slam = SlamController(self)

        self.get_logger().info("Starting SLAM...")
        self.slam.start_slam()

        self.timer = self.create_timer(1.0, self.check_status)

    def check_status(self):

        self.get_logger().info(
            f"running={self.slam.is_running()} "
            f"map={self.slam.is_map_active()} "
            f"map_tf={self.slam.is_map_tf_active()} "
            f"odom_tf={self.slam.is_odom_tf_active()}"
        )

        if self.slam.is_ready():
            self.get_logger().info("SLAM READY")

            time.sleep(3)

            self.get_logger().info("Stopping SLAM")
            self.slam.stop_slam()

            rclpy.shutdown()


def main():
    rclpy.init()

    node = SlamTestNode()

    rclpy.spin(node)


if __name__ == "__main__":
    main()