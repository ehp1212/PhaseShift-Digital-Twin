import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException

class SlamTestNode(Node):

    def __init__(self):
        super().__init__('slam_runtime_test_node')

        self.map_received = False

        # Map subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.check_slam_status)

        self.get_logger().info("SLAM Runtime Test Started")

    def map_callback(self, msg):
        self.map_received = True

    def check_slam_status(self):

        tf_active = False

        try:
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time()
            )
            tf_active = True
        except LookupException:
            tf_active = False

        self.get_logger().info(
            f"Map received: {self.map_received}, "
            f"map->odom TF active: {tf_active}"
        )

        if self.map_received and tf_active:
            self.get_logger().info("SLAM IS ACTIVE ✅")
        else:
            self.get_logger().warn("SLAM NOT ACTIVE ❌")


def main(args=None):
    rclpy.init(args=args)
    node = SlamTestNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()