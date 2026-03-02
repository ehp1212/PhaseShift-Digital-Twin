import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class SlamController:

    def __init__(self, node: Node):
        self.node = node

        self._map_received = False

        self._map_sub = node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

        self._save_map_cli = node.create_client(
            SaveMap,
            '/slam_toolbox/save_map'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    # ----------------------------------------------
    # Readiness
    # ----------------------------------------------

    def is_map_active(self) -> bool:
        return self._map_received

    def is_map_tf_active(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            return True
        except:
            return False
    
    def is_odom_tf_active(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            return True
        except:
            return False

    # Slam Ready Condition
    # receive laser scan 
    # odom -> bast_footprint tf
    # after pose graph initialized
    def is_ready(self) -> bool:
        return self.is_map_active() and self.is_map_tf_active() and self.is_odom_tf_active()

    # ----------------------------------------------
    # Map Save
    # ----------------------------------------------

    def save_map(self, path: str) -> bool:

        if not self._save_map_cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("SaveMap service not available.")
            return False

        request = SaveMap.Request()
        request.name = path

        future = self._save_map_cli.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result() is not None

    # ----------------------------------------------

    def _map_callback(self, msg):

        if self._map_received:
            return

        self._map_received = True
        self.node.get_logger().info("First map received")