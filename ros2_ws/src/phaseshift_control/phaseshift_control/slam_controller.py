import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class SlamController:

    def __init__(self, node: Node):
        self.node = node

        self._map_received = False
        self.is_map_saved = False
        self._map_sub = node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

        self._save_map_cli = self.node.create_client(
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
    
    def map_saved(self) -> bool:
        return self.is_map_saved

    # ----------------------------------------------
    # Map Save
    # ----------------------------------------------

    def save_map_async(self, path: str):

        if not self._save_map_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("SaveMap service not available.")
            self.node.on_map_save_failed()
            return

        request = SaveMap.Request()
        request.name = String()
        request.name.data = path
        self.is_map_saved = False

        self._save_map_future = self._save_map_cli.call_async(request)
        # self._save_map_future.add_done_callback(self._on_save_map_done)

        # TODO: As this is single thread, use another service call cannot handle callback
        self._check_future_timer = self.node.create_timer(
            0.1,
            self.check_save_map_status
        )
    
    def check_save_map_status(self):
        if self._save_map_future is None:
            return
        
        if not self._save_map_future.done():
            return
        
        try:
            result = self._save_map_future.result()
            self.is_map_saved = True
            # self.node.get_logger().info("Map saved successfully")
            self.node.on_map_save_succeeded()

        except Exception as e:
            self.node.get_logger().error(f"SaveMap failed: {e}")
            self.node.on_map_save_failed()
            return
        finally:
            self._check_future_timer.destroy()
            self._check_future_timer = None
            self._save_map_future = None
    
    # ----------------------------------------------

    def _map_callback(self, msg):

        if self._map_received:
            return

        self._map_received = True
        self.node.get_logger().info("First map received")