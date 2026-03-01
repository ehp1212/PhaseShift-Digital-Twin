# slam_controller.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformListener

class SlamController:
    """
    1. SLAM 실행 중
    2. Orchestrator = SLAM_CONNECTING 상태
    3. 주기적으로 slam_controller.is_ready() 호출
    4. True 되면
    5. system_state = SLAM_ACTIVE publish
    """
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

    # --------------------------------------------------
    # Runtime check
    # --------------------------------------------------

    def is_slam_running(self) -> bool:
        nodes = self.node.get_node_names()
        return 'slam_toolbox' in nodes

    def is_map_active(self) -> bool:
        return self._map_received

    def wait_until_map_active(self, timeout_sec=5.0) -> bool:
        start = self.node.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self._map_received:
                return True

            if (self.node.get_clock().now() - start).nanoseconds / 1e9 > timeout_sec:
                return False

        return False
    
    def is_map_tf_active(self):
        try:
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time()
            )
            return True
        except:
            return False
        
    def is_ready(self) -> bool:
        if not self.map_received:
            return False

        if not self.is_map_tf_active(self):
            return False
        
        return True
    
    # --------------------------------------------------
    # Map save
    # --------------------------------------------------

    def save_map(self, path: str) -> bool:

        if not self._save_map_cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("SaveMap service not available.")
            return False

        request = SaveMap.Request()
        request.name = path

        future = self._save_map_cli.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is None:
            return False

        return True

    # --------------------------------------------------

    def _map_callback(self, msg):
        if self._map_received:
            return

        if not self._is_tf_ready():
            return

        self._map_received = True
        self.node.get_logger().info("SLAM READY")
    
        self.node.destroy_subscription(self._map_sub)
        self._map_sub = None

"""
def enter_slam_phase(self):

    if not self.slam_controller.is_slam_running():
        self.system_state = ERROR
        self.publish_state()
        return

    if not self.slam_controller.wait_until_map_active():
        self.system_state = ERROR
        self.publish_state()
        return

    self.system_state = SLAM_ACTIVE
    self.publish_state()
"""