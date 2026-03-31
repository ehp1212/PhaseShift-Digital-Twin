import os
import rclpy

from rclpy.node import Node

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

class OdometryController:

    def __init__(self, node: Node):
        self.node = node
        
        # Odom service
        self.odom_client = self.node.create_client(
            ChangeState,
            '/odometry_node/change_state'
        )

        self.odom_state_client = self.node.create_client(
            GetState,
            '/odometry_node/get_state'
        )

        self._is_odom_active = False
        self._odom_future = None
    
    # ==================================================
    # PUBLIC API
    # ==================================================
    def is_active(self) -> bool:
        return self._is_odom_active

    def activate(self):

        if not self.odom_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Odom lifecycle service not available")
                return

        self.node.get_logger().info("Configuring odom node...")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE

        self._odom_future = self.odom_client.call_async(req)
        self._odom_future.add_done_callback(self._on_odom_configured)

    def deactivate(self):
    
        if not self.odom_client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().warn("Odom lifecycle service not available")
                return

        self.node.get_logger().info("Deactivating odom node...")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE

        self._odom_future = self.odom_client.call_async(req)
        self._odom_future.add_done_callback(self._on_odom_deactivate)
        self.odom_client.call_async(req)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def _on_odom_configured(self, future):
        
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Odom configure failed: {e}")
            return

        if not result.success:
            self.node.et_logger().error("Odom configure failed (response false)")
            return

        self.node.get_logger().info("[Odom] CONFIGURED, activating...")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE

        future = self.odom_client.call_async(req)
        future.add_done_callback(self._on_odom_activated)

    # -----------------------------

    def _on_odom_activated(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Odom activate failed: {e}")
            return

        if not result.success:
            self.node.get_logger().error("Odom activate failed (response false)")
            return

        self.node.get_logger().info("[Odom] ACTIVE")
        self._is_odom_active = True

    # -----------------------------

    def _on_odom_deactivate(self, future):
        
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Odom deactivate failed: {e}")
            return

        if not result.success:
            self.node.get_logger().error("Odom deactivate failed (response false)")
            return

        self.node.get_logger().info("[Odom] DEACTIVATED, cleaning up...")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CLEANUP
        self._is_odom_active = False

        future = self.odom_client.call_async(req)
        future.add_done_callback(self._on_odom_cleanup)

    # -----------------------------

    def _on_odom_cleanup(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Odom cleanup failed: {e}")
            return

        if not result.success:
            self.node.get_logger().error("Odom cleanup failed (response false)")
            return

        self.node.get_logger().info("[Odom] CLEANUP")
