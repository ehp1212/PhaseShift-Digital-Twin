import os
import rclpy
from rclpy.node import Node
from enum import Enum

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from phaseshift_control.slam_controller import SlamController
from .system_state_publisher import SystemStatePublisher

class SystemPhase(Enum):
    BOOT = 0

    CHECK_MAP = 1
    CONNECTING = 2
    SLAM_ACTIVE = 3
    MAP_SAVED = 4

    NAV_READY = 5
    NAVIGATING = 6
    
    ERROR = 7

class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator')
        self.get_logger().info("Orchestrator started")

        # Controllers
        self._system_publisher = SystemStatePublisher(self)
        self.slam_controller = SlamController(self)

        self.odom_client = self.create_client(
            ChangeState,
            '/odometry_node/change_state'
        )

        # Internal state
        self.phase = SystemPhase.BOOT
        self._has_map = False
        self._map_path = "/home/jimmy/maps/map.yaml"

        # Publish initial state
        self.publish_state(self.phase)

        # Condition loop (Hybrid: condition watcher only)
        self.create_timer(0.5, self._check_condition_loop)

    # ==================================================
    # Phase Management
    # ==================================================

    def set_phase(self, new_phase: SystemPhase):

        if self.phase == new_phase:
            return

        old_phase = self.phase
        self.on_exit(old_phase)

        self.phase = new_phase
        self.get_logger().info(f"STATE → {self.phase.name}")

        self.on_enter(new_phase)
        self.publish_state(new_phase)

    def on_enter(self, phase: SystemPhase):

        if phase == SystemPhase.BOOT:
            self.get_logger().info("System booting...")

        elif phase == SystemPhase.CHECK_MAP:
            self.get_logger().info("Checking map availability...")

        elif phase == SystemPhase.CONNECTING:
            self.get_logger().info("Waiting for SLAM readiness...")

        elif phase == SystemPhase.SLAM_ACTIVE:
            self.get_logger().info("SLAM is ACTIVE")

        elif phase == SystemPhase.ERROR:
            self.get_logger().error("System entered ERROR state")

    def on_exit(self, old_phase: SystemPhase):
        pass

    # ==================================================
    # Hybrid Condition Loop (Watcher Only)
    # ==================================================

    def _check_condition_loop(self):

        # BOOT → CHECK_MAP (1회 전이)
        if self.phase == SystemPhase.BOOT:
            self.set_phase(SystemPhase.CHECK_MAP)

        # CHECK_MAP → CONNECTING or NAV_READY
        elif self.phase == SystemPhase.CHECK_MAP:
            self._has_map = os.path.exists(self._map_path)

            if self._has_map:
                self.set_phase(SystemPhase.NAV_READY)
            else:
                self.set_phase(SystemPhase.CONNECTING)
                self.activate_odom()

        # CONNECTING → SLAM_ACTIVE
        elif self.phase == SystemPhase.CONNECTING:
            if self.slam_controller.is_ready():
                self.set_phase(SystemPhase.SLAM_ACTIVE)

    # ==================================================
    # System State Publishing
    # ==================================================

    def publish_state(self, phase: SystemPhase):
        self._system_publisher.publish(phase.value)


    # ==================================================
    # Odom State Publishing
    # ==================================================
    def activate_odom(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        self.odom_client.call_async(req)

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self.odom_client.call_async(req)

    def deactivate_odom(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.odom_client.call_async(req)

    # def change_state(self, transition_id):
    #     req = ChangeState.Request()
    #     req.transition.id = transition_id
    #     future = self.odom_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()

# ======================================================
# Main Entry
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()