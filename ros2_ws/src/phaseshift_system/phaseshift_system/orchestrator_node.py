import os
import time
import rclpy
from rclpy.node import Node
from enum import Enum

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from slam_toolbox.srv import SaveMap

from phaseshift_control.slam_controller import SlamController
from .system_state_publisher import SystemStatePublisher

class SystemPhase(Enum):
    BOOT = 0
    SYSTEM_INITIALIZING = 1

    SLAM_PREPARING = 2
    SLAM_ACTIVE = 3
    MAP_SAVING = 4
    MAP_SAVED = 5

    NAV_PREPARING = 6
    NAV_READY = 7
    NAV_EXECUTING = 8

    ERROR = 9

class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator')
        self.map_directory = "/home/jimmy/maps"
        os.makedirs(self.map_directory, exist_ok=True)
        self.get_logger().info(f"Map directory: {self.map_directory}")
        self.get_logger().info("Orchestrator started")

        # Controllers
        self._system_publisher = SystemStatePublisher(self)
        self.slam_controller = SlamController(self)

        # Odom service
        self.odom_client = self.create_client(
            ChangeState,
            '/odometry_node/change_state'
        )

        # Services
        self.save_map_srv = self.create_service(
            SaveMap,
            '/system/save_map',
            self.handle_save_map
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
            self.get_logger().info("[Boot]System booting...")

        elif phase == SystemPhase.SYSTEM_INITIALIZING:
            self.get_logger().info("[Init]Checking map availability...")

        elif phase == SystemPhase.SLAM_PREPARING:
            self.get_logger().info("[SLAM]Waiting for SLAM readiness...")

        elif phase == SystemPhase.SLAM_ACTIVE:
            self.get_logger().info("[SLAM]SLAM is ACTIVE")

        elif phase == SystemPhase.MAP_SAVING:
            self.get_logger().info("[SLAM]Saving map...")

        elif phase == SystemPhase.MAP_SAVED:
            self.get_logger().info("[SLAM]Map saved")

        elif phase == SystemPhase.ERROR:
            self.get_logger().error("System entered ERROR state")

    def on_exit(self, old_phase: SystemPhase):
        pass

    # ==================================================
    # Hybrid Condition Loop (Watcher Only)
    # ==================================================

    def _check_condition_loop(self):

        # BOOT → INIT
        if self.phase == SystemPhase.BOOT:
            self.set_phase(SystemPhase.SYSTEM_INITIALIZING)

        # INIT → CONNECTING or NAV_READY
        elif self.phase == SystemPhase.SYSTEM_INITIALIZING:
            self._has_map = os.path.exists(self._map_path)

            if self._has_map:
                self.set_phase(SystemPhase.NAV_PREPARING)
            else:
                self.set_phase(SystemPhase.SLAM_PREPARING)
                self.activate_odom()

        # CONNECTING → SLAM_ACTIVE
        elif self.phase == SystemPhase.SLAM_PREPARING:
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

    # ==================================================
    # System Service
    # ==================================================
    def handle_save_map(self, request, response):

        if self.phase != SystemPhase.SLAM_ACTIVE:
            response.accepted = False
            response.message = "SLAM not active"
            return response

        map_name = request.map_name.strip()
        if map_name == "":
            response.accepted = False
            response.message = "Invalid map name"
            return response

        if "/" in map_name:
            response.accepted = False
            response.message = "Map name must not contain '/'"
            return response

        self.set_phase(SystemPhase.MAP_SAVING)
        timestamp = int(time.time())
        map_file = f"{map_name}_{timestamp}"

        map_path = os.path.join(self.map_directory, map_file)
        self.slam_controller.save_map_async(map_path)

        response.accepted = True
        response.message = f"Saving map {map_name}"
        return response
    
    def on_map_saved(self):
        self.set_phase(SystemPhase.MAP_SAVED)

    def on_map_save_failed(self):
        self.set_phase(SystemPhase.SLAM_ACTIVE)

# ======================================================
# Main Entry
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()