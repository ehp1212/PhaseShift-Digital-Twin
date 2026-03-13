import os
import time
import rclpy
from rclpy.node import Node
from enum import Enum

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, TransitionEvent
from geometry_msgs.msg import PoseWithCovarianceStamped

from phaseshift_interfaces.srv import SaveMap
from phaseshift_interfaces.srv import SetGoal
from phaseshift_interfaces.srv import NavigateToGoal

from phaseshift_control.slam_controller import SlamController
from phaseshift_control.nav2_controller import Nav2Controller

from .system_state_publisher import SystemStatePublisher
from .map_manager import MapManager

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
        self.get_logger().info("Orchestrator started")

        # Controllers
        self._system_publisher = SystemStatePublisher(self)
        self.slam_controller = SlamController(self)
        self.nav2_controller = Nav2Controller(self)

        # Map Manager
        self.map_manager = MapManager()
        self._map_yaml = None

        # Odom service
        self.odom_client = self.create_client(
            ChangeState,
            '/odometry_node/change_state'
        )

        self.odom_state_client = self.create_client(
            GetState,
            '/odometry_node/get_state'
        )

        # Check odom transition 
        self.create_subscription(
            TransitionEvent,
            '/odometry_node/transition_event',
            self._on_odom_activated,
            10
        )

        # Services
        self.save_map_srv = self.create_service(
            SaveMap,
            '/system/save_map',
            self.handle_save_map
        )

        self.set_goal_srv = self.create_service(
            SetGoal,
            '/system/set_goal',
            self.handle_set_goal
        )

        self.navigate_srv = self.create_service(
            NavigateToGoal,
            '/system/navigate',
            self.handle_navigate
        )

        # Internal state
        self.phase = SystemPhase.BOOT

        self.localization_active = False
        self.initial_pose_sent = False
        
        self.is_odom_active = False
        self._odom_future = None
        self._odom_state = "IDLE"

        # initial pose publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )

        # Publish initial state
        self.publish_state(self.phase)

        # Condition loop (Hybrid: condition watcher only)
        self.create_timer(0.5, self._check_condition_loop)
    
        
    # ==================================================
    # Phase Management (Behaviour)
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
            self._has_map = self.map_manager.has_map()
            self.activate_odom()

        elif phase == SystemPhase.SLAM_PREPARING:
            self.get_logger().info("[SLAM]Waiting for SLAM readiness...")
            if not self.slam_controller.is_running():
                self.slam_controller.start_slam()
            
            self._map_yaml = None

        elif phase == SystemPhase.SLAM_ACTIVE:
            self.get_logger().info("[SLAM]SLAM is ACTIVE")

        elif phase == SystemPhase.MAP_SAVING:
            self.get_logger().info("[SLAM]Saving map...")

        elif phase == SystemPhase.MAP_SAVED:
            self.get_logger().info("[SLAM]Map saved")

        elif phase == SystemPhase.NAV_PREPARING:
            pass

        elif phase == SystemPhase.ERROR:
            self.get_logger().error("System entered ERROR state")

    def on_exit(self, old_phase: SystemPhase):
        pass

    # ==================================================
    # Condition Loop (Condition / Set Phase)
    # ==================================================

    def _check_condition_loop(self):
        # BOOT → INIT
        if self.phase == SystemPhase.BOOT:
            self.set_phase(SystemPhase.SYSTEM_INITIALIZING)
            return

        # INIT → CONNECTING or NAV_READY
        if self.phase == SystemPhase.SYSTEM_INITIALIZING:

            self._update_odom_lifecycle()

            if not self.is_odom_active:
                return

            if self._has_map:
                self.set_phase(SystemPhase.NAV_PREPARING)
            else:
                self.set_phase(SystemPhase.SLAM_PREPARING)
                return

        # ==================================================
        # SLAM
        # ==================================================
        # CONNECTING → SLAM_ACTIVE
        if self.phase == SystemPhase.SLAM_PREPARING:
            if self.slam_controller.is_ready():
                self.set_phase(SystemPhase.SLAM_ACTIVE)
            return
        
        # MAP_SAVED -> SYSTEM_INITIALIZING
        if self.phase == SystemPhase.MAP_SAVED:
            self.slam_controller.stop_slam()
            self.set_phase(SystemPhase.SYSTEM_INITIALIZING)
            return
        
        # ==================================================
        # NAV2
        # ==================================================

        if self.phase == SystemPhase.NAV_PREPARING:
            if self.nav2_controller.is_busy():
                return
            
            # 1 Configure
            if not self.nav2_controller.is_configured():
                self.nav2_controller.configure_nav2()
                return
            
            # 2 activate map_server + amcl
            if not self.localization_active:
                self.nav2_controller.activate_localization()
                self.localization_active = True
                return

            # 3 load map
            if not self.nav2_controller.is_map_loaded():
                map_yaml = self.map_manager.get_latest_map()
                self.nav2_controller.load_map_nav2(map_yaml)
                return

            # 4 send initial pose
            if not self.initial_pose_sent:
                self.publish_initial_pose()
                self.initial_pose_sent = True
                return
            
            # 5 activate rest of nav2
            if not self.nav2_controller.is_activated():
                self.nav2_controller.activate_navigation()
                return
            
            self.get_logger().info(f"==========")            
            self.get_logger().info(f"==========")            
            self.get_logger().info(f"==========")            
            self.get_logger().info(f"READYREADY")
            self.get_logger().info(f"==========")            
            self.get_logger().info(f"==========")            
            self.get_logger().info(f"==========")            

            # nav2 configure and activate
            if self.nav2_controller.is_ready():
                self.set_phase(SystemPhase.NAV_READY)

            return

        if self.phase == SystemPhase.NAV_EXECUTING:
            result = self.nav2_controller.update_navigation()

            if result == "SUCCEEDED":
                self.set_phase(SystemPhase.NAV_READY)
            elif result == "FAILED":
                self.set_phase(SystemPhase.ERROR)


    # ==================================================
    # System State Publishing
    # ==================================================

    def publish_state(self, phase: SystemPhase):
        self._system_publisher.publish(phase.value)

    # ==================================================
    # Odom State Publishing
    # ==================================================
    def activate_odom(self):
            
        if not self.odom_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Odom lifecycle service not available")
                return

        self.get_logger().info("Configuring odom node...")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE

        self._odom_future = self.odom_client.call_async(req)
        self._odom_state = "CONFIGURING"

        # req = ChangeState.Request()
        # req.transition.id = Transition.TRANSITION_ACTIVATE
        # self.odom_client.call_async(req)

    def _update_odom_lifecycle(self):
            
        if self._odom_state == "CONFIGURING":

            if not self._odom_future.done():
                return

            self.get_logger().info("Odom configured, activating...")

            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE

            self._odom_future = self.odom_client.call_async(req)
            self._odom_state = "ACTIVATING"
            return


        if self._odom_state == "ACTIVATING":

            if not self._odom_future.done():
                return

            self.get_logger().info("Odom ACTIVE")

            self.is_odom_active = True
            self._odom_state = "ACTIVE"     

    def deactivate_odom(self):
       
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.odom_client.call_async(req)

    def _on_odom_activated(self, msg: TransitionEvent):
       
        start = msg.start_state.label
        goal = msg.goal_state.label

        self.get_logger().info(
            f"Lifecycle transition: {start} --------> {goal}"
        )
        if goal == "active":
            self.get_logger().info(
                "[Odom] Odometry is active"
            )
            self.is_odom_active = True
            return
        
        if goal == "inactive":
            self.get_logger().info(
                "[Odom] Odometry is inactive"
            )
            self.is_odom_active = False

    # ==================================================
    # System Service
    # ==================================================
    def handle_save_map(self, request, response):

        if self.phase != SystemPhase.SLAM_ACTIVE:
            response.success = False
            response.message = "SLAM not active"
            return response

        map_name = request.map_name.strip()
        if map_name == "":
            response.success = False
            response.message = "Invalid map name"
            return response

        if "/" in map_name:
            response.success = False
            response.message = "Map name must not contain '/'"
            return response

        self.set_phase(SystemPhase.MAP_SAVING)
        timestamp = int(time.time())
        map_file = f"{map_name}_{timestamp}"

        map_path = os.path.join(self.map_manager.map_directory, map_file)
        self.slam_controller.save_map_async(map_path)

        response.success = True
        response.message = f"Saving map {map_name}"
        return response

    def on_map_save_succeeded(self):
        self.set_phase(SystemPhase.MAP_SAVED)

    def on_map_save_failed(self):
        self.set_phase(SystemPhase.SLAM_ACTIVE)


    def handle_set_goal(self, request, response):
        if self.phase != SystemPhase.NAV_READY:
            response.success = False
            response.message = "Only available in NAV2_READY phase"
            return response
        
        self.nav2_controller.set_goal(
            x=request.x,
            y=request.y,
            yaw=request.yaw,
            frame_id=request.frame_id
        )

        self.set_phase(SystemPhase.NAV_EXECUTING)

        response.success = True
        response.message = "Goal stored"
        return response

    def handle_navigate(self, request, response):
        pass


    # ======================================================
    # Utility
    # ======================================================

    def publish_initial_pose(self):

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.initial_pose_pub.publish(msg)

        self.get_logger().info("[Nav2] Initial pose published")

# ======================================================
# Main Entry
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()