import os
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from enum import Enum
from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, TransitionEvent
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from phaseshift_interfaces.msg import NavState, NavigationFeedback
from phaseshift_interfaces.srv import SaveMap, SaveVoxelMap, SetGoal, NavigateToGoal, InternalRequestNavigate, InternalCancelNavigate, InternalRequestRecovery

from phaseshift_control.slam_controller import SlamController
from phaseshift_control.nav2_controller import Nav2Controller
from phaseshift_control.odometry_controller import OdometryController
from phaseshift_control.perception_controller import PerceptionController

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
        self._log_phase("ORCHESTRA NODE", "Started...")

        # ------------------------------
        # PARAMETERS
        # ------------------------------
        self.declare_parameter("project_name", "demo")

        self.project_name = self.get_parameter("project_name").value
        self.use_example_map = self.project_name == "demo"
        self.example_map_name = "demo"

        self._log_phase("ORCHESTRA NODE - PARAM", f"use_example_map={self.use_example_map} / example_map_name={self.example_map_name} / project_name={self.project_name}")

        # ------------------------------
        # CONTROLLER 
        # ------------------------------
        self._system_publisher = SystemStatePublisher(self)
        self.slam_controller = SlamController(self)
        self.nav2_controller = Nav2Controller(self)
        self.odom_controller = OdometryController(self)
        self.perception_controller = PerceptionController(self)

        # ------------------------------
        # CONTROLLER SERVICES
        # ------------------------------
        self.save_map_srv = self.create_service(
            SaveMap,
            '/system/save_map',
            self.handle_save_map
        )

        self._voxel_save_client = self.create_client(
            SaveVoxelMap,
            "/system/save_voxel_map"
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

        # ------------------------------
        # PERCEPTION GEOMETRY LAYER
        # ------------------------------
        self.voxel_costmap_client = self.create_client(
            ChangeState,
            '/voxel_costmap_node/change_state'
        )

        self._voxel_costmap_set_param_client = self.create_client(
            SetParameters,
            '/voxel_costmap_node/set_parameters'
        )

        self.voxel_change_detection_client = self.create_client(
            ChangeState,
            'voxel_change_detection_node/change_state'
        )

        self._voxel_change_detection_set_param_client = self.create_client(
            SetParameters,
            '/voxel_change_detection_node/set_parameters'
        )

        # ------------------------------
        # BEHAVIOUR NODE
        # ------------------------------
        self._behanvior_change_state_client = self.create_client(
            ChangeState,
            "/robot_behaviour_node/change_state"
        )

        self._nav_state_pub = self.create_publisher(
            NavState,
            '/system/internal/nav_state',
            10
        )

        self._nav2_feedback_sub = self.create_subscription(
            NavigationFeedback,
            "/system/navigation_feedback",
            self._feedback_callback,
            10
        )

        self._behaviour_request_srv = self.create_service(
            InternalRequestNavigate,
            '/system/internal/request_navigate',
            self._on_behaviour_navigate_request
        )

        self._behaviour_recover_srv = self.create_service(
            InternalRequestRecovery,
            '/system/internal/request_recovery',
            self._on_behaviour_recovery_request
        )

        self._behaviour_cancel_srv = self.create_service(
            InternalCancelNavigate,
            '/system/internal/cancel_navigate',
            self._on_behaviour_cancel_request
        )
    
        self._latest_nav_feedback = None
        self._distance_history = []  

        # ------------------------------
        # MAP MANAGER
        # ------------------------------
        package_dir = get_package_share_directory("phaseshift_bringup")
        package_map_dir = os.path.join(package_dir, "maps")
        
        self.map_manager = MapManager(self, package_map_dir)
        self._map_yaml = None

        self._slam_saved = False
        self._voxel_saved = False

        self.initial_pose_sent = False 
        self._behaviour_activated = False

        # initial pose publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )

        # Internal state
        self.phase = SystemPhase.BOOT

        # Publish initial state
        self.publish_state(self.phase)

        # Condition loop (Hybrid: condition watcher only)
        self.create_timer(0.5, self._update)
        
    # ==================================================
    # Phase Management (Behaviour)
    # ==================================================
    
    def set_phase(self, new_phase: SystemPhase):

        if self.phase == new_phase:
            return

        old_phase = self.phase
        self.on_exit(old_phase)

        self.phase = new_phase
        self._log_phase("ORCHESTRA NODE", f"{self.phase.name} entered...")

        self.on_enter(new_phase)
        self.publish_state(new_phase)

    def on_enter(self, phase: SystemPhase):

        if phase == SystemPhase.BOOT:
            self.get_logger().info("[Boot]System booting...")
            self._behaviour_activated = False

        elif phase == SystemPhase.SYSTEM_INITIALIZING:
            self.get_logger().info("[Init]Checking map availability...")

            if not self.odom_controller.is_active():
                self.odom_controller.activate()

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
            self.get_logger().info("[NAV]Waiting for NAV readiness...")

            # Activate behaviour node
            self._activate_behaviour_node()

            # TODO: Need to set this as must-meet condition for nav2 phase
            # Activate voxel costmap node
            voxel_map_path = self.map_manager.resolve_3d_map(self.project_name)
            self._activate_voxel_costmap_node(voxel_map_path)
            # self._activate_change_detection_node(voxel_map_path)

        elif phase == SystemPhase.NAV_READY:
            voxel_map_path = self.map_manager.resolve_3d_map(self.project_name)
            self._activate_change_detection_node(voxel_map_path)
            pass

        elif phase == SystemPhase.ERROR:
            self.get_logger().error("System entered ERROR state")

    def on_exit(self, old_phase: SystemPhase):
        pass

    # ==================================================
    # UPDATE
    # ==================================================

    def _update(self):
        self._check_nav_feedback()
        self._check_condition_loop()

    def _check_nav_feedback(self):
        if self._latest_nav_feedback is None:
            return
        
        feedback = self._latest_nav_feedback

        # -------------------------
        # record distance history 
        # -------------------------
        self._record_distance(feedback.distance_remaining)

        # -------------------------
        # NavState 
        # -------------------------
        nav_state = NavState()
        nav_state.stamp = self.get_clock().now().to_msg()

        current_goal = self.nav2_controller.get_active_goal()
        nav_state.current_goal = current_goal
        nav_state.has_goal = current_goal is not None

        # -------------------------
        # check struck
        # -------------------------
        nav_state.is_stuck = self._is_stuck(feedback)

        nav_state.distance_to_goal = feedback.distance_remaining
        nav_state.remaining_path_length = feedback.distance_remaining

        nav_state.navigation_time = feedback.navigation_time
        nav_state.estimated_time_remaining = feedback.estimated_time_remaining

        nav_state.current_pose = feedback.current_pose

        # -------------------------
        # check current status
        # -------------------------
        if not nav_state.has_goal:
            nav_state.status = NavState.IDLE

        elif self.nav2_controller.is_succeeded():
            nav_state.status = NavState.SUCCEEDED

        elif self.nav2_controller.is_failed():
            nav_state.status = NavState.FAILED

        elif nav_state.is_stuck:
            nav_state.status = NavState.BLOCKED

        elif feedback.recovery_count > 0:
            nav_state.status = NavState.RECOVERING

        else:
            nav_state.status = NavState.MOVING

        # -------------------------
        # debug
        # -------------------------
        nav_state.detail = f"recovery={feedback.recovery_count}"

        # -------------------------
        # publish
        # -------------------------
        self._nav_state_pub.publish(nav_state)
        self._latest_nav_feedback = None

    def _check_condition_loop(self):
        # BOOT → INIT
        if self.phase == SystemPhase.BOOT:
            self.set_phase(SystemPhase.SYSTEM_INITIALIZING)
            return

        # INIT → CONNECTING or NAV_READY
        if self.phase == SystemPhase.SYSTEM_INITIALIZING:

            if not self.odom_controller.is_active():
                return
            
            has_map = self.map_manager.resolve_2d_map(self.project_name, False) is not None
            if self.use_example_map or has_map:
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

            # Nav2 localization
            if not self.nav2_controller.is_localization_ready():
                return

            # Load map
            if not self.nav2_controller.is_map_loaded():

                if self._map_yaml is None:

                    if self.use_example_map:
                        self._map_yaml = self.map_manager.resolve_2d_map(self.example_map_name, True)
                    else:
                        self._map_yaml = self.map_manager.resolve_2d_map(self.project_name)

                    if self._map_yaml is None:
                        self.get_logger().error("No runtime map found")
                        self.set_phase(SystemPhase.ERROR)
                        return

                self.nav2_controller.load_map(self._map_yaml)
                return
         
            # Initial pose
            if not self.initial_pose_sent:
                self.publish_initial_pose()
                self.initial_pose_sent = True
                return

            # Nav2 navigation
            if not self.nav2_controller.is_navigation_ready():
                return
            
            # Perception Layer
            self.perception_controller.activate()
            if not self.perception_controller.is_active():
                return

            # Behaviour Node
            if not self._behaviour_activated:
                return

            # nav2 configure and activate
            if self.nav2_controller.is_ready() and self.perception_controller.is_active():
                self.get_logger().info(f"NAV Phase Ready to use")            
                self.set_phase(SystemPhase.NAV_READY)
                return

        if self.phase == SystemPhase.NAV_EXECUTING:

            if not self.nav2_controller.tick_navigation():
                if self.nav2_controller.is_error():
                    self.set_phase(SystemPhase.ERROR)
                return
            
            self.set_phase(SystemPhase.NAV_READY)

    # ==================================================
    # System State Publishing
    # ==================================================
    def publish_state(self, phase: SystemPhase):
        self._system_publisher.publish(phase.value)

    # ==================================================
    # System Service
    # ==================================================
    def handle_save_map(self, request, response):

        if self.phase != SystemPhase.SLAM_ACTIVE:
            response.success = False
            response.message = "SLAM not active"
            return response

        map_name = f"{self.project_name}"
        if map_name == "":
            response.success = False
            response.message = "Invalid map name"
            return response

        self.set_phase(SystemPhase.MAP_SAVING)

        # 2D map
        base_path = self.map_manager.generate_2d_map_path(map_name)
        self.slam_controller.save_map_async(
            base_path,
            on_done=self._on_slam_saved
        )

        # 3D map
        voxel_path = self.map_manager.generate_3d_map_path(map_name)
        self._save_voxel_map_async(voxel_path)

        response.success = True
        response.message = f"Saving map {map_name}"
        return response

    def handle_set_goal(self, request, response):
        if self.phase != SystemPhase.NAV_READY:
            response.success = False
            response.message = "Only available in NAV2_READY phase"
            return response
        
        # NAV2 controller
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
    # BEHAVIOAUR NODE
    # ======================================================

    def _activate_behaviour_node(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE

        self.get_logger().info("Configure Robot Behaviour")
        
        future = self._behanvior_change_state_client.call_async(req)
        future.add_done_callback(self._on_behaviour_configured)

    def _on_behaviour_configured(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Robot Behaviour configure failed: {e}")
            return

        if not result.success:
            self.get_logger().error(f"Robot Behaviour configure failed: {e}")
            return

        self.get_logger().info("[Robot Behaviour] CONFIGURED, activating...")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE

        future = self._behanvior_change_state_client.call_async(req)
        future.add_done_callback(self._on_behaviour_activated)    

    def _on_behaviour_activated(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Robot Behaviour activation failed: {e}")
            return

        if not result.success:
            self.get_logger().error(f"Robot Behaviour activation failed")
            return

        self._behaviour_activated = True
        self.get_logger().info("[Robot Behaviour] ACTIVATED")

    def _feedback_callback(self, msg: NavigationFeedback):
        self._latest_nav_feedback = msg


    def _on_behaviour_navigate_request(self, request, response):

        self.nav2_controller.set_goal(
            x=request.goal.pose.position.x,
            y=request.goal.pose.position.y,
            yaw=0
        )

        self.set_phase(SystemPhase.NAV_EXECUTING)

        response.accepted = True
        response.message = "Goal stored"
        return response
    
    def _on_behaviour_recovery_request(self, request, response):

        # TODO: Currently cancel to retry. Add more behaviour
        try:
            self.get_logger().warn("[ORCHESTRATOR] Recovery requested")

            current_goal = self.nav2_controller.get_active_goal()

            if current_goal is None:
                self.get_logger().error("[RECOVERY] No active goal → cannot recover")
                response.accepted = False
                return response

            saved_goal = current_goal

            cancel_ok = self.nav2_controller.cancel_navigation()
            if not cancel_ok:
                self.get_logger().warn("[RECOVERY] Cancel failed or no active goal")

            time.sleep(0.3)

            self.nav2_controller.set_goal(
                x=saved_goal.pose.position.x,
                y=saved_goal.pose.position.y,
                yaw=0.0,
                frame_id=saved_goal.header.frame_id
            )

            self.set_phase(SystemPhase.NAV_EXECUTING)

            response.accepted = True
            response.message = "Recovery executed"
            return response

        except Exception as e:
            self.get_logger().error(f"[RECOVERY ERROR] {str(e)}")
            response.accepted = False
            return response

    def _on_behaviour_cancel_request(self, request, response):

        self.get_logger().warn("[ORCHESTRATOR] Cancel navigation requested")

        success = self.nav2_controller.cancel_navigation()

        if success:
            response.accepted = True
            response.message = "Navigation cancelled"
        else:
            response.accepted = False
            response.message = "No active navigation"

        return response

    # ======================================================
    # PERCEPTION GEOMETRY
    # ======================================================

    def _activate_voxel_costmap_node(self, voxel_map_path: str):
        try:
            while not self._voxel_costmap_set_param_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Waiting for voxel_costmap_node")

            params = Parameter(
                'voxel_map_path',
                Parameter.Type.STRING,
                voxel_map_path
            )

            param_req = SetParameters.Request()
            param_req.parameters = [params.to_parameter_msg()]
            future = self._voxel_costmap_set_param_client.call_async(param_req)
            future.add_done_callback(self._on_set_voxel_map)

        except Exception as e:
            self.get_logger().error(f"Failed to activate perception geometry node ... {e}")

    def _on_set_voxel_map(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Voxel Costmap Set Param failed: {e}")
            return

        for r in result.results:
            if not r.successful:
                self.get_logger().error("Parameter set failed")
                return

        # Start 
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE

        self.get_logger().info("Configuring Voxel Costmap Node...")
        
        future = self.voxel_costmap_client.call_async(req)
        future.add_done_callback(self._on_perception_geometry_configured)

    def _on_perception_geometry_configured(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Voxel Costmap configure Param failed: {e}")
            return

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE

        self.get_logger().info("Activating Voxel Costmap Node...")
        
        future = self.voxel_costmap_client.call_async(req)
        future.add_done_callback(self._on_perception_geometry_activated)

    def _on_perception_geometry_activated(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Voxel Costmap configure Param failed: {e}")
            return
        
    # ------------------------------------------------------

    def _activate_change_detection_node(self, voxel_map_path: str):
        try:
            while not self._voxel_change_detection_set_param_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Waiting for voxel_change_detection_node")

            params = Parameter(
                'voxel_map_path',
                Parameter.Type.STRING,
                voxel_map_path
            )

            param_req = SetParameters.Request()
            param_req.parameters = [params.to_parameter_msg()]
            future = self._voxel_change_detection_set_param_client.call_async(param_req)
            future.add_done_callback(self._on_set_voxel_map_change_detection)

        except Exception as e:
            self.get_logger().error(f"Failed to activate perception geometry node ... {e}")

    def _on_set_voxel_map_change_detection(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Voxel Change Detection Set Param failed: {e}")
            return

        for r in result.results:
            if not r.successful:
                self.get_logger().error("Parameter set failed")
                return

        # Start 
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE

        self.get_logger().info("Configuring Change Detection Node...")
        
        future = self.voxel_change_detection_client.call_async(req)
        future.add_done_callback(self._on_change_detection_configured)

    def _on_change_detection_configured(self, future):

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Voxel Change Detection Param failed: {e}")
            return

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE

        self.get_logger().info("Activating Change Detection Node...")
        
        future = self.voxel_change_detection_client.call_async(req)
        future.add_done_callback(self._on_change_detection_activated)

    def _on_change_detection_activated(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Voxel Change Detection Param failed: {e}")
            return

    # ======================================================
    # Utility
    # ======================================================

    def _log_phase(self, phase: str, msg: str):
        self.get_logger().info(f"\033[92m[{phase}]\033[0m - {msg}")  # Green

    def publish_initial_pose(self):

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.initial_pose_pub.publish(msg)

        self.get_logger().info("[Nav2] Initial pose published")

    def _record_distance(self, distance):

        now = self.get_clock().now().nanoseconds / 1e9

        self._distance_history.append((now, distance))

        window_sec = 5.0
        self._distance_history = [
            (t, d) for (t, d) in self._distance_history
            if now - t < window_sec
        ]

    def _is_stuck(self, feedback):
        if feedback is None:
            return False

        if len(self._distance_history) < 2:
            return False
        
        now = self.get_clock().now().nanoseconds / 1e9
        first_time = self._distance_history[0][0]

        if (now - first_time) < 3.0:
            return False

        distances = [d for (_, d) in self._distance_history]

        progress = max(distances) - min(distances)

        # check progress
        no_progress = progress < 0.1

        # TODO check recovering
        recovering = feedback.recovery_count > 0
        
        return no_progress
    
    # ======================================================
    # Map 
    # ======================================================
    def _save_voxel_map_async(self, path: str):

        if not self._voxel_save_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Voxel save service not available")
            return

        request = SaveVoxelMap.Request()
        request.path = path

        future = self._voxel_save_client.call_async(request)

        future.add_done_callback(self._on_voxel_saved)

    def _on_voxel_saved(self, future):

        try:
            result = future.result()

            if not result.success:
                self.get_logger().error("Voxel map save failed")
                self.set_phase(SystemPhase.ERROR)
                return

            self.get_logger().info("Voxel map saved successfully")
            self._voxel_saved = True
            self._check_all_maps_saved()

        except Exception as e:
            self.get_logger().error(f"Voxel save exception: {e}")
            self.set_phase(SystemPhase.ERROR)

    def _on_slam_saved(self, success: bool):

        if not success:
            self.get_logger().error("2D map save failed")
            self.set_phase(SystemPhase.ERROR)
            return

        self.get_logger().info("2D map saved")

        self._slam_saved = True
        self._check_all_maps_saved()

    def _check_all_maps_saved(self):

        if self._slam_saved and self._voxel_saved:
            self.get_logger().info("All maps saved")

            self.set_phase(SystemPhase.MAP_SAVED)        


# ======================================================
# Main Entry
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()

    executor = rclpy.executors.MultiThreadedExecutor() 
    executor.add_node(node)

    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (Ctrl+C)")
    except Exception as e:
        print(f"Destroy error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()