import rclpy
import math

from enum import Enum, auto

from typing import Callable, Dict, List, Optional
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import LoadMap
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import Quaternion, PoseStamped
from phaseshift_interfaces.msg import NavigationFeedback

from .nav2_lifecycle_manager import NavLifeCycleManager

class NavStatus:
    def __init__(self):
        self.configured = False
        self.localization = False
        self.activated = False

class NavControllerState(Enum):

    IDLE = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    CANCELED = auto()
    FAILED = auto()
    ERROR = auto()

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class Nav2Controller:

    def __init__(self, node):
        self.node = node

        self.lifecycle = NavLifeCycleManager(node)


        # -----------------------------
        # Map
        # -----------------------------
        self._load_map_client = self.node.create_client(
            LoadMap,
            "/map_server/load_map"
        )

        self._map_loaded = False
        self._map_loading = False

        # -----------------------------
        # Navigation
        # -----------------------------
        self._goal_pose = None

        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            "navigate_to_pose"
        )

        self._planner_client = ActionClient(
            node,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        self._goal_handle = None
        self._result_future = None
        self._goal_future = None

        self._path_future = None
        self._path_result = None
        self._last_result_status = None
        self._state = NavControllerState.IDLE

        # Nav2 Feedback
        self.feedback_pub = node.create_publisher(
            NavigationFeedback,
            "/system/navigation_feedback",
            10
        )




        self._status = NavStatus()
        self._distance_remaining = None

    # =========================================================
    # Public API
    # =========================================================

    def is_ready(self) -> bool:
        return self._status.configured and self._status.activated
    
    def is_error(self):
        return self._state is NavControllerState.ERROR
    
    def is_map_loaded(self) -> bool:
        return self._map_loaded
    
    def is_activated(self) -> bool:
        return self._status.activated
    
    def is_localization_ready(self) -> bool:
        if not self._status.configured:
            self._configure_nav2()    
            return False
        
        if not self._status.localization:
            self._activate_localization()
            return False
        
        return True
        
    def is_navigation_ready(self) -> bool:

        if not self._status.configured:
            return False

        if not self._status.localization:
            return False

        if not self._status.activated:
            self.activate_navigation()
            return False

        return True
    
    def get_active_goal(self) -> PoseStamped:
        return self._goal_pose
    
    def is_succeeded(self):
        return self._last_result_status == GoalStatus.STATUS_SUCCEEDED

    def is_failed(self):
        return self._last_result_status in [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED
        ]

    def activate_navigation(self):

        if self._status.activated:
            return
        
        def on_success():
            self.node.get_logger().info("[NAV2] navigation stack active")
            self._status.activated = True

        self.lifecycle.activate_navigation_stack(
            on_success=on_success,
            on_failure=lambda e: self.node.get_logger().error(e)
        )

    # =========================================================
    # Load map
    # =========================================================
    def load_map(self, map_yaml_path: str):

        if self._map_loading:
            return

        if not self._load_map_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("load_map service unavailable")
            return

        req = LoadMap.Request()
        req.map_url = map_yaml_path

        self._map_loading = True

        future = self._load_map_client.call_async(req)

        future.add_done_callback(self._on_map_loaded)


    def _on_map_loaded(self, future):
        self._map_loading = False

        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"map load exception: {e}")
            return

        if result.result != LoadMap.Response.RESULT_SUCCESS:
            self.node.get_logger().error("map load failed")
            return

        self.node.get_logger().info("[NAV2] map loaded")
        self._map_loaded = True
    
    # =========================================================
    # Set Goal
    # =========================================================

    def set_goal(self, x: float, y: float, yaw: float, frame_id: str):
        
        pose = self._build_pose(x, y, yaw, "map")
        self._goal_pose = pose

        self.node.get_logger().info(
            f"[NAV2] Goal set ({x:.2f}, {y:.2f}, yaw={yaw:.2f})"
        )

        # Preview
        self._compute_path()

        # Navigate
        self.navigate_to_goal()

    def navigate_to_goal(self):

        if self._goal_pose is None:
            self.node.get_logger().error("Goal not set")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose

        self.node.get_logger().info("[NAV2] Navigation started")
        self._goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
            )
        
        self._goal_future.add_done_callback(self._goal_response_callback)

        self._goal_handle = None
        self._result_future = None

    def tick_navigation(self) -> bool:

        if self._goal_future is None:
            self._state = NavControllerState.IDLE
            return False

        if self._goal_handle is None:

            if not self._goal_future.done():
                self._state = NavControllerState.RUNNING
                return False

            self._goal_handle = self._goal_future.result()

            if not self._goal_handle.accepted:
                self.node.get_logger().error("[NAV2CONTROLLER] Goal rejected")
                self._goal_future = None
                self._state = NavControllerState.FAILED
                return False
        
            self.node.get_logger().info("[NAV2CONTROLLER] Goal accepted")
            
            self._result_future = self._goal_handle.get_result_async()
            self._state = NavControllerState.RUNNING
            return False
        
        # Check navigation result
        if self._result_future is not None:

            if not self._result_future.done():
                self._state = NavControllerState.RUNNING
                return False
            
            result = self._result_future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("[NAV2CONTROLLER] Goal reached")

                self._reset_navigation()
                self._state = NavControllerState.SUCCEEDED
                return True

            elif status == GoalStatus.STATUS_ABORTED: 
                self.node.get_logger().error("[NAV2CONTROLLER] Navigation aborted")

                self._reset_navigation()
                self._state = NavControllerState.FAILED
                return False

            elif status == GoalStatus.STATUS_CANCELED:  
                self.node.get_logger().warn("[NAV2CONTROLLER] Navigation canceled")

                self._reset_navigation()

                self._state = NavControllerState.CANCELED
                return False
        
        self._state = NavControllerState.RUNNING
        return False
    
    def cancel_navigation(self):
        if self._goal_handle is None:
            self.node.get_logger().warn("[NAV2CONTROLLER] No active goal to cancel")
            return False

        self.node.get_logger().info("[NAV2CONTROLLER] Canceling current goal")

        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_done_callback)

        return True

    def _cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            self.node.get_logger().info(f"[NAV2CONTROLLER] Cancel response: {cancel_response}")
        except Exception as e:
            self.node.get_logger().error(f"[NAV2CONTROLLER] Cancel failed: {e}")

    # =========================================================
    # Nav Feedback
    # =========================================================
    def _feedback_callback(self, msg):
        
        feedback = msg.feedback

        nav_msg = NavigationFeedback()
        nav_msg.distance_remaining = feedback.distance_remaining
        nav_msg.navigation_time = float(feedback.navigation_time.sec)
        nav_msg.estimated_time_remaining = float(feedback.estimated_time_remaining.sec)
        nav_msg.current_pose = feedback.current_pose
        nav_msg.recovery_count = feedback.number_of_recoveries

        self.feedback_pub.publish(nav_msg)

    def _goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            return

        self._goal_handle = goal_handle

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):

        result = future.result()
        status = result.status 

        self.node.get_logger().info(f"Result received: {status}")

        self._last_result_status = status
    
    # =========================================================
    # Internal operation bootstrap
    # =========================================================
    
    def _reset_navigation(self):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def _compute_path(self) -> bool: 

        if self._goal_pose is None:
            self.node.get_logger().error("Goal not set")
            return False
        
        if not self._planner_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("Planner server not available")
            return False

        goal = ComputePathToPose.Goal()

        goal.goal = self._goal_pose
        goal.use_start = False
        goal.planner_id = ""    

        self._path_future = self._planner_client.send_goal_async(goal)
        self.node.get_logger().info("[NAV2CONTROLLER] ComputePathToPose requested")
        return True
    
    def _configure_nav2(self):

        if self._status.configured:
           return

        def on_success():
            self.node.get_logger().info("[NAV2CONTROLLER] Configure complete")
            self._status.configured = True

        self.lifecycle.configure(
            on_success=on_success,
            on_failure=lambda e: self.node.get_logger().error(f"Configure failed: {e}")
        )

    def _activate_localization(self):

        if self._status.localization:
            return

        def on_success():
            self.node.get_logger().info("[NAV2CONTROLLER] map_server + amcl active")
            self._status.localization = True

        self.lifecycle.activate_localization(
            on_success=on_success,
            on_failure=lambda e: self.node.get_logger().error(e)
        )

    
    # =========================================================
    # UTILS
    # =========================================================

    def _build_pose(self, x, y, yaw, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose