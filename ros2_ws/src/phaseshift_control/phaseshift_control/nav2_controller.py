import rclpy
import math

from typing import Callable, Dict, List, Optional
from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import LoadMap
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import Quaternion, PoseStamped
from phaseshift_interfaces.msg import NavigationFeedback

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class Nav2Controller:
    CONFIGURE_ID = Transition.TRANSITION_CONFIGURE      # 1
    ACTIVATE_ID = Transition.TRANSITION_ACTIVATE        # 3
    DEACTIVATE_ID = Transition.TRANSITION_DEACTIVATE    # 4
    CLEANUP_ID = Transition.TRANSITION_CLEANUP          # 5

    CONFIGURE_ORDER = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    DEACTIVATE_ORDER = list(reversed(CONFIGURE_ORDER))

    def __init__(self, node):
        self.node = node

        # -----------------------------
        # Cached service clients
        # -----------------------------
        self._change_state_clients: Dict[str, ChangeState] = {}
        self._get_state_clients: Dict[str, GetState] = {}

        for node_name in self.CONFIGURE_ORDER:
            self._change_state_clients[node_name] = self.node.create_client(
                ChangeState,
                f"/{node_name}/change_state"
            )
            self._get_state_clients[node_name] = self.node.create_client(
                GetState,
                f"/{node_name}/get_state"
            )

        self._load_map_client = self.node.create_client(
            LoadMap,
            "/map_server/load_map"
        )

        # Set Goal
        self._goal_pose = None
        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            "navigate_to_pose"
        )

        self._goal_handle = None
        self._result_future = None

        # Preview
        self._planner_client = ActionClient(
            node,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        self._path_future = None
        self._goal_future = None
        self._path_result = None

        # Nav2 Feedback
        self.feedback_pub = node.create_publisher(
            NavigationFeedback,
            "/system/navigation_feedback",
            10
        )

        self._distance_remaining = None

        # -----------------------------
        # Internal operation state
        # -----------------------------
        self._op_name: Optional[str] = None
        self._op_queue: List[str] = []
        self._op_transition_id: Optional[int] = None
        self._op_current_node: Optional[str] = None
        self._op_future = None
        self._op_verify_future = None
        self._op_expected_state: Optional[str] = None

        self._load_map_future = None
        self._load_map_path: Optional[str] = None

        self._on_success: Optional[Callable[[], None]] = None
        self._on_failure: Optional[Callable[[str], None]] = None

        self._configured = False
        self._is_configuring = False

        self._localization_active = False
        self._is_localization_activating = False

        self._map_loaded = False
        self._map_loading = False

        self._activated = False
        self._activating = False

        # single-thread executor safe polling timer
        self._timer = self.node.create_timer(0.1, self._poll)

    # =========================================================
    # Public API
    # =========================================================

    def is_ready(self):
        return (
            self._configured
            and self._activated
        )
    
    def is_configured(self) -> bool:
        return self._configured
    
    def is_localization_activated(self) -> bool:
        return self._localization_active
    
    def is_map_loaded(self) -> bool:
        return self._map_loaded
    
    def is_activated(self) -> bool:
        return self._activated
    
    def configure_nav2(self):

        if self._configured or self._is_configuring:
           return

        self._is_configuring = True

        def on_configure_success():
            self.node.get_logger().info("[Nav2] Configure complete")
            self._configured = True
            self._is_configuring = False

        self.configure(
            on_success=on_configure_success,
            on_failure=lambda e: self.node.get_logger().error(f"Configure failed: {e}")
        )

    def deactivate_nav2(self):
   
        self.node.get_logger().info("[Nav2] deactivating Nav2 stack")

        def on_cleanup_success():
            self.node.get_logger().info("[Nav2] Cleanup complete")
            self._configured = False
            self._activated = False

        def on_cleanup_failure(err: str):
            self.node.get_logger().error(f"[Nav2] Cleanup failed: {err}")

        def on_deactivate_success():
            self.node.get_logger().info("[Nav2] Deactivate complete")
            self.activate = False

            # Clean up after deactivate
            self.cleanup(
                on_success=on_cleanup_success,
                on_failure=on_cleanup_failure
            )

        def on_deactivate_failure(err: str):
            self.node.get_logger().error(f"[Nav2] Deactivate failed: {err}")

        self.deactivate(
            on_success=on_deactivate_success,
            on_failure=on_deactivate_failure
        )

    def is_localization_ready(self) -> bool:
        self.configure_nav2()    
        if not self.is_configured():
            return False
        
        self.activate_localization()
        if not self.is_localization_activated():
            return False
        
        return True
    
    def is_navigation_ready(self) -> bool:
        self.activate_navigation()
        if not self.is_activated():
            return False
        
        return True

    def load_map_nav2(self, map_yaml):

        if self._map_loaded or self._map_loading:
            return

        self._map_loading = True

        def on_success():
            self.node.get_logger().info("[Nav2] map loaded")
            self._map_loaded = True
            self._map_loading = False

        self.load_map(
            map_yaml_path=map_yaml,
            on_success=on_success,
            on_failure=lambda e: self.node.get_logger().error(e)
        )

    def activate_localization(self):

        if self._localization_active or self._is_localization_activating:
            return
        
        self._is_localization_activating = True

        def on_success():
            self.node.get_logger().info("[Nav2] map_server + amcl active")
            self._is_localization_activating = False
            self._localization_active = True

        self._start_transition_operation(
            op_name="activate_localization",
            node_order=["map_server", "amcl"],
            transition_id=self.ACTIVATE_ID,
            expected_state="active",
            on_success=on_success,
            on_failure=lambda e: self.node.get_logger().error(e)
        )

    def activate_navigation(self):

        if self._activated or self._activating:
            return
        
        self._activating = True

        def on_success():
            self.node.get_logger().info("[Nav2] navigation stack active")
            self._activated = True
            self._activating = False

        self._start_transition_operation(
            op_name="activate_navigation",
            node_order=[
                "planner_server",
                "controller_server",
                "behavior_server",
                "bt_navigator",
            ],
            transition_id=self.ACTIVATE_ID,
            expected_state="active",
            on_success=on_success,
            on_failure=lambda e: self.node.get_logger().error(e)
        )

    def configure(
        self,
        on_success: Optional[Callable[[], None]] = None,
        on_failure: Optional[Callable[[str], None]] = None,
    ) -> bool:
        
        return self._start_transition_operation(
            op_name="configure",
            node_order=self.CONFIGURE_ORDER,
            transition_id=self.CONFIGURE_ID,
            expected_state="inactive",
            on_success=on_success,
            on_failure=on_failure,
        )

    def activate(
        self,
        on_success: Optional[Callable[[], None]] = None,
        on_failure: Optional[Callable[[str], None]] = None,
    ) -> bool:
        
        return self._start_transition_operation(
            op_name="activate",
            node_order=self.CONFIGURE_ORDER,
            transition_id=self.ACTIVATE_ID,
            expected_state="active",
            on_success=on_success,
            on_failure=on_failure,
        )

    def deactivate(
        self,
        on_success: Optional[Callable[[], None]] = None,
        on_failure: Optional[Callable[[str], None]] = None,
    ) -> bool:
        
        return self._start_transition_operation(
            op_name="deactivate",
            node_order=self.DEACTIVATE_ORDER,
            transition_id=self.DEACTIVATE_ID,
            expected_state="inactive",
            on_success=on_success,
            on_failure=on_failure,
        )

    def cleanup(
        self,
        on_success: Optional[Callable[[], None]] = None,
        on_failure: Optional[Callable[[str], None]] = None,
    ) -> bool:
        
        return self._start_transition_operation(
            op_name="cleanup",
            node_order=self.DEACTIVATE_ORDER,
            transition_id=self.CLEANUP_ID,
            expected_state="unconfigured",
            on_success=on_success,
            on_failure=on_failure,
        )

    # =========================================================
    # Load map
    # =========================================================
    def load_map(
        self,
        map_yaml_path: str,
        on_success: Optional[Callable[[], None]] = None,
        on_failure: Optional[Callable[[str], None]] = None,
    ) -> bool:
        
        if self.is_busy():
            self.node.get_logger().warn("Nav2Controller is busy. load_map request ignored.")
            return False

        if not self._load_map_client.wait_for_service(timeout_sec=1.0):
            self._fail("load_map service is not available", on_failure)
            return False

        req = LoadMap.Request()
        req.map_url = map_yaml_path

        self._load_map_path = map_yaml_path
        self._load_map_future = self._load_map_client.call_async(req)
        self._on_success = on_success
        self._on_failure = on_failure

        self.node.get_logger().info(f"[nav2] load_map requested: {map_yaml_path}")
        return True

    def is_busy(self) -> bool:
        
        return (
            self._op_future is not None or
            self._op_verify_future is not None or
            self._load_map_future is not None or
            len(self._op_queue) > 0
        )
    
    # =========================================================
    # Set Goal
    # =========================================================

    def set_goal(self, x: float, y: float, yaw: float, frame_id: str):
        
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation = yaw_to_quaternion(yaw)

        self._goal_pose = pose

        self.node.get_logger().info(
            f"[Nav2] Goal set ({x:.2f}, {y:.2f}, yaw={yaw:.2f})"
        )

        # Preview
        self.compute_path()

        # Navigate
        self.navigate_to_goal()

    def navigate_to_goal(self):

        if self._goal_pose is None:
            self.node.get_logger().error("Goal not set")
            return
        
        # TODO: Check server is active
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose

        self.node.get_logger().info("[NAV2] Navigation started")
        self._goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
            )

        self._goal_handle = None
        self._result_future = None

    def update_navigation(self):

        # No goal request
        if self._goal_future is None:
            return "IDLE"
        
        # Check goal handle
        if self._goal_handle is None:

            if not self._goal_future.done():
                return "RUNNING"
            
            self._goal_handle = self._goal_future.result()

            if not self._goal_handle.accepted:
                self.node.get_logger().error("[NAV2] Goal rejected")
                self._goal_future = None
                return "FAILED"
            
            self.node.get_logger().info("[NAV2] Goal accepted")

            # Result future
            self._result_future = self._goal_handle.get_result_async()
            return "RUNNING"

        # Check navigation result
        if self._result_future is not None:

            if not self._result_future.done():
                return "RUNNING"
            
            result = self._result_future.result()
            status = result.status

            if status == 4:  # STATUS_SUCCEEDED
                self.node.get_logger().info("[Nav2] Goal reached")

                self._reset_navigation()

                return "SUCCEEDED"

            elif status == 6:  # STATUS_ABORTED
                self.node.get_logger().error("[Nav2] Navigation aborted")

                self._reset_navigation()

                return "FAILED"

            elif status == 5:  # STATUS_CANCELED
                self.node.get_logger().warn("[Nav2] Navigation canceled")

                self._reset_navigation()

                return "CANCELED"
        
        return "RUNNING"

    def _reset_navigation(self):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def compute_path(self) -> bool: 

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
        self.node.get_logger().info("[Nav2] ComputePathToPose requested")
        return True

    # =========================================================
    # Nav Feedback
    # =========================================================
    def _feedback_callback(self, msg):
        
        feedback = msg.feedback

        nav_msg = NavigationFeedback()
        nav_msg.distance_remaining = feedback.distance_remaining
        nav_msg.current_pose = feedback.current_pose.pose
        nav_msg.navigation_time = float(feedback.navigation_time.sec)
        nav_msg.estimated_time_remaining = float(feedback.estimated_time_remaining.sec)
        nav_msg.recovery_count = feedback.number_of_recoveries

        self.feedback_pub.publish(nav_msg)
    
    # =========================================================
    # Internal operation bootstrap
    # =========================================================

    def _start_transition_operation(
        self,
        op_name: str,
        node_order: List[str],
        transition_id: int,
        expected_state: str,
        on_success: Optional[Callable[[], None]],
        on_failure: Optional[Callable[[str], None]],
    ) -> bool:
        if self.is_busy():
            self.node.get_logger().warn(f"Nav2Controller is busy. {op_name} request ignored.")
            return False
        self._op_name = op_name
        self._op_queue = list(node_order)
        self._op_transition_id = transition_id
        self._op_expected_state = expected_state
        self._op_current_node = None
        self._op_future = None
        self._op_verify_future = None
        self._on_success = on_success
        self._on_failure = on_failure

        self.node.get_logger().info(f"[nav2] start {op_name}")
        self._request_next_transition()
        return True

    # =========================================================
    # Polling loop
    # =========================================================

    def _poll(self):
        self._poll_check_path()
        self._poll_load_map()
        self._poll_transition()
        self._poll_state_verification()

    def _poll_load_map(self):
        if self._load_map_future is None:
            return

        if not self._load_map_future.done():
            return

        try:
            result = self._load_map_future.result()
        except Exception as e:
            self._load_map_future = None
            self._fail(f"load_map exception: {e}")
            return

        self._load_map_future = None

        if result is None:
            self._fail("load_map returned None")
            return

        # nav2_msgs/srv/LoadMap.Response.result
        # 보통 RESULT_SUCCESS == 0
        if result.result != LoadMap.Response.RESULT_SUCCESS:
            self._fail(f"load_map failed with code: {result.result}")
            return

        self.node.get_logger().info(f"[nav2] load_map success: {self._load_map_path}")
        self._succeed()

    def _poll_transition(self):
        if self._op_future is None:
            return

        if not self._op_future.done():
            return

        node_name = self._op_current_node
        transition_id = self._op_transition_id
        self._op_future_local_clear()

        try:
            result = self._last_future_result
        except AttributeError:
            self._fail(f"{node_name}: internal future result missing")
            return

        if result is None:
            self._fail(f"{node_name}: transition {transition_id} returned None")
            return

        if not result.success:
            self._fail(f"{node_name}: transition {transition_id} rejected")
            return

        self.node.get_logger().info(
            f"[nav2] {node_name}: transition {transition_id} accepted"
        )

        self._request_get_state(node_name)

    def _poll_state_verification(self):
        if self._op_verify_future is None:
            return

        if not self._op_verify_future.done():
            return

        node_name = self._op_current_node
        expected = self._op_expected_state
        self._op_verify_future_local_clear()

        try:
            result = self._last_verify_result
        except AttributeError:
            self._fail(f"{node_name}: internal verify result missing")
            return

        if result is None:
            self._fail(f"{node_name}: get_state returned None")
            return

        label = result.current_state.label
        self.node.get_logger().info(f"[nav2] {node_name}: current state = {label}")

        if label != expected:
            self._fail(f"{node_name}: expected state '{expected}', got '{label}'")
            return

        self._request_next_transition()

    def _poll_check_path(self):

        if self._path_future is None:
            return None

        if not self._path_future.done():
            return None

        goal_handle = self._path_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("Path request rejected")
            return None

        result_future = goal_handle.get_result_async()

        if not result_future.done():
            return None

        result = result_future.result().result

        self._path_result = result.path

        return self._path_result

    # =========================================================
    # Transition flow
    # =========================================================

    def _request_next_transition(self):
        if not self._op_queue:
            self.node.get_logger().info(f"[nav2] {self._op_name} complete")
            self._clear_operation_state()
            self._succeed()
            return

        node_name = self._op_queue.pop(0)
        self._op_current_node = node_name

        client = self._change_state_clients[node_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self._fail(f"{node_name}: change_state service not available")
            return

        req = ChangeState.Request()
        req.transition.id = self._op_transition_id

        self.node.get_logger().info(
            f"[nav2] {self._op_name}: request transition {self._op_transition_id} -> {node_name}"
        )

        self._op_future = client.call_async(req)

    def _request_get_state(self, node_name: str):
        client = self._get_state_clients[node_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self._fail(f"{node_name}: get_state service not available")
            return

        req = GetState.Request()
        self._op_verify_future = client.call_async(req)

    # =========================================================
    # Helpers
    # =========================================================

    def _op_future_local_clear(self):
        try:
            self._last_future_result = self._op_future.result()
        except Exception as e:
            self._op_future = None
            self._fail(f"{self._op_current_node}: transition future exception: {e}")
            return
        self._op_future = None

    def _op_verify_future_local_clear(self):
        try:
            self._last_verify_result = self._op_verify_future.result()
        except Exception as e:
            self._op_verify_future = None
            self._fail(f"{self._op_current_node}: get_state future exception: {e}")
            return
        self._op_verify_future = None

    def _clear_operation_state(self):
        self._op_name = None
        self._op_queue = []
        self._op_transition_id = None
        self._op_current_node = None
        self._op_future = None
        self._op_verify_future = None
        self._op_expected_state = None

    def _succeed(self):
        callback = self._on_success
        self._on_success = None
        self._on_failure = None
        if callback is not None:
            callback()

    def _fail(self, msg: str, failure_cb: Optional[Callable[[str], None]] = None):
        self.node.get_logger().error(f"[nav2] {msg}")

        self._clear_operation_state()
        self._load_map_future = None
        self._load_map_path = None

        callback = failure_cb if failure_cb is not None else self._on_failure

        self._on_success = None
        self._on_failure = None

        if callback is not None:
            callback(msg)