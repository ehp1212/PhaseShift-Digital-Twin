from typing import Callable, Dict, List, Optional

from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import LoadMap
from lifecycle_msgs.msg import Transition


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
        self._actiavated = False

        # single-thread executor safe polling timer
        self._timer = self.node.create_timer(0.1, self._poll)

    # =========================================================
    # Public API
    # =========================================================

    def is_ready(self):
        return (
            self._configured
            and self._actiavated
        )

    def activate_nav2(self, map_yaml: str):
        self.node.get_logger().info("[Nav2] Activating Nav2 stack")

        def on_activate_success():
            self.node.get_logger().info("[Nav2] ACTIVE")
            self.load_map(map_yaml, on_success=on_map_loaded)

        def on_map_loaded():
            self.node.get_logger().info("[Nav2] Map loaded")
            self._actiavated = True

        def on_configure_success():
            self.node.get_logger().info("[Nav2] Configure complete")
            self._configured = True

            self.activate(
                on_success=on_activate_success,
                on_failure=lambda e: self.node.get_logger().error(f"Activated failed: {e}")
            )
        
        self.configure(
            on_success=on_configure_success,
            on_failure=lambda e: self.node.get_logger().error(f"Configure failed: {e}")
            )

    def deactivate_nav2(self):
        self.node.get_logger().info("[Nav2] deactivating Nav2 stack")

        def on_cleanup_success():
            self.node.get_logger().info("[Nav2] Cleanup complete")
            self._configured = False
            self._actiavated = False

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