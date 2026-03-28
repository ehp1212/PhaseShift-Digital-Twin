import rclpy
from rclpy.node import Node

from typing import Callable, Dict, List, Optional
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

class NavLifeCycleManager:

    CONFIGURE_ID = Transition.TRANSITION_CONFIGURE
    ACTIVATE_ID = Transition.TRANSITION_ACTIVATE
    DEACTIVATE_ID = Transition.TRANSITION_DEACTIVATE
    CLEANUP_ID = Transition.TRANSITION_CLEANUP

    CONFIGURE_ORDER = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    DEACTIVATE_ORDER = list(reversed(CONFIGURE_ORDER))

    def __init__(self, node: Node):
        self._node = node

        # -----------------------------
        # Cached service clients
        # -----------------------------
        self._change_state_clients: Dict[str, ChangeState] = {}
        self._get_state_clients: Dict[str, GetState] = {}

        for node_name in self.CONFIGURE_ORDER:
            self._change_state_clients[node_name] = self._node.create_client(
                ChangeState,
                f"/{node_name}/change_state"
            )
            self._get_state_clients[node_name] = self._node.create_client(
                GetState,
                f"/{node_name}/get_state"
            )

        # -----------------------------
        # Internal operation state
        # -----------------------------
        self._queue: List[str] = []
        self._transition_id: Optional[int] = None
        self._expected_state: Optional[str] = None

        self._on_success: Optional[Callable[[], None]] = None
        self._on_failure: Optional[Callable[[str], None]] = None

    # =========================================================
    # Public API
    # =========================================================
    
    def is_busy(self) -> bool:
        return len(self._queue) > 0 

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
    
    def activate_navigation_stack(self, on_success, on_failure):
        return self._activate_nodes(
            ["planner_server", "controller_server", "behavior_server", "bt_navigator"],
            on_success,
            on_failure
        )

    def activate_localization(self, on_success, on_failure):
        return self._activate_nodes(
            ["map_server", "amcl"],
            on_success,
            on_failure
        )
        
    def _activate_nodes(self, nodes: List[str], on_success, on_failure):
        return self._start_transition_operation(
            op_name="activate_custom",
            node_order=nodes,
            transition_id=self.ACTIVATE_ID,
            expected_state="active",
            on_success=on_success,
            on_failure=on_failure
        )

    # =========================================================
    # TRANSITION OPERATION
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
            self._node.get_logger().warn(f"Nav2Controller is busy. {op_name} request ignored.")
            return False
        
        self._queue = list(node_order)
        self._transition_id = transition_id
        self._expected_state = expected_state
        self._on_success = on_success
        self._on_failure = on_failure

        self._node.get_logger().info(f"[NavLifecycle] start {op_name}")
        self._request_next()
        return True
    
    def _request_next(self):
        if not self._queue:
            self._succeed()
            return

        node_name = self._queue.pop(0)

        client = self._change_state_clients[node_name]

        if not client.wait_for_service(timeout_sec=1.0):
            self._fail(f"{node_name}: change_state service not available")
            return

        req = ChangeState.Request()
        req.transition.id = self._transition_id

        self._node.get_logger().info(
            f"[NavLifecycle] {node_name}: transition {self._transition_id}"
        )

        future = client.call_async(req)
        future.add_done_callback(
            lambda f, n=node_name: self._on_transition_done(f, n)
        )

    def _on_transition_done(self, future, node_name: str):
        try:
            result = future.result()
        except Exception as e:
            self._fail(f"{node_name}: transition exception: {e}")
            return

        if result is None or not result.success:
            self._fail(f"{node_name}: transition rejected")
            return

        self._node.get_logger().info(
            f"[NavLifecycle] {node_name}: transition accepted"
        )

        self._request_verify(node_name)

    def _request_verify(self, node_name: str):
        client = self._get_state_clients[node_name]

        if not client.wait_for_service(timeout_sec=1.0):
            self._fail(f"{node_name}: get_state service not available")
            return

        req = GetState.Request()
        future = client.call_async(req)

        future.add_done_callback(
            lambda f, n=node_name: self._on_verify_done(f, n)
        )

    def _on_verify_done(self, future, node_name: str):
        try:
            result = future.result()
        except Exception as e:
            self._fail(f"{node_name}: get_state exception: {e}")
            return

        if result is None:
            self._fail(f"{node_name}: get_state returned None")
            return

        label = result.current_state.label

        self._node.get_logger().info(
            f"[NavLifecycle] {node_name}: state = {label}"
        )

        if label != self._expected_state:
            self._fail(
                f"{node_name}: expected '{self._expected_state}', got '{label}'"
            )
            return

        self._request_next()

    # =========================================================
    # Result handling
    # =========================================================

    def _clear(self):
        self._queue = []
        self._transition_id = None
        self._expected_state = None
        self._on_success = None
        self._on_failure = None

    def _succeed(self):
        callback = self._on_success
        self._clear()

        if callback is not None:
            callback()

    def _fail(self, msg: str, failure_cb: Optional[Callable[[str], None]] = None):
        self._node.get_logger().error(f"[NavLifecycle] {msg}")

        callback = failure_cb if failure_cb is not None else self._on_failure

        self._clear()

        if callback is not None:
            callback(msg)