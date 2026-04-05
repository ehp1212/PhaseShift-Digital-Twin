from enum import Enum, auto
from typing import Dict

from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

# ========================================
# ENUMS
# ========================================

class PerceptionControllerState(Enum):
    IDLE = auto()
    CONFIGURING = auto()
    INACTIVE = auto()
    ACTIVATING = auto()
    ACTIVE = auto()
    DEACTIVATING = auto()
    CLEANING_UP = auto()
    ERROR = auto()

class ManagedNode(Enum):
    YOLO = "/yolo_detector_node"
    YOLO_TRACKER = "/yolo_tracker_node"
    PROJECTION = "/projection_node"
    MEMORY = "/detection_memory_node"
    VOXEL_EST = "/voxel_state_estimator_node"

class LifecycleStage(Enum):
    CONFIGURED = auto()
    ACTIVATED = auto()
    DEACTIVATED = auto()
    CLEANED = auto()

# ========================================
# CONTROLLER
# ========================================

class PerceptionController:

    CONFIGURE_ID = Transition.TRANSITION_CONFIGURE
    ACTIVATE_ID = Transition.TRANSITION_ACTIVATE
    DEACTIVATE_ID = Transition.TRANSITION_DEACTIVATE
    CLEANUP_ID = Transition.TRANSITION_CLEANUP

    def __init__(self, node: Node):
        self.node = node

        self._clients: Dict[ManagedNode, any] = {}
        self._flags: Dict[ManagedNode, Dict[LifecycleStage, bool]] = {}

        for mnode in ManagedNode:
            self._clients[mnode] = node.create_client(
                ChangeState,
                f"{mnode.value}/change_state"
            )

            self._flags[mnode] = {
                stage: False for stage in LifecycleStage
            }

        self._state = PerceptionControllerState.IDLE

    # ========================================
    # PUBLIC API
    # ========================================

    @property
    def state(self):
        return self._state

    def is_active(self):
        return self._state == PerceptionControllerState.ACTIVE

    def is_idle(self):
        return self._state == PerceptionControllerState.IDLE

    def activate(self) -> bool:

        if self._state not in (
            PerceptionControllerState.IDLE,
            PerceptionControllerState.INACTIVE
        ):
            self.node.get_logger().warn(
                f'[PERCEPTION] Activate ignored. Current state: {self._state.name}'
            )
            return False

        if not self._wait_for_service():
            self._set_error("Lifecycle node service unavailable")
            return False

        self.node.get_logger().info("[PERCEPTION] Activating perception subsystem...")

        self._state = PerceptionControllerState.CONFIGURING
        self._reset_flags([LifecycleStage.CONFIGURED, LifecycleStage.CLEANED])

        self._call_all(
            self.CONFIGURE_ID,
            lambda node: self._make_callback(
                node,
                LifecycleStage.CONFIGURED,
                self._try_activate
            )
        )

        return True

    def deactivate(self) -> bool:

        if self._state != PerceptionControllerState.ACTIVE:
            self.node.get_logger().warn(
                f'[PERCEPTION] Deactivate ignored. Current state: {self._state.name}'
            )
            return False

        if not self._wait_for_service():
            self._set_error("Lifecycle service unavailable")
            return False

        self.node.get_logger().info("[PERCEPTION] Deactivating perception subsystem...")

        self._state = PerceptionControllerState.DEACTIVATING
        self._reset_flags([LifecycleStage.DEACTIVATED])

        self._call_all(
            self.DEACTIVATE_ID,
            lambda node: self._make_callback(
                node,
                LifecycleStage.DEACTIVATED,
                self._try_cleanup
            )
        )

        return True

    # ========================================
    # INTERNAL
    # ========================================

    def _wait_for_service(self) -> bool:
        ok = True

        for node, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error(
                    f"[PERCEPTION] Service unavailable: {node.value}/change_state"
                )
                ok = False

        return ok

    def _call(self, client, transition_id, callback=None):
        try:
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = client.call_async(req)

            if callback:
                future.add_done_callback(callback)

            return future

        except Exception as e:
            self.node.get_logger().error(f"[PERCEPTION] Call failed: {e}")

    def _call_all(self, transition_id, callback_factory):
        for node in ManagedNode:
            self._call(
                self._clients[node],
                transition_id,
                callback_factory(node)
            )

    def _make_callback(self, node, stage, next_step):
        def callback(future):
            if not self._check_future_success(future):
                self._set_error(f"{node.name} failed at {stage.name}")
                return

            self.node.get_logger().info(
                f"[PERCEPTION] {node.name} {stage.name} success"
            )

            self._flags[node][stage] = True
            next_step()

        return callback

    def _check_future_success(self, future) -> bool:
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(
                f"[PERCEPTION] Exception: {e}"
            )
            return False

        if result is None:
            self.node.get_logger().error("[PERCEPTION] No result")
            return False

        if not result.success:
            self.node.get_logger().error("[PERCEPTION] success=False")
            return False

        return True

    def _all_done(self, stage: LifecycleStage) -> bool:
        return all(self._flags[node][stage] for node in ManagedNode)

    def _reset_flags(self, stages):
        for node in ManagedNode:
            for stage in stages:
                self._flags[node][stage] = False

    def _set_error(self, msg):
        self.node.get_logger().error(f"[PERCEPTION] {msg}")
        self._state = PerceptionControllerState.ERROR

    # ========================================
    # CHAIN LOGIC
    # ========================================

    def _try_activate(self):
        if not self._all_done(LifecycleStage.CONFIGURED):
            return

        self.node.get_logger().info("[PERCEPTION] All configured → activating")

        self._state = PerceptionControllerState.ACTIVATING

        self._call_all(
            self.ACTIVATE_ID,
            lambda node: self._make_callback(
                node,
                LifecycleStage.ACTIVATED,
                self._try_set_active
            )
        )

    def _try_set_active(self):
        if not self._all_done(LifecycleStage.ACTIVATED):
            return

        self._state = PerceptionControllerState.ACTIVE
        self.node.get_logger().info("[PERCEPTION] Subsystem ACTIVE")

    def _try_cleanup(self):
        if not self._all_done(LifecycleStage.DEACTIVATED):
            return

        self.node.get_logger().info("[PERCEPTION] All deactivated → cleanup")

        self._state = PerceptionControllerState.CLEANING_UP

        self._call_all(
            self.CLEANUP_ID,
            lambda node: self._make_callback(
                node,
                LifecycleStage.CLEANED,
                self._try_set_inactive
            )
        )

    def _try_set_inactive(self):
        if not self._all_done(LifecycleStage.CLEANED):
            return

        self._state = PerceptionControllerState.INACTIVE
        self.node.get_logger().info("[PERCEPTION] Subsystem INACTIVE")