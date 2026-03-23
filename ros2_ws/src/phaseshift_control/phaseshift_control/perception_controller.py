
from enum import Enum, auto

from rclpy.node import Node

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

class PerceptionControllerState(Enum):
    IDLE = auto()
    CONFIGURING = auto()
    INACTIVE = auto()
    ACTIVATING = auto()
    ACTIVE = auto()
    DEACTIVATE = auto()
    CLEANING_UP = auto()
    ERROR = auto()

class PerceptionController:
    
    CONFIGURE_ID = Transition.TRANSITION_CONFIGURE      # 1
    ACTIVATE_ID = Transition.TRANSITION_ACTIVATE        # 3
    DEACTIVATE_ID = Transition.TRANSITION_DEACTIVATE    # 4
    CLEANUP_ID = Transition.TRANSITION_CLEANUP          # 5

    def __init__(self, node: Node):
        self.node = node

        self._yolo_name = '/yolo_detector_node'
        self._projection_name = '/projection_node'

        # YOLO node client
        self._yolo_client = node.create_client(
            ChangeState,
            f'{self._yolo_name}/change_state'
        )

        # Projection node client
        self._projection_client = node.create_client(
            ChangeState,
            f'{self._projection_name}/change_state'
        )

        self._state = PerceptionControllerState.IDLE

        self._yolo_configured = False
        self._projection_configured = False
        self._yolo_deactivated = False
        self._projection_deactivated = False

        self._yolo_activated = False
        self._projection_activated = False

        self._yolo_cleaned = False
        self._projection_cleaned = False

    # ========================================
    # PUBLIC API
    # ========================================
    @property
    def state(self) -> PerceptionControllerState:
        return self._state
    
    def is_active(self) -> bool:
        return self._state == PerceptionControllerState.ACTIVE
    
    def is_idle(self) -> bool:
        return self._state == PerceptionControllerState.IDLE
    
    def activate(self) -> bool:
        if self._state not in (PerceptionControllerState.IDLE, 
                               PerceptionControllerState.INACTIVE, ):
            
            self.node.get_logger().warn(
                f'[PERCEPTION] Activate ignored. Current state: {self._state.name}')
            return False
        
        if not self._wait_for_service():
            self._set_error('Lifecycle node service unavailable')
            return False

        self.node.get_logger().info('[PERCEPTION] Activating perception subsystem...')        
        self._state = PerceptionControllerState.CONFIGURING

        self._yolo_configured = False
        self._projection_configured = False
        
        # Activate process YOLO     
        self._call(
            self._yolo_client,
            self.CONFIGURE_ID,
            self._on_yolo_configured
        )
        
        # Activate process PROJECTION
        self._call(
            self._projection_client,
            self.CONFIGURE_ID,
            self._on_projection_configured
        )

    def deactivate(self) -> bool:
        if self._state != PerceptionControllerState.ACTIVE:
            self.node.get_logger().warn(
                f'[PERCEPTION] Deactivate ignored. Current state: {self._state.name}'
            )
            return False 
        
        if not self._wait_for_services():
            self._set_error('Lifecycle service unavailable')
            return False
        
        self.node.get_logger().info('[PERCEPTION] Deactivating perception subsystem...')
        self._state = PerceptionControllerState.DEACTIVATING

        self._yolo_deactivated = False
        self._projection_deactivated = False

        # Cleanup process YOLO
        self._call(
            self._yolo_client,
            self.DEACTIVATE_ID,
            self._on_yolo_deactivated
        )
        
        # Cleanup process PROJECTION
        self._call(
            self._projection_client,
            self.DEACTIVATE_ID,
            self._on_projection_deactivated
        )

    # ========================================
    # INTERNAL
    # ========================================

    def _wait_for_service(self) -> bool:
        ok_yolo = self._yolo_client.wait_for_service(timeout_sec=1.0)
        ok_proj = self._projection_client.wait_for_service(timeout_sec=1.0)

        if not ok_yolo:
            self.node.get_logger().error(
                f'[PERCEPTION] Service unavailable: {self._yolo_name}/change_state'
            )
        if not ok_proj:
            self.node.get_logger().error(
                f'[PERCEPTION] Service unavailable: {self._projection_name}/change_state'
            )

        return ok_yolo and ok_proj

    def _call(self, client, transition_id, callback=None):

        try:
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = client.call_async(req)

            if callback:
                future.add_done_callback(callback)

            return future
        except Exception as e:
            self.node.get_logger().error(f"Failed to call, {e}")

    def _check_future_success(self, future) -> bool:
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(
                f'[PERCEPTION] failed with exception: {e}'
            )
            return False

        if result is None:
            self.node.get_logger().error(
                f'[PERCEPTION] returned no result'
            )
            return False

        if not result.success:
            self.node.get_logger().error(
                f'[PERCEPTION] failed (success=False)'
            )
            return False

        self.node.get_logger().info(
            f'[PERCEPTION] success'
        )
        return True

    def _set_error(self, message: str):
        self.node.get_logger().error(f'[PERCEPTION] {message}')
        self._state = PerceptionControllerState.ERROR
    
    # ========================================
    # CONFIGURE -> ACTIVATE CHAIN
    # ========================================
    def _on_yolo_configured(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("YOLO configure failed")
            return


        self._yolo_configured = True
        self._try_activate()

    def _on_projection_configured(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("Projection configure failed")
            return
        
        self._projection_configured = True
        self._try_activate()

    def _on_yolo_activated(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("YOLO activate failed")
            return

        self._yolo_activated = True
        self._try_set_active()

    def _on_projection_activated(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("Projection activate failed")
            return

        self._projection_activated = True
        self._try_set_active()

    def _try_activate(self):

        if not (self._yolo_configured and self._projection_configured):
            return

        self.node.get_logger().info("[PERCEPTION] Both configured → activating")

        self._state = PerceptionControllerState.ACTIVATING

        self._call(
            self._yolo_client,
            self.ACTIVATE_ID,
            self._on_yolo_activated
        )

        self._call(
            self._projection_client,
            self.ACTIVATE_ID,
            self._on_projection_activated
        )

    def _try_set_active(self):

        if not (self._yolo_activated and self._projection_activated):
            return

        self._state = PerceptionControllerState.ACTIVE
        self.node.get_logger().info("[PERCEPTION] Subsystem ACTIVE")

    # ========================================
    # ACTIVATE -> CLEAN UP CHAIN
    # ========================================

    def _on_yolo_deactivated(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("YOLO deactivate failed")
            return

        self._yolo_deactivated = True
        self._try_cleanup()

    def _on_projection_deactivated(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("Projection deactivate failed")
            return

        self._projection_deactivated = True
        self._try_cleanup()

    def _on_yolo_cleaned(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("YOLO cleanup failed")
            return

        self._yolo_cleaned = True
        self._try_set_inactive()

    def _on_projection_cleaned(self, future):

        result = future.result()
        if not result.success:
            self.node.get_logger().error("Projection cleanup failed")
            return

        self._projection_cleaned = True
        self._try_set_inactive()

    def _try_cleanup(self):

        if not (self._yolo_deactivated and self._projection_deactivated):
            return

        self.node.get_logger().info("[PERCEPTION] Both deactivated → cleanup")

        self._state = PerceptionControllerState.CLEANING_UP

        self._call(
            self._yolo_client,
            self.CLEANUP_ID,
            self._on_yolo_cleaned
        )

        self._call(
            self._projection_client,
            self.CLEANUP_ID,
            self._on_projection_cleaned
        )
    
    def _try_set_inactive(self):

        if not (self._yolo_cleaned and self._projection_cleaned):
            return

        self._state = PerceptionControllerState.INACTIVE
        self.node.get_logger().info("[PERCEPTION] Subsystem INACTIVE")