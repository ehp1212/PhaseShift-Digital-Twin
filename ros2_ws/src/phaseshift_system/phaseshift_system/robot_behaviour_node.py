import math
import rclpy

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from enum import Enum

from geometry_msgs.msg import PoseStamped
from phaseshift_interfaces.msg import NavState, DetectedObjectArray
from phaseshift_interfaces.srv import InternalRequestNavigate, InternalCancelNavigate, InternalRequestRecovery
from .waypoint_buffer import WaypointBuffer

class BehaviourState(Enum):
    IDLE = 0
    MOVING = 1
    RECOVERING = 2
    RETURNING = 3
    REPLANNING = 4 

class RobotBehaviourNode(LifecycleNode):
    def __init__(self):
        super().__init__('robot_behaviour_node')

        # NAV STATE SUB
        self._nav_state_sub = None

        # NAVIGATE CLIENT
        self._nav_request_client = None
        self._nav_cancel_client = None
        self._nav_recovery_client = None

        self._perception_sub = None

        self._prev_nav_status = None
        self._latest_nav_state = None
        self._timer = None

        self._state_enter_time = None
        self._recovery_attempts = 0

        self._primary_goal = None
        self._active_goal = None

        self._last_detected_obstacle = None

        self._last_waypoint_time = None
        self._waypoint_buffer = WaypointBuffer(
            max_size=50,
            min_distance=0.5
        )
        
        self._state = BehaviourState.IDLE
    
    # -----------------------------
    # LIFECYCLE
    # -----------------------------
    def on_configure(self, state: State):
        self.get_logger().info("Configuring Robot Behaviour Node...")
        try:
            nav_state_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

            self._nav_state_sub = self.create_subscription(
                NavState,
                '/system/internal/nav_state',
                self._nav_state_callback,
                nav_state_qos
            )

            self._nav_request_client = self.create_client(
                InternalRequestNavigate,
                '/system/internal/request_navigate'
            )

            self._nav_recovery_client = self.create_client(
                InternalRequestRecovery,
                '/system/internal/request_recovery'
            )

            self._nav_cancel_client = self.create_client(
                InternalCancelNavigate,
                '/system/internal/cancel_navigate'
            )

            # Perception
            self._perception_sub = self.create_subscription(
                DetectedObjectArray,
                '/perception/objects',
                self._perception_callback,
                10
            )

            # Declare parameters
            self.declare_parameter("max_recovery_attempts", 2)
            self.declare_parameter("recovery_distance_threshold", 1.0)
            self.declare_parameter("recovery_timeout_sec", 5.0)

            self._max_recovery_attempts = self.get_parameter(
                "max_recovery_attempts"
            ).value

            self._recovery_distance_threshold = self.get_parameter(
                "recovery_distance_threshold"
            ).value

            self._recovery_timeout_sec = self.get_parameter(
                "recovery_timeout_sec"
            ).value

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State):
        self.get_logger().info("Activating Robot Behaviour Node...")
        try:
            available = (
                self._nav_request_client.wait_for_service(timeout_sec=1.0) and
                self._nav_cancel_client.wait_for_service(timeout_sec=1.0) and
                self._nav_recovery_client.wait_for_service(timeout_sec=1.0)
            )

            # if not available:
            #     self.get_logger().info(f"******************************")
            #     self.get_logger().info(f"******************************")
            #     self.get_logger().warn("Nav request service not available")
            #     return TransitionCallbackReturn.FAILURE

            self._timer = self.create_timer(0.1, self._update)
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"[BEHAVIOUR] Failed to activate - {str(e)}")
            return TransitionCallbackReturn.FAILURE
        
        finally:
            return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("Deactivating Robot Behaviour Node...")
        try:
            self._latest_nav_state = None
            self._prev_nav_status = None
            self._state = BehaviourState.IDLE
            self._state_enter_time = None
            self._recovery_attempts = 0
            self._last_detected_obstacle = None

            if self._timer:
                self._timer.cancel()

            self._timer = None

        except Exception as e:
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State):
        self.get_logger().info("Cleaning up Robot Behaviour Node...")
        try:
            self._nav_state_sub = None
            self._perception_sub = None
            self._nav_request_client = None
            self._nav_cancel_client = None
            self._nav_recovery_client = None
            self._timer = None
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE
    
    # ==================================================
    # FSM UPDATE
    # ==================================================
    def _update(self):
        if self._latest_nav_state is None:
            return 

        msg = self._latest_nav_state
        if self._primary_goal is None and msg.has_goal:
            self._primary_goal = msg.current_goal

        if msg.status == NavState.MOVING and self._latest_nav_state.has_goal: 

            now = self.get_clock().now().nanoseconds / 1e9
            if self._last_waypoint_time is None or (now - self._last_waypoint_time) > 3.0:
                self._waypoint_buffer.add(msg.current_pose)
                self._last_waypoint_time = now
                self.get_logger().info(
                    f"Added. [X : {msg.current_pose.pose.position.x}] "
                    f"[Y : {msg.current_pose.pose.position.y}] "
                    f"[Z : {msg.current_pose.pose.position.z}]"
                )

        nav_status_changed = (msg.status != self._prev_nav_status)

        self._run_fsm(msg, nav_status_changed)
        self._prev_nav_status = msg.status
    
    def _run_fsm(self, msg: NavState, nav_status_changed: bool):
        if self._state == BehaviourState.IDLE:
            if nav_status_changed and msg.status == NavState.MOVING and msg.has_goal:
                self._transition(BehaviourState.MOVING)

        elif self._state == BehaviourState.MOVING:
            if nav_status_changed and msg.status == NavState.BLOCKED:
                self._handle_block(msg)

            elif nav_status_changed and msg.status == NavState.SUCCEEDED:
                self._handle_goal_reached()

        elif self._state == BehaviourState.RECOVERING:
          
            elapsed = (self.get_clock().now() - self._state_enter_time).nanoseconds / 1e9

            if elapsed < 1.0:
                return

            if nav_status_changed and msg.status == NavState.MOVING:
                self._handle_recovery_succeeded()

            elif nav_status_changed and msg.status == NavState.FAILED:
                self._handle_recovery_failed()

            elif msg.status == NavState.RECOVERING:
                self._check_recovery_timeout()

        elif self._state == BehaviourState.RETURNING:
            if nav_status_changed and msg.status == NavState.SUCCEEDED:
                self.get_logger().info("Returned to waypoint → resume navigation")
                self._transition(BehaviourState.REPLANNING)

            elif nav_status_changed and msg.status == NavState.FAILED:
                self.get_logger().warn("Returning failed → abort")
                self._transition(BehaviourState.IDLE)

        elif self._state == BehaviourState.REPLANNING:
            self._handle_replanning()

    # ==================================================
    # SUB CALLBACK
    # ==================================================
    def _nav_state_callback(self, msg: NavState):
        self._latest_nav_state = msg

    def _perception_callback(self, msg: DetectedObjectArray):
        if self._latest_nav_state is None:
            return
        
        if not msg.objects:
            self._last_detected_obstacle = None
            return

        # Get closest obstacle
        closest = min(
            msg.objects,
            key=lambda obj: self._distance_to_robot(obj)
        )

        self._last_detected_obstacle = closest

    # ==================================================
    # NAV2 STATE BEHAVIOUR  
    # ==================================================
    def _handle_block(self, msg):
        if (
            self._recovery_attempts < self._max_recovery_attempts
            and msg.distance_to_goal < self._recovery_distance_threshold
        ):
            self.get_logger().info("Tryijng to recover")

            self._send_nav_recovery_request()
            self._recovery_attempts += 1
            self._transition(BehaviourState.RECOVERING)
        else:
            self._start_returning()

    def _handle_goal_reached(self):
        self.get_logger().info("Goal reached")

        self._waypoint_buffer.clear() 
        self._recovery_attempts = 0
        self._primary_goal = None
        self._active_goal = None
        self._last_detected_obstacle = None
        self._transition(BehaviourState.IDLE)

    def _handle_recovery_succeeded(self):
        self.get_logger().info("Recovery succeeded")
        self._transition(BehaviourState.MOVING)

    def _handle_recovery_failed(self):
        self.get_logger().warn("Recovery failed")
        self._start_returning()

    def _handle_replanning(self):

        if self._last_detected_obstacle:
            new_goal = self._compute_avoidance_goal(self._last_detected_obstacle)
            if new_goal is None:
                self._transition(BehaviourState.IDLE)
                return

            self._send_nav_navigate_request(new_goal, is_primary_goal=False)

        elif self._primary_goal is not None:
            self._send_nav_navigate_request(self._primary_goal, is_primary_goal=False)

        else:
            self.get_logger().error("No primary goal available for replanning")
            self._transition(BehaviourState.IDLE)
            return

        self._transition(BehaviourState.MOVING)

    def _start_returning(self):
        self.get_logger().warn("Start returning to fallback waypoint")

        prev_goal = self._waypoint_buffer.get_previous()

        if prev_goal is None:
            self.get_logger().error("No waypoint available → abort")
            self._transition(BehaviourState.IDLE)
            return

        self.get_logger().info("Retrunning to previous waypoint")
        self._send_nav_navigate_request(prev_goal, is_primary_goal=False)
        self._transition(BehaviourState.RETURNING)

    def _check_recovery_timeout(self):
        if self._state_enter_time is None:
            return

        current_time = self.get_clock().now()
        time_in_recover = (current_time - self._state_enter_time).nanoseconds / 1e9

        if time_in_recover > self._recovery_timeout_sec:
            self._handle_recovery_timeout()

    def _handle_recovery_timeout(self):
        self.get_logger().warn("Recovery timeout")
        self._start_returning()

    # ==================================================
    # NAV2 SERVICE  
    # ==================================================
    def _send_nav_navigate_request(self, goal: PoseStamped, *, is_primary_goal: bool = False):      
        
        req = InternalRequestNavigate.Request()
        req.goal = goal

        self._active_goal = goal
        if is_primary_goal:
            self._primary_goal = goal

        future = self._nav_request_client.call_async(req)
        future.add_done_callback(self._on_nav_response)

    def _send_nav_cancel_request(self):
        req = InternalCancelNavigate.Request()
        future = self._nav_cancel_client.call_async(req)
        future.add_done_callback(self._on_nav_cancel_response)

    def _send_nav_recovery_request(self):

        self.get_logger().info("Send recovery request......")

        req = InternalRequestRecovery.Request()
        future = self._nav_recovery_client.call_async(req)
        future.add_done_callback(self._on_nav_recovery_response)

    # ==================================================
    # NAV2 SERVICE CALLBACK  
    # ==================================================

    def _on_nav_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Navigate response: {res.accepted}")
        except Exception as e:
            self.get_logger().error(str(e))

    def _on_nav_cancel_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Navigate Cancel response: {res.accepted}")
        except Exception as e:
            self.get_logger().error(str(e))

    def _on_nav_recovery_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Navigate Recovery response: {res.accepted}")
        except Exception as e:
            self.get_logger().error(str(e))

    # ==================================================
    # UTILS  
    # ==================================================
    def _transition(self, new_state: BehaviourState):
        if self._state == new_state:
            return

        self._log_state("BEHAVIOUR NODE", f"{self._state.name} → {new_state.name}")
        self._state = new_state
        self._state_enter_time = self.get_clock().now()

    def _compute_avoidance_goal(self, obstacle):

        robot_pose = self._latest_nav_state.current_pose

        rx = robot_pose.pose.position.x
        ry = robot_pose.pose.position.y

        ox = obstacle.pose.position.x
        oy = obstacle.pose.position.y

        dx = ox - rx
        dy = oy - ry

        length = math.sqrt(dx * dx + dy * dy)

        if self._primary_goal is None:
            self.get_logger().error("No primary goal for avoidance fallback")
            return None

        dx /= length
        dy /= length

        perp_left = (-dy, dx)
        perp_right = (dy, -dx)

        avoidance_distance = 1.0

        # Left only for now
        px, py = perp_left

        new_x = rx + px * avoidance_distance
        new_y = ry + py * avoidance_distance

        self.get_logger().info(
            f"Avoidance goal: ({new_x:.2f}, {new_y:.2f})"
        )

        return self._make_goal(new_x, new_y, robot_pose)
    
    def _make_goal(self, x, y, reference_pose):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = reference_pose.pose.orientation

        return goal
    
    def _distance_to_robot(self, obj):
        if self._latest_nav_state is None:
            return float("inf")

        robot_pose = self._latest_nav_state.current_pose

        dx = obj.pose.position.x - robot_pose.pose.position.x
        dy = obj.pose.position.y - robot_pose.pose.position.y

        return (dx**2 + dy**2) ** 0.5
    
    def _log_state(self, state: str, msg: str):
        self.get_logger().info(f"\033[93m[{state}]\033[0m - {msg}")  # Green


def main(args=None):
    rclpy.init(args=args)

    node = RobotBehaviourNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()