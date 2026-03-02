#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


@dataclass
class VelCmd:
    v: float = 0.0      # m/s
    w: float = 0.0      # rad/s
    stamp: Time | None = None


def yaw_to_quat(yaw: float):
    # planar yaw -> quaternion (x=y=0)
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class DiffDriveIntegratorNode(LifecycleNode):
    """
    /cmd_vel (Twist) -> integrate planar pose -> publish /odom + TF(odom->base_footprint)

    - ROS2 authority: odom/TF are generated here.
    - Unity: publishes only /cmd_vel, and consumes TF for visualization.
    """

    def __init__(self):
        super().__init__('diff_drive_integrator')

        # Parameters (safe defaults)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate_hz', 50.0)   # integration loop rate
        self.declare_parameter('cmd_timeout_sec', 0.25)   # if cmd_vel is stale, stop
        self.declare_parameter('publish_tf', True)

        # Internal state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._last_update_time: Time | None = None
        self._last_cmd = VelCmd()

        # ROS entities (created during configure/activate)
        self._cmd_sub = None
        self._odom_pub = None
        self._tf_broadcaster = None
        self._timer = None

        self.get_logger().info("DiffDriveIntegratorNode constructed (lifecycle).")

    # -------------------------
    # Lifecycle callbacks
    # -------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")

        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self._odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._cmd_timeout = self.get_parameter('cmd_timeout_sec').get_parameter_value().double_value
        self._publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # QoS: cmd_vel는 최신만 중요 (reliable 추천)
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # QoS: odom은 연속 스트림 (reliable 추천)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publisher/subscriber (lifecycle publisher 아님: 최소 구현)
        self._cmd_sub = self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, cmd_qos)
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, odom_qos)

        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # Reset integrator state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_update_time = None
        self._last_cmd = VelCmd()

        self.get_logger().info(
            f"Configured. frames: {self._odom_frame} -> {self._base_frame}, "
            f"topics: cmd={self._cmd_vel_topic}, odom={self._odom_topic}, "
            f"rate={self._rate}Hz, tf={self._publish_tf}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")

        period = 1.0 / max(self._rate, 1e-6)
        self._timer = self.create_timer(period, self._update_loop)
        self._last_update_time = self.get_clock().now()

        self.get_logger().info("Active: integrating + publishing /odom + TF.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")

        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        # 안전: 비활성화 시 속도 0 처리(상태는 유지하되 명령은 끊음)
        self._last_cmd = VelCmd()

        self.get_logger().info("Inactive: stopped update loop.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")

        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        if self._cmd_sub is not None:
            self.destroy_subscription(self._cmd_sub)
            self._cmd_sub = None

        if self._odom_pub is not None:
            self.destroy_publisher(self._odom_pub)
            self._odom_pub = None

        self._tf_broadcaster = None

        self.get_logger().info("Cleaned.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    # -------------------------
    # Runtime
    # -------------------------
    def _on_cmd_vel(self, msg: Twist):
        now = self.get_clock().now()
        self._last_cmd = VelCmd(v=float(msg.linear.x), w=float(msg.angular.z), stamp=now)

    def _update_loop(self):
        now = self.get_clock().now()
        if self._last_update_time is None:
            self._last_update_time = now
            return

        dt = (now - self._last_update_time).nanoseconds * 1e-9
        self._last_update_time = now
        if dt <= 0.0:
            return

        # cmd timeout -> stop
        v = self._last_cmd.v
        w = self._last_cmd.w
        if self._last_cmd.stamp is None:
            v, w = 0.0, 0.0
        else:
            age = (now - self._last_cmd.stamp).nanoseconds * 1e-9
            if age > self._cmd_timeout:
                v, w = 0.0, 0.0

        # Integrate planar motion
        # x += v * cos(yaw) * dt
        # y += v * sin(yaw) * dt
        # yaw += w * dt
        self._yaw += w * dt
        # normalize yaw to [-pi, pi] optional
        self._yaw = (self._yaw + math.pi) % (2.0 * math.pi) - math.pi

        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self._yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self._odom_pub.publish(odom)

        # Broadcast TF: odom -> base_footprint
        if self._publish_tf and self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveIntegratorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()