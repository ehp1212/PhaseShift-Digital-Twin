import math
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

"""
TODO: Diff drive
Manual Driving 기준:

configure → publisher/subscription 생성
activate → timer 시작
deactivate → timer 정지 + velocity reset
cleanup → 리소스 삭제
"""

class OdometryLifecycleNode(LifecycleNode):

    def __init__(self):
        super().__init__('odometry_node')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.wz = 0.0

        self.last_time = None
        self.last_cmd_time = None

        self.timer = None

    # ==============================
    # CONFIGURE
    # ==============================
    def on_configure(self, state: State):
        self.get_logger().info("Configuring Odometry Node...")

        result = super().on_configure(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_lifecycle_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # self.timer = self.create_timer(0.02, self.update)
        # self.timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    # ==============================
    # ACTIVATE
    # ==============================
    def on_activate(self, state):
        self.get_logger().info("Activating Odometry Node...")

        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result

        # self.timer.reset()
        self.timer = self.create_timer(0.02, self.update)
        return TransitionCallbackReturn.SUCCESS

    # ==============================
    # DEACTIVATE
    # ==============================
    def on_deactivate(self, state):
        self.get_logger().info("Deactivating Odometry Node...")
        self.timer.cancel()

        self.vx = 0.0
        self.wz = 0.0

        return TransitionCallbackReturn.SUCCESS

    # ==============================
    # CLEANUP
    # ==============================
    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up Odometry Node...")

        if self.timer:
            self.destroy_timer(self.timer)

        self.destroy_publisher(self.odom_pub)
        self.destroy_subscription(self.cmd_sub)

        return TransitionCallbackReturn.SUCCESS


    # ==============================
    # CMD CALLBACK
    # ==============================
    def cmd_callback(self, msg: Twist):
        
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    # ==============================
    # UPDATE LOOP
    # ==============================        
    def update(self):
        if not self.odom_pub.is_activated:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Timeout safety (500ms)
        if (current_time - self.last_cmd_time).nanoseconds > 5e8:
            self.vx = 0.0
            self.wz = 0.0

        # Integrate
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.wz * dt

        # Quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # ================= TF =================
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # ================= ODOM MSG =================
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OdometryLifecycleNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()