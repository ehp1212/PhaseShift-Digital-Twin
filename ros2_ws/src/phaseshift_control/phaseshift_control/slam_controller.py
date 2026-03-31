import subprocess
import os
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class SlamController:

    def __init__(self, node: Node):
        self.node = node

        self._slam_process = None
        pkg_share = get_package_share_directory("phaseshift_control")

        self._slam_yaml = os.path.join(
            pkg_share,
            "config", 
            "slam.yaml"
        )

        self._map_received = False
        self.is_map_saved = False
        self._map_sub = node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

        self._save_map_cli = self.node.create_client(
            SaveMap,
            '/slam_toolbox/save_map'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    # ----------------------------------------------
    # Lifecycle
    # ----------------------------------------------
    def start_slam(self):
        if self._slam_process is not None:
            self.node.get_logger().warn("SLAM is already running")
            return
        
        self.node.get_logger().info("[SLAM] starting slam_toolbox")

        self._slam_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "slam_toolbox",
                "sync_slam_toolbox_node",
                "--ros-args",
                "--params-file",
                self._slam_yaml
            ],
            preexec_fn=os.setsid 
        )

        self._map_received = False
        self.is_map_saved = False

    def stop_slam(self):
        if self._slam_process is None:
            self.node.get_logger().warn("SLAM is not running")
            return
        
        self.node.get_logger().info("[SLAM] stopping slam_toolbox")

        try:
            os.killpg(os.getpgid(self._slam_process.pid), signal.SIGINT)

            self._slam_process.wait(timeout=5)

        except Exception as e:
            self.node.get_logger().warn(f"[SLAM] graceful stop failed: {e}")

            try:
                os.killpg(os.getpgid(self._slam_process.pid), signal.SIGKILL)
            except Exception as e:
                self.node.get_logger().error(f"[SLAM] force kill failed: {e}")

        finally:
            self._slam_process = None

    # ----------------------------------------------
    # Readiness
    # ----------------------------------------------

    def is_map_active(self) -> bool:
        return self._map_received

    def is_map_tf_active(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            return True
        except:
            return False
    
    def is_odom_tf_active(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            return True
        except:
            return False

    def is_running(self) -> bool:
        if self._slam_process is None:
            return False
        return self._slam_process.poll() is None

    def is_ready(self) -> bool:
        return (
            self.is_running()
            and self.is_map_active()
            and self.is_map_tf_active()
            and self.is_odom_tf_active()
        )
    
    def map_saved(self) -> bool:
        return self.is_map_saved

    # ----------------------------------------------
    # Map Save
    # ----------------------------------------------

    def save_map_async(self, path: str, on_done= None):

        if not self._save_map_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("SaveMap service not available.")
            self.node.on_map_save_failed()
            return

        request = SaveMap.Request()
        request.name = String()
        request.name.data = path
        self.is_map_saved = False

        future = self._save_map_cli.call_async(request)
        self._check_future_timer = self.node.create_timer(
            0.1,
            # self.check_save_map_status
            lambda: self._check_future(future, on_done)
        )
    
    def _check_future(self, future, on_done):

        if not future.done():
            return

        try:
            future.result()  
            self.node.get_logger().info("SLAM map saved")

            if on_done:
                on_done(success=True)

        except Exception as e:
            self.node.get_logger().error(f"SaveMap failed: {e}")

            if on_done:
                on_done(success=False)

        finally:
            self._check_future_timer.destroy()
            self._check_future_timer = None
    
    # ----------------------------------------------

    def _map_callback(self, msg):

        if self._map_received:
            return

        self._map_received = True
        self.node.get_logger().info("First map received")