import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

class VoxelCostmapNode(LifecycleNode):

    def __init__(self):
        super().__init__('voxel_costmap_node')
        self._log_info_label('VOXEL COSTMAP NODE', "started...")
        
        # -------------------------
        # VOXEL MAP
        # -------------------------
        self.declare_parameter('voxel_map_path', '')
        self._voxel_map_path = None
        self.map_loaded = False

    # ==============================
    # LIFECYCLE
    # ==============================
    def on_configure(self, state):

        self._log_info_label('VOXEL COSTMAP NODE', "configuring node...")
        self.voxel_map_path = self.get_parameter('voxel_map_path').value

        if not self.voxel_map_path:
            self._log_info_label('VOXEL COSTMAP NODE', "No voxel_map_path provided...")
            return TransitionCallbackReturn.FAILURE

        self._log_info_label('VOXEL COSTMAP NODE', f"{self.voxel_map_path}")

        success = self._load_voxel_map(self.voxel_map_path)

        if not success:
            self._log_info_label('VOXEL COSTMAP NODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE

        self.map_loaded = True
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state):

        self._log_info_label('VOXEL COSTMAP NODE', "activating node...")

        if not self.map_loaded:
            self._log_info_label('VOXEL COSTMAP NODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE
        self._log_info_label('VOXEL COSTMAP NODE', "activated successfully")

        # publisher 활성화
        self._start_publishing()

        self._log_info_label('VOXEL COSTMAP NODE', "activated successfully")
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        return super().on_deactivate(state)
    
    def on_cleanup(self, state):
        return super().on_cleanup(state)

    # ==============================
    # PUBLIC API
    # ==============================
    def set_voxel_map_path(self, map_path: str):
        if not map_path:
            self.get_logger().error("Map path is invalid.")
            return
        
        self._voxel_map_path = map_path
    
    # ==============================
    # INTERNAL
    # ==============================

    def _load_voxel_map(self, path):

        try:
            # TODO: Open3D / PCL load
            self._log_info_label("VOXEL COSTMAP NODE", "PCD file loaded successfully")

            return True
        except Exception as e:
            self.get_logger().error(f"Load failed: {e}")
            return False
        
    def _start_publishing(self):
        pass
        # self.timer = self.create_timer(
        #     1.0,
        #     self.publish_costmap
        # )
        
    # ==============================
    # UTILS
    # ==============================

    def _log_info_label(self, label: str, msg: str):
        self.get_logger().info(f"\033[93m[{label}]\033[0m - {msg}")  # Green


def main():
    rclpy.init()
    node = VoxelCostmapNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try: 
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        try:
            node._cleanup()
        except Exception as e:
            node.get_logger().warn(f"Lifecycle shutdown failed: {e}")

        executor.shutdown()
        node.destroy_node()