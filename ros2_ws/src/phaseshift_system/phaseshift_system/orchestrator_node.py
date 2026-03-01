import rclpy
from rclpy.node import Node

# phaseshift_system
#     └── orchestrator_node.py
#             ├── SlamController (from control pkg)
#             └── Nav2Controller (from control pkg)

# phaseshift_control
#     ├── lifecycle_client_base.py
#     ├── slam_controller.py
#     ├── nav2_controller.py

class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator')
        self.get_logger().info("Phaseshift Orchestrator Started")


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()