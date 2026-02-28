import rclpy
from rclpy.node import Node

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