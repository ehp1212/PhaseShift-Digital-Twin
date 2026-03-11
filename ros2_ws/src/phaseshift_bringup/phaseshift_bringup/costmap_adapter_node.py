#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from phaseshift_interfaces.msg import CostmapGrid

class CostmapAdapterNode(Node):

    def __init__(self):
        super().__init__("costmap_adapter_node")

        # subscribe Nav2 costmap
        self._sub = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._costmap_callback,
            10
        )

        # publish Unity friendly message
        self._pub = self.create_publisher(
            CostmapGrid,
            "/system/costmap_grid",
            10
        )

        self.get_logger().info("Costmap adapter node started")

    def _costmap_callback(self, msg: OccupancyGrid):

        grid = CostmapGrid()

        # header
        grid.header = msg.header

        # grid info
        grid.width = msg.info.width
        grid.height = msg.info.height
        grid.resolution = msg.info.resolution

        # origin
        grid.origin_x = msg.info.origin.position.x
        grid.origin_y = msg.info.origin.position.y

        q = msg.info.origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        grid.origin_yaw = yaw

        # cost data
        grid.data = [(0 if c < 0 else int(c)) for c in msg.data]

        self._pub.publish(grid)


def main(args=None):

    rclpy.init(args=args)

    node = CostmapAdapterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()