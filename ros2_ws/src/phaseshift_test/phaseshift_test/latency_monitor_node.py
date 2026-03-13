#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

from rclpy.time import Time


class LatencyMonitor(Node):

    def __init__(self):

        super().__init__("latency_monitor_node")

        self.get_logger().info("Latency Monitor Started")

        # --------------------------------
        # Timestamp storage
        # --------------------------------

        self.sensor_time = None
        self.costmap_time = None
        self.plan_time = None

        # --------------------------------
        # Subscriptions
        # --------------------------------

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.costmap_callback,
            10
        )

        self.create_subscription(
            Path,
            "/plan",
            self.plan_callback,
            10
        )

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

    # --------------------------------
    # Sensor callback
    # --------------------------------

    def scan_callback(self, msg: LaserScan):

        self.sensor_time = Time.from_msg(msg.header.stamp)

    # --------------------------------
    # Costmap callback
    # --------------------------------

    def costmap_callback(self, msg: OccupancyGrid):

        self.costmap_time = self.get_clock().now()

        if self.sensor_time is None:
            return

        perception_latency = self.costmap_time - self.sensor_time

        self.get_logger().info(
            f"Perception latency: {perception_latency.nanoseconds / 1e6:.2f} ms"
        )

    # --------------------------------
    # Planner callback
    # --------------------------------

    def plan_callback(self, msg: Path):

        self.plan_time = self.get_clock().now()

        if self.costmap_time is None:
            return

        planning_latency = self.plan_time - self.costmap_time

        self.get_logger().info(
            f"Planning latency: {planning_latency.nanoseconds / 1e6:.2f} ms"
        )

    # --------------------------------
    # Control callback
    # --------------------------------

    def cmd_callback(self, msg: Twist):

        if self.sensor_time is None:
            return

        now = self.get_clock().now()

        total_latency = now - self.sensor_time

        self.get_logger().info(
            f"Sensor → Control latency: {total_latency.nanoseconds / 1e6:.2f} ms"
        )


# --------------------------------
# main
# --------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = LatencyMonitor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()