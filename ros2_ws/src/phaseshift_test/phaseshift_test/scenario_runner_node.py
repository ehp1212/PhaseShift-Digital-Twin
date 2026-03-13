#!/usr/bin/env python3

import yaml
import os
import math

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from phaseshift_interfaces.srv import SetGoal
from phaseshift_interfaces.msg import SystemState


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__("scenario_runner_node")

        self.get_logger().info("ScenarioRunner starting")

        # -------------------------
        # Service Client
        # -------------------------

        self.set_goal_client = self.create_client(
            SetGoal,
            "/system/set_goal"
        )

        self._system_client = self.create_subscription(
            SystemState,
            'system_state',
            self._system_callback,
            10
        )

        while not self.set_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /system/set_goal service...")

        # -------------------------
        # Scenario
        # -------------------------

        scenario = self.load_scenario()

        self.frame_id = scenario["frame_id"]
        self.waypoints = scenario["waypoints"]
        self.route = scenario["route"]
        self.loops = scenario["loops"]

        self.get_logger().info(f"Loaded scenario: {scenario['name']}")

        # -------------------------
        # Runtime State
        # -------------------------

        self.loop_index = 0
        self.route_index = 0

        self.goal_future = None
        self.goal_state = "IDLE"
        self.system_phase = None

        # -------------------------
        # Timer FSM
        # -------------------------

        self.timer = self.create_timer(
            0.2,
            self.update
        )


    # --------------------------------------------------
    # System 
    # --------------------------------------------------

    def _system_callback(self, msg: SystemState):

        self.system_phase = msg.phase

    # --------------------------------------------------
    # Load YAML
    # --------------------------------------------------

    def load_scenario(self):

        pkg_path = get_package_share_directory("phaseshift_test")

        scenario_path = os.path.join(
            pkg_path,
            "config",
            "scenario.yaml"
        )

        with open(scenario_path, "r") as f:
            data = yaml.safe_load(f)

        return data["scenario"]

    # --------------------------------------------------
    # Main FSM
    # --------------------------------------------------

    def update(self):

        # ------------------------------
        # Nav executing guard
        # ------------------------------
        if self.system_phase == SystemState.PHASE_NAV_EXECUTING:
            return

        if self.goal_state == "IDLE":

            if self.loop_index >= self.loops:
                return

            waypoint_name = self.route[self.route_index]
            wp = self.waypoints[waypoint_name]

            self.send_goal(wp)
            return
        
        # ------------------------------
        # WAITING_RESPONSE
        # ------------------------------

        elif self.goal_state == "WAITING_RESPONSE":
            
            if not self.goal_future.done():
                return

            result = self.goal_future.result()

            if not result.success:
                self.get_logger().error("Goal rejected by orchestrator")
                self.goal_state = "IDLE"
                return

            self.get_logger().info("Goal accepted")
            self.advance_route()

            self.goal_state = "IDLE"
            
    # --------------------------------------------------
    # Send Goal
    # --------------------------------------------------

    def send_goal(self, wp):

        req = SetGoal.Request()

        req.x = wp["x"]
        req.y = wp["y"]
        req.yaw = wp["yaw"]

        self.goal_future = self.set_goal_client.call_async(req)
        self.goal_state = "WAITING_RESPONSE"

    # --------------------------------------------------
    # Route Progression
    # --------------------------------------------------

    def advance_route(self):

        self.route_index += 1

        if self.route_index >= len(self.route):

            self.route_index = 0
            self.loop_index += 1

            self.get_logger().info(
                f"Loop finished → {self.loop_index}/{self.loops}"
            )


# --------------------------------------------------
# Main
# --------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = ScenarioRunner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()