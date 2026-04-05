from .core import PipelineStep

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ======================================================
# NAV2 STEPS
# ======================================================

class LocalizationReadyStep(PipelineStep):
    def __init__(self, node, nav2):
        super().__init__(node)
        self.nav2 = nav2

    def run(self):
        return self.nav2.is_localization_ready()


class LoadMapStep(PipelineStep):
    def __init__(self, node, nav2, map_manager, ctx, system_phase_enum):
        super().__init__(node)
        self.nav2 = nav2
        self.map_manager = map_manager
        self.ctx = ctx
        self.SystemPhase = system_phase_enum

    def on_enter(self):

        if self.nav2.is_map_loaded():
            return

        if self.ctx._map_yaml is None:
            if self.ctx.use_example_map:
                self.ctx._map_yaml = self.map_manager.resolve_2d_map(
                    self.ctx.example_map_name,
                    True
                )
            else:
                self.ctx._map_yaml = self.map_manager.resolve_2d_map(
                    self.ctx.project_name
                )

            if self.ctx._map_yaml is None:
                self.node.get_logger().error("[PIPELINE] No runtime map found")
                self.ctx.set_phase(self.SystemPhase.ERROR)
                return

        self.node.get_logger().info(
            f"[PIPELINE] Loading map: {self.ctx._map_yaml}"
        )

        self.nav2.load_map(self.ctx._map_yaml)

    def run(self):
        if self.ctx.phase == self.SystemPhase.ERROR:
            return False

        return self.nav2.is_map_loaded()


# ======================================================
# MAP TOPIC WAIT (FIXED)
# ======================================================

class WaitForMapTopicStep(PipelineStep):

    def __init__(self, node):
        super().__init__(node)
        self._received = False
        self.sub = None

    def on_enter(self):

        self._received = False

        self.node.get_logger().info("[PIPELINE] Waiting for /map topic")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self._callback,
            qos
        )

    def _callback(self, msg):
        self._received = True

    def run(self):
        return self._received

    def reset(self):
        super().reset()
        self._received = False


# ======================================================
# INITIAL POSE
# ======================================================

class InitialPoseStep(PipelineStep):

    def __init__(self, node, ctx):
        super().__init__(node)
        self.ctx = ctx

    def on_enter(self):

        if self.ctx.initial_pose_sent:
            return

        self.node.get_logger().info("[PIPELINE] Publishing initial pose")

        self.ctx.publish_initial_pose()
        self.ctx.initial_pose_sent = True

    def run(self):
        return self.ctx.initial_pose_sent


# ======================================================
# NAV2 READY
# ======================================================

class Nav2ReadyStep(PipelineStep):

    def __init__(self, node, nav2):
        super().__init__(node)
        self.nav2 = nav2

    def run(self):
        ready = self.nav2.is_navigation_ready()
        self.node.get_logger().info(f"[PIPELINE] Nav2 ready = {ready}")
        return ready


# ======================================================
# PERCEPTION (FIXED)
# ======================================================

class PerceptionStep(PipelineStep):

    def __init__(self, node, perception):
        super().__init__(node)
        self.perception = perception

    def on_enter(self):
        self.node.get_logger().info("[PIPELINE] Activating perception")
        self.perception.activate()

    def run(self):
        return self.perception.is_active()


# ======================================================
# BEHAVIOUR
# ======================================================

class BehaviourStep(PipelineStep):

    def __init__(self, node, ctx):
        super().__init__(node)
        self.ctx = ctx

    def on_enter(self):
        self.node.get_logger().info("[PIPELINE] Activating behaviour node")
        self.ctx._activate_behaviour_node()

    def run(self):
        return self.ctx._behaviour_activated


# ======================================================
# VOXEL
# ======================================================

class VoxelLayerStep(PipelineStep):

    def __init__(self, node, ctx):
        super().__init__(node)
        self.ctx = ctx

    def on_enter(self):
        self.node.get_logger().info("[PIPELINE] Activating voxel layer")

        voxel_map_path = self.ctx.map_manager.resolve_3d_map(
            self.ctx.project_name
        )

        self.ctx._activate_voxel_costmap_node(voxel_map_path)
        self.ctx._activate_change_detection_node(voxel_map_path)

    def run(self):
        return (
            self.ctx._voxel_costmap_activated and
            self.ctx._voxel_change_detection_activated
        )