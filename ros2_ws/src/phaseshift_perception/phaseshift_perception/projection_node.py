import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose

import numpy as np
import cv2
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs

class ProjectionNode(LifecycleNode):
    def __init__(self):
        super().__init__('projection_node')

        self._bridge = CvBridge()
        self._depth = None
        self._camera_info = None

        self._tf_buffer = tf2_ros.buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self
        )

    # -----------------------------
    # CONFIGURE
    # -----------------------------
    def on_configure(self, state: State):
    
        self.get_logger().info("Configuring Perception nodes...")

        try:
            self._sub_det = self.create_subscription(
                Detection2DArray,
                '/perception/detections_2ds',
                self._detection_callback,
                10
            )

            self._sub_depth = self.create_subscription(
                Image,
                '/depth/camera/image',
                self._depth_callback,
                10
            )

            self._sub_info = self.create_subscription(
                CameraInfo,
                '/camera/camera_info',
                10
            )

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configure failed: {e}")
            return TransitionCallbackReturn.FAILURE
        
    # -----------------------------
    # Callbacks
    # -----------------------------
    def _depth_callback(self, msg):

        self._depth = self._bridge.imgmsg_to_cv2(msg)

    def _info_callback(self, msg):

        self._camera_info = msg

    def _detection_callback(self, msg):

        if self._depth is None or self._camera_info is None:
            return
        
        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]

        height, width = self._depth.shape

        for det in msg.detections:
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            # -----------------------------
            # safety check
            # -----------------------------
            if u < 0 or v < 0 or u >= width or v >= height:
                continue
            Z = self._depth[v, u]

            # invalid depth skip
            if Z == 0:
                continue

            # ⚠️ depth 단위 확인 필요 (mm → m)
            if Z > 10:  # heuristic
                Z = Z / 1000.0

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            self.get_logger().info(
                f"[Projection] 3D -> X:{X:.2f}, Y:{Y:.2f}, Z:{Z:.2f}"
            )


