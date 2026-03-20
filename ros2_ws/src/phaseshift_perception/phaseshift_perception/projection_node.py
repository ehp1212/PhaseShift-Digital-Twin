import math
import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformException

from phaseshift_interfaces.msg import DetectedObjectArray, DetectedObject


class ProjectionNode(LifecycleNode):
    def __init__(self):
        super().__init__('projection_node')

        self._bridge = CvBridge()

        self._sub_det = None
        self._sub_depth = None
        self._sub_info = None
        self._pub_objects = None

        self._latest_depth = None
        self._latest_depth_header = None
        self._camera_info = None

        self._target_frame = 'map'
        self._camera_frame = 'camera_optical_frame'

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self
        )

        self.get_logger().info('[Project] Lifecycle Node Created...')

    # -----------------------------
    # CONFIGURE
    # -----------------------------
    def on_configure(self, state: State):
    
        self.get_logger().info("Configuring Perception nodes...")

        try:
            qos_sensor = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )

            qos_output = QoSProfile(
                reliability= ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            self._sub_det = self.create_subscription(
                Detection2DArray,
                '/perception/detections_2d',
                self._detection_callback,
                qos_output
            )

            self._sub_depth = self.create_subscription(
                Image,
                '/camera/depth/image',
                self._depth_callback,
                qos_sensor
            )

            self._sub_info = self.create_subscription(
                CameraInfo,
                '/camera/depth/camera_info',
                self._info_callback,
                qos_sensor
            )

            self._pub_objects = self.create_publisher(
                DetectedObjectArray,
                '/perception/objects_3d',
                qos_output
            )

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configure failed: {e}")
            return TransitionCallbackReturn.FAILURE

    # -----------------------------
    # ACTIVATE
    # -----------------------------
    def on_activate(self, state):
        self.get_logger().info("Activating Perception nodes...")
        return super().on_activate(state)    
    
    # -----------------------------
    # DEACTIVATE
    # -----------------------------
    def on_deactivate(self, state):
        self.get_logger().info("Deactivating Perception nodes...")
        return super().on_deactivate(state)
    
    # -----------------------------
    # CLEANUP
    # -----------------------------
    def on_cleanup(self, state):
        try:
            if self._sub_det:
                self.destroy_subscription(self._sub_det)
                self._sub_det = None

            if self._sub_depth:
                self.destroy_subscription(self._sub_depth)
                self._sub_depth = None

            if self._sub_info:
                self.destroy_subscription(self._sub_info)
                self._sub_info = None

            if self._pub_objects:
                self.destroy_publisher(self._pub_objects)
                self._pub_objects = None
                
            self._latest_depth = None
            self._latest_depth_header = None
            self._camera_info = None
            
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configure failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    # -----------------------------
    # Depth Callbacks
    # -----------------------------
    def _depth_callback(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self._latest_depth = depth
            self._latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f'[Projection] Depth convert error: {e}')

    # -----------------------------
    # Camera Info Callbacks
    # -----------------------------
    def _info_callback(self, msg):
        self._camera_info = msg
        if msg.header.frame_id:
            self._camera_frame=msg.header.frame_id

    # -----------------------------
    # Depth Callbacks
    # -----------------------------
    def _detection_callback(self, msg):

        if self._latest_depth is None or self._camera_info is None:
            return
        
        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]

        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn('[Projection] Invalid camera intrinsics')
            return

        depth_img = self._latest_depth
        height, width = depth_img.shape[:2]

        out = Detection2DArray()
        out.header = msg.header
        out.header.frame_id = self._target_frame

        for det in msg.detections:
            if len(det.results) == 0:
                continue

            u = int(round(det.bbox.center.position.x))
            v = int(round(det.bbox.center.position.y))

            if not self._is_valid_pixel(u, v, width, height):
                continue
            
            z = self._sample_depth(depth_img, u, v)
            if z is None:
                continue

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            point_cam = PointStamped()
            point_cam.header.stamp = msg.header.stamp
            point_cam.header.frame_id = self._camera_frame
            point_cam.point.x = float(x)
            point_cam.point.y = float(y)
            point_cam.point.z = float(z)

            try:
                point_map = self._tf_buffer.transform(
                    point_cam,
                    self._target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except TransformException as e:
                self.get_logger().warn(
                    f'[Projection] TF transform failed: {e}',
                    throttle_duration_sec=2.0
                )
                continue

            obj = DetectedObject()
            obj.class_id = det.results[0].hypothesis.class_id
            obj.confidence = float(det.results[0].hypothesis.score)
            obj.pose.position.x = point_map.point.x
            obj.pose.position.y = point_map.point.y
            obj.pose.position.z = point_map.point.z
            obj.pose.orientation.w = 1.0

            out.objects.append(obj)

        if len(out.objects) > 0:
            self._pub_objects.publish(out)

