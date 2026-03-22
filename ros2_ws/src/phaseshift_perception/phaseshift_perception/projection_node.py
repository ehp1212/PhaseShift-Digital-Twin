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
import tf2_geometry_msgs
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

        out = DetectedObjectArray()
        out.header = msg.header
        out.header.frame_id = self._target_frame

        for det in msg.detections:
            if len(det.results) == 0:
                continue

            u_center = int(round(det.bbox.center.position.x))
            v_center = int(round(det.bbox.center.position.y))

            bbox_w = int(det.bbox.size_x)
            bbox_h = int(det.bbox.size_y)

            if not self._is_valid_pixel(u_center, v_center, width, height):
                continue

            if bbox_w < 2 or bbox_h < 2:
                continue


            roi = self._compute_bottom_roi(
                u_center, v_center, bbox_w, bbox_h, width, height
            )

            z = None

            if roi is not None:
                u_min, u_max, v_min, v_max = roi
                z = self._sample_depth_region(depth_img, u_min, u_max, v_min, v_max)

            # 3) fallback: center depth
            if z is None:
                z = depth_img[v_center, u_center]
                if not self._is_valid_depth(z):
                    continue


            depth_camera_range = 20
            z *= depth_camera_range

            x = (u_center - cx) * z / fx
            y = (v_center - cy) * z / fy

            point_cam = PointStamped()
            point_cam.header.stamp = msg.header.stamp
            point_cam.header.frame_id = self._camera_frame
            point_cam.point.x = float(x)
            point_cam.point.y = float(y)
            point_cam.point.z = float(z)


            # self.get_logger().info(f"u,v: {u},{v}")
            # self.get_logger().info(f"depth: {z}")


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
        
    def _is_valid_pixel(self, u, v, width, height):
        return 0 <= u < width and 0 <= v < height
    
    def _sample_depth(self, depth_img, u, v):
        z = depth_img[v, u]

        if z == 0 or not np.isfinite(z):
            return None

        return float(z)
    
    # def _sample_depth_region(self, depth, u, v, w=5):
    #     h, W = depth.shape[:2]

    #     u0 = max(0, u - w)
    #     u1 = min(W, u + w)
    #     v0 = max(0, v - w)
    #     v1 = min(h, v + w)

    #     patch = depth[v0:v1, u0:u1]

    #     valid = patch[np.isfinite(patch) & (patch > 0)]

    #     if len(valid) == 0:
    #         return None

    #     return float(np.median(valid)) 
    
    def _compute_bottom_roi(self, u_center, v_center, bbox_w, bbox_h, width, height):
        
        bbox_w = max(2, bbox_w)
        bbox_h = max(2, bbox_h)

        roi_width = max(3, int(bbox_w * 0.5))
        roi_height = max(3, int(bbox_h * 0.25))

        u_min = u_center - roi_width // 2
        u_max = u_center + roi_width // 2

        # bbox 하단 쪽에 ROI 배치
        v_min = int(v_center + bbox_h * 0.25)
        v_max = v_min + roi_height

        # clamp
        u_min = max(0, u_min)
        u_max = min(width - 1, u_max)
        v_min = max(0, v_min)
        v_max = min(height - 1, v_max)

        # 유효성 검사
        if u_min >= u_max or v_min >= v_max:
            return None

        return (u_min, u_max, v_min, v_max)
        
    def _is_valid_depth(self, z):
        if z is None:
            return False

        if np.isnan(z) or np.isinf(z):
            return False

        if z <= 0.0:
            return False

        return True
        
    def _sample_depth_region(self, depth_img, u_min, u_max, v_min, v_max):

        if u_min >= u_max or v_min >= v_max:
            return None

        depth_values = []

        u_samples = np.linspace(u_min, u_max, num=5, dtype=int)
        v_samples = np.linspace(v_min, v_max, num=5, dtype=int)

        for v in v_samples:
            for u in u_samples:
                z = depth_img[v, u]
                if self._is_valid_depth(z):
                    depth_values.append(float(z))

        if len(depth_values) < 5:
            return None

        return float(np.median(depth_values))

def main(args=None):
    rclpy.init(args=args)
    node = ProjectionNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()