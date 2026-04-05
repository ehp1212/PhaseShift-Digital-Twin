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
    """
    Role:
    - Convert 2D detections into 3D positions using depth camera
    - Transform object coordinates from camera frame to global frame (map)

    Pipeline:
    2D Detection → Depth Sampling → 3D Reconstruction → TF Transform → 3D Object

    Notes:
    - This node bridges perception (image space) and robotics (world space)
    - No tracking or state estimation is performed here
    - Output is purely geometric (metric space)
    """

    # TODO: Fix this, hard coded for unity depth camera distance value
    DEPTH_SCALE = 20

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
                '/perception/tracked_detections',
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
        """
        Main processing pipeline for 2D → 3D projection.

        Steps:
        1. Validate depth and camera info availability
        2. Extract camera intrinsics (fx, fy, cx, cy)
        3. For each detection:
            - Compute ROI (adaptive roi calculation)
            - Sample depth points (grid sampling)
            - Estimate depth using median
            - Back-project to 3D (camera frame)
            - Transform to target frame (map)
        4. Publish 3D detected objects

        Notes:
        - Uses median depth to reduce noise and outliers
        - ROI focuses on lower part of object (ground contact assumption)
        """

        if self._latest_depth is None or self._camera_info is None:
            return
        
        if self._latest_depth_header is None:
            return
            
        dt = abs(
            (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) -
            (self._latest_depth_header.stamp.sec + self._latest_depth_header.stamp.nanosec * 1e-9)
        )

        if dt > 0.1:
            self.get_logger().warn(f"[Projection] Depth-Detection desync: {dt:.3f}s")
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

            # ---------------------------
            # Select ROI
            # ---------------------------
            roi = self._compute_adaptive_roi(
                u_center, v_center, bbox_w, bbox_h, width, height
            )

            points = []
        
            if roi is not None:
                u_min, u_max, v_min, v_max = roi

                # 5x5 grid sampling
                u_samples = np.linspace(u_min, u_max, num=5, dtype=int)
                v_samples = np.linspace(v_min, v_max, num=5, dtype=int)

                for v in v_samples:
                    for u in u_samples:

                        z = depth_img[v, u] * self.DEPTH_SCALE

                        if not self._is_valid_depth(z):
                            continue

                        # 아직 scaling 전
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy

                        points.append([x, y, z])

            # ---------------------------
            # median 
            # ---------------------------
            if len(points) >= 5:
                pts = np.array(points)

                z_values = pts[:, 2]

                median_z = np.median(z_values)

                # depth filtering
                mask = np.abs(z_values - median_z) < 1.0
                filtered_pts = pts[mask]

                if len(pts) >= 3:
                    pts = filtered_pts

                # ---------------------------
                # robust depth (ground-aware)
                # ---------------------------
                z = float(np.percentile(pts[:, 2], 50))

                # ---------------------------
                # position
                # ---------------------------
                x = float(np.median(pts[:, 0]))
                y = float(np.median(pts[:, 1]))

                # optional clamp
                z = np.clip(z, 0.2, 10.0)

            else:
                # fallback (center pixel)
                z = depth_img[v_center, u_center] * self.DEPTH_SCALE

                if not self._is_valid_depth(z):
                    continue

                x = (u_center - cx) * z / fx
                y = (v_center - cy) * z / fy

            # ---------------------------
            # TF
            # ---------------------------
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
            # obj.pose.position.z = 0.0
            obj.pose.orientation.w = 1.0

            out.objects.append(obj)

        if len(out.objects) > 0:
            self._pub_objects.publish(out)
        
    def _is_valid_pixel(self, u, v, width, height):
        return 0 <= u < width and 0 <= v < height
    
    def _compute_adaptive_roi(self, u_center, v_center, bbox_w, bbox_h, width, height):
        """
        Adaptive ROI selection based on object size.

        Strategy:
        - Small objects → center-focused ROI
        - Large objects → bottom-focused ROI (ground contact assumption)
        """

        bbox_w = max(4, bbox_w)
        bbox_h = max(4, bbox_h)

        bbox_area = bbox_w * bbox_h

        # ---------------------------
        # Adaptive vertical ROI
        # ---------------------------
        if bbox_area < 2000:
            # small object → center
            v_min = int(v_center - bbox_h * 0.2)
            v_max = int(v_center + bbox_h * 0.2)
        else:
            # large object → bottom
            v_min = int(v_center + bbox_h * 0.2)
            v_max = int(v_center + bbox_h * 0.5)        


        # ---------------------------
        # horizontal ROI (center 50%)
        # ---------------------------
        roi_width = int(bbox_w * 0.5)

        u_min = int(u_center - roi_width / 2)
        u_max = int(u_center + roi_width / 2)

        # ---------------------------
        # clamp
        # ---------------------------
        u_min = max(0, u_min)
        u_max = min(width - 1, u_max)
        v_min = max(0, v_min)
        v_max = min(height - 1, v_max)

        if u_min >= u_max or v_min >= v_max:
            return None

        return (u_min, u_max, v_min, v_max)

    def _compute_bottom_roi(self, u_center, v_center, bbox_w, bbox_h, width, height):
        """
        Compute a region-of-interest focused on the lower part of the bounding box.

        Rationale:
        - Lower part of objects is more likely to be grounded
        - Reduces noise from upper regions (e.g., occlusions, background)

        Strategy:
        - Use 30%~50% vertical region from bbox bottom
        - Use center 50% horizontally
        """

        # minimum scale
        bbox_w = max(4, bbox_w)
        bbox_h = max(4, bbox_h)

        # ---------------------------
        # bbox boundary
        # ---------------------------
        u_min_box = int(u_center - bbox_w / 2)
        u_max_box = int(u_center + bbox_w / 2)

        v_min_box = int(v_center - bbox_h / 2)
        v_max_box = int(v_center + bbox_h / 2)

        bottom_start_ratio = 0.3
        bottom_end_ratio = 0.5

        # height ROS
        v_min = int(v_min_box + bbox_h * bottom_start_ratio)
        v_max = int(v_min_box + bbox_h * bottom_end_ratio)

        # width ROI (center 50%)
        roi_width = int(bbox_w * 0.5)

        u_min = int(u_center - roi_width / 2)
        u_max = int(u_center + roi_width / 2)

        # clamp
        u_min = max(0, u_min)
        u_max = min(width - 1, u_max)
        v_min = max(0, v_min)
        v_max = min(height - 1, v_max)

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