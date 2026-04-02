import rclpy
from rclpy.time import Time
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

import struct
import numpy as np
import open3d as o3d
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class VoxelChangeDetectionNode(LifecycleNode):
    def __init__(self):
        super().__init__('voxel_change_detection_node')
        self._log_info_label('VOXEL CHANGE DETECTION NODE', "started...")

        # -------------------------
        # PARAMETERS
        # -------------------------
        self.declare_parameter('voxel_map_path', '')
        self.declare_parameter('voxel_resolution', 0.2)
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/perception_geometry/change_points')
        self.declare_parameter('target_frame', 'map')

        self._voxel_map_path = ''
        self._resolution = 0.2
        self._input_topic = '/velodyne_points'
        self._output_topic = '/perception_geometry/change_points'

        self._map_loaded = False
        self._voxel_map = None
        self._voxel_hash = None
        self._hash_mul = np.array([73856093, 19349663, 83492791], dtype=np.int64)

        self._sub = None
        self._dynamic_pub = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # ==============================
    # LIFECYCLE
    # ==============================
    def on_configure(self, state):

        self._voxel_map_path = self.get_parameter('voxel_map_path').value
        self._resolution = float(self.get_parameter('voxel_resolution').value)
        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._target_frame = self.get_parameter('target_frame').value

        if not self._voxel_map_path:
            self._log_info_label('VOXEL CHANGE DETECTION NODE', "No voxel_map_path provided...")
            return TransitionCallbackReturn.FAILURE
        
        success = self._load_voxel_map(self._voxel_map_path)
        if not success:
            self._log_info_label('VOXEL CHANGE DETECTION NODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE

        self._dynamic_pub = self.create_publisher(
            PointCloud2,
            self._output_topic,
            10
        )

        self.map_loaded = True
        self._log_info_label('VOXEL CHANGE DETECTION NODE', 'configured successfully')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        if not self.map_loaded:
            self._log_info_label('VOXEL CHANGE DETECTION NODENODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE

        sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self._sub = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._points_callback,
            sensor
        )

        self._log_info_label('VOXEL CHANGE DETECTION NODE', 'activated')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        self._log_info_label('VOXEL CHANGE DETECTION NODE', 'deactivating...')

        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self._log_info_label('VOXEL CHANGE DETECTION NODE', 'cleaning up...')

        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None

        self._voxel_map.clear()
        self._map_loaded = False
        self._dynamic_pub = None

        return TransitionCallbackReturn.SUCCESS    
    
    # ==============================
    # PUBLIC API
    # ==============================
    def set_voxel_map_path(self, map_path: str):
        if not map_path:
            self.get_logger().error("Map path is invalid.")
            return
        
        self._voxel_map_path = map_path
    
    # ==============================
    # INTERNAL
    # ==============================

    def _load_voxel_map(self, path):

        try:
            self._log_info_label('VOXEL COSTMAP NODE', f"Loading PCD: {path}")

            pcd = o3d.io.read_point_cloud(path)

            # if len(pcd.points) == 0:
            #     self.get_logger().error("Empty PCD file")
            #     return False
            
            points = np.asarray(pcd.points)
            if points is None or points.size == 0:
                self.get_logger().error("Voxel map has no valid points")
                return False

            # voxel index
            indices = np.floor(points / self._resolution).astype(np.int32)

            # hash 
            self._voxel_hash = np.sum(indices * self._hash_mul, axis=1)

            # self._log_info_label(
            #     'VOXEL CHANGE DETECTION NODE',
            #     f'Loaded {len(self._voxel_map)} voxels'
            # )

            return True
        
        except Exception as e:
            self.get_logger().error(f"Load failed: {e}")
            return False

    def _points_callback(self, msg):
        points = self._pointcloud2_to_numpy(msg)

        if points.size == 0:
            return
                
        transformed = self._transform_points(points, msg.header.frame_id, msg.header.stamp)
        if transformed is None:
            return
        
        # voxel index
        indices = np.floor(transformed / self._resolution).astype(np.int32)
        
        # hash
        hashes = np.sum(indices * self._hash_mul, axis=1)

        # comparison
        mask = ~np.isin(hashes, self._voxel_hash)

        # dynamic points
        dynamic_points = transformed[mask]

        self._publish_dynamic_points(dynamic_points, self._target_frame)

    def _pointcloud2_to_numpy(self, msg):
        points_iter = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        points = np.array(list(points_iter))  # dtype 유지

        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        xyz = np.stack([points['x'], points['y'], points['z']], axis=1)
        return xyz.astype(np.float32)

    def _publish_dynamic_points(self, dynamic_points, frame_id: str):

        if self._dynamic_pub is None:
            return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id 

        # -----------------------------
        # fields 
        # -----------------------------
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # -----------------------------
        # data packing
        # -----------------------------
        data = []
        for p in dynamic_points:
            data.append(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))

        data = b''.join(data)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(dynamic_points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(dynamic_points)
        msg.is_dense = True
        msg.data = data

        self._dynamic_pub.publish(msg)
        
    def _transform_points(self, points: np.ndarray, source_frame: str, timestamp: str):

        if source_frame == self._target_frame:
            return points
            
        # if not self._tf_buffer.can_transform(
        #     self._target_frame,
        #     source_frame,
        #     rclpy.time.Time(),
        #     timeout=rclpy.duration.Duration(seconds=0.1)
        # ):
        #     self.get_logger().warn(
        #         f"TF not available: {source_frame} → {self._target_frame}"
        #     )
        #     return None        

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return None
            
        # translation
        t = transform.transform.translation
        tx, ty, tz = t.x, t.y, t.z

        # rotation (quaternion)
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # quaternion → rotation matrix
        R = self._quat_to_rot(qx, qy, qz, qw)

        # transform
        transformed = (R @ points.T).T
        transformed += np.array([tx, ty, tz])

        return transformed        

    # ==============================
    # UTILS
    # ==============================

    def _log_info_label(self, label: str, msg: str):
        self.get_logger().info(f"\033[93m[{label}]\033[0m - {msg}")  # Green

    def _quat_to_rot(self, x, y, z, w):
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
    
    def _points_to_voxel_indices(self, points: np.ndarray):
        return np.floor(points / self._resolution).astype(np.int32)

def main():
    rclpy.init()
    node = VoxelChangeDetectionNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try: 
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        executor.shutdown()
        node.destroy_node()