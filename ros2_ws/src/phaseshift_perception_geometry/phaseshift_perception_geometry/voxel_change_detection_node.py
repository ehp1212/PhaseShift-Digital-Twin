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
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

"""
State-aware perception filtering
Probabilistic spatial modeling
Spatial memory abstraction

This node acts as a geometry perception layer that
 bridges static spatial memory (voxel map) and live sensor observations.
"""

class VoxelChangeDetectionNode(LifecycleNode):
    def __init__(self):
        super().__init__('voxel_change_detection_node')
        self._log_info_label('VOXEL CHANGE DETECTION NODE', "started...")

        # -------------------------
        # PARAMETERS
        # -------------------------
        self.declare_parameter('voxel_map_path', '')
        self.declare_parameter('voxel_resolution', 0.15)
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/perception_geometry/change_points')
        self.declare_parameter('target_frame', 'map')

        self._voxel_map_path = ''
        self._resolution = 0.15
        self._input_topic = '/velodyne_points'
        self._output_topic = '/perception_geometry/change_points'

        self._voxel_dict = {}
        self._map_loaded = False
        self._hash_mul = np.array([73856093, 19349663, 83492791], dtype=np.int64)

        self._sub = None
        self._dynamic_pub = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # TODO: Debuging voxel map
        self._map_pub = self.create_publisher(PointCloud2, '/voxel_map', 10)

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

        self._map_loaded = True
        self._log_info_label('VOXEL CHANGE DETECTION NODE', 'configured successfully')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        if not self._map_loaded:
            self._log_info_label('VOXEL CHANGE DETECTION NODENODE', "Failed to load voxel map")
            return TransitionCallbackReturn.FAILURE

        sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
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

            points = np.asarray(pcd.points)
            if points is None or points.size == 0:
                self.get_logger().error("Voxel map has no valid points")
                return False

            if pcd.has_colors():
                confs = np.asarray(pcd.colors)[:, 0]
            else:
                confs = np.ones(len(points))

            # voxel index
            indices = np.floor(points / self._resolution).astype(np.int32)

            for (ix, iy, iz), conf in zip(indices, confs):
                key = (int(ix), int(iy), int(iz))
                self._voxel_dict[key] = float(conf)

            self.get_logger().info(f"Loaded voxel map: {len(self._voxel_dict)} voxels")
            return True
        
        except Exception as e:
            self.get_logger().error(f"Load failed: {e}")
            return False
        finally:
            # TODO: Testing voxel map
            header = Header()
            header.frame_id = 'map'

            msg = point_cloud2.create_cloud_xyz32(
                header,
                points.tolist()
            )

            self._map_msg = msg

            self.create_timer(1.0, self._publish_map)

    def _publish_map(self):
        if self._map_msg:
            self._map_pub.publish(self._map_msg)

    def _points_callback(self, msg):
        transformed_msg = self._transform_cloud_msg(msg)
        if transformed_msg is None:
            return

        points = self._pointcloud2_to_numpy(transformed_msg)
        if points.size == 0:
            return

        indices = np.floor(points / self._resolution).astype(np.int32)

        voxel_keys = [
            (int(ix), int(iy), int(iz))
            for ix, iy, iz in indices
        ]

        mask = [
            not self._is_static(key)
            for key in voxel_keys
        ]

        dynamic_points = points[mask]

        self._publish_dynamic_points(dynamic_points, self._target_frame, msg.header.stamp)

    def _transform_cloud_msg(self, cloud_msg: PointCloud2):
        source_frame = cloud_msg.header.frame_id

        if source_frame == self._target_frame:
            return cloud_msg

        if not self._tf_buffer.can_transform(
            self._target_frame,
            source_frame,
            cloud_msg.header.stamp,
            timeout=rclpy.duration.Duration(seconds=0.1)
        ):
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return None

        try:
            transformed_cloud = do_transform_cloud(cloud_msg, transform)
            return transformed_cloud
        except Exception as e:
            self.get_logger().warn(f"Cloud transform failed: {e}")
            return None

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

    def _publish_dynamic_points(self, dynamic_points, frame_id: str, timestamp):

        if self._dynamic_pub is None:
            return
        
        header = Header()
        header.stamp = timestamp
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
        
    # Probabilistic spatial model
    def _is_static(self, key):
        x, y, z = key

        best_conf = 0.0

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    neighbor = (x + dx, y + dy, z + dz)

                    if neighbor in self._voxel_dict:
                        conf = self._voxel_dict[neighbor]
                        best_conf = max(best_conf, conf)


        if best_conf > 0.6:
            return True   # static

        return False      # possible dynamic
    
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