import rclpy

from enum import Enum
import math

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesis, ObjectHypothesisWithPose
from phaseshift_interfaces.msg import DetectedObjectArray

# -----------------------------
# STATE
# -----------------------------
class TrackState(Enum):
    ACTIVE = 1
    LOST = 2
    EXPIRED = 3

class DetectionMemoryNode(LifecycleNode):
    def __init__(self):
        super().__init__('detection_memory_node')

        self._sub = None
        self._pub = None

        self._ttl = 10.0
        self._max_distance = 10.0

        # id -> track
        self._tracks = {} 

        self._qos_result = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
    
        self.get_logger().info("Detection Memory Node started...")

    # -----------------------------
    # LIFECYCLE
    # -----------------------------
    def on_configure(self, state):
        self.get_logger().info("Configuring YOLO Tracker nodes...")
        try:
            self._sub = self.create_subscription(
                DetectedObjectArray,
                '/perception/objects_3d',
                self._callback,
                self._qos_result
            )

            self._pub = self.create_publisher(
                DetectedObjectArray,
                '/perception/objects_filtered',
                self._qos_result
            )

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state):
        self.get_logger().info("Activating YOLO Tracker nodes...")
        try:
            pass
        except Exception as e:
            pass
        return super().on_activate(state)
    
    def on_deactivate(self, state):
        self.get_logger().info("Deactivating YOLO Tracker nodes...")
        try:
            pass
        except Exception as e:
            pass
        return super().on_deactivate(state)
    
    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up YOLO Tracker nodes...")
        try:
            if self._sub:
                    self.destroy_subscription(self._sub)
                    self._sub = None

            if self._pub:
                self.destroy_publisher(self._pubs)
                self._pub = None
        except Exception as e:
                self.get_logger().error(f"Cleanup failed: {e}")
        finally:
            # Clean up track list
            # self._tracks = {}
            pass
        return super().on_cleanup(state)
    
    # -----------------------------
    # PROCESS CALLBACK
    # -----------------------------
    def _callback(self, msg):
        now = self._now()

        # Update tracks
        self._update_tracks(msg, now)

        # Update states
        self._update_states(now)

        # Filter valid tracks
        valid_tracks = self._filter_tracks()

        # Cleanup expired
        self._cleanup_tracks()

        # Publish
        self._publish(valid_tracks, msg.header)

    # -----------------------------
    # DETECTION MEMORY PROCESS
    # -----------------------------
    def _update_tracks(self, msg, now):
        for obj in msg.objects:

            track_id = obj.class_id # This is track id for now
            if track_id not in self._tracks:
                self._tracks[track_id] = self._create_track(obj, now)
            
            else:
                self._update_track(self._tracks[track_id], obj, now)
    
    def _create_track(self, obj, now):
        return {
            'obj': obj,
            'last_seen': now,
            'state': TrackState.ACTIVE
        }
    
    def _update_track(self, track, obj, now):
        track['obj'] = obj
        track['last_seen'] = now
        track['state'] = TrackState.ACTIVE

    def _update_states(self, now):
        for track in self._tracks.values():
            dt = now - track['last_seen']

            if dt < self._ttl:
                if track['state'] != TrackState.ACTIVE:
                    track['state'] = TrackState.LOST
            else:
                track['state'] = TrackState.EXPIRED

    def _filter_tracks(self):
        result = []

        for track in self._tracks.values():
            
            if track['state'] == TrackState.EXPIRED:
                continue

            if not self._within_distance(track['obj']):
                continue

            result.append(track['obj'])
        
        return result
    
    def _cleanup_tracks(self):
        self._tracks = {
            k: v for k, v in self._tracks.items()
            if v['state'] != TrackState.EXPIRED
        }

    def _publish(self, tracks, header):
        out = DetectedObjectArray()
        out.header = header
        out.objects = tracks

        self._pub.publish(out)

    # -----------------------------
    # UTILS
    # -----------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9
    
    def _within_distance(self, obj):
        x = obj.pose.position.x
        y = obj.pose.position.y
        d = math.sqrt(x*x + y*y)

        return d < self._max_distance

def main(args=None):
    rclpy.init(args=args)

    node = DetectionMemoryNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()