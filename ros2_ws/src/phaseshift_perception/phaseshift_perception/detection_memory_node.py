import rclpy

from enum import Enum
import math

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from phaseshift_interfaces.msg import EstimatedObjectArray, TrackedObject, TrackedObjectArray

# -----------------------------
# STATE
# -----------------------------
class TrackState(Enum):
    TENTATIVE = 0
    ACTIVE = 1
    LOST = 2
    EXPIRED = 3

class DetectionMemoryNode(LifecycleNode):
    def __init__(self):
        super().__init__('detection_memory_node')

        self._sub = None
        self._pub = None

        self._max_distance = 10.0

        # id -> track
        self._tracks = {} 
        self._id_counter = 0

        self._match_distance_threshold = 1.0
        self._active_timeout = 0.5
        self._ttl = 2.0
        self._reacquire_time = 1.0
        self._min_hits = 3
        self._max_misses = 5
        self._velocity_alpha = 0.7

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
                EstimatedObjectArray,
                '/perception/estimated_objects',
                self._callback,
                self._qos_result
            )

            self._pub = self.create_publisher(
                TrackedObjectArray,
                '/perception/tracked_objects',
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
                self.destroy_publisher(self._pub)
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
        matched_track_ids = set()

        for obj in msg.objects:
            track = self._find_matching_track(obj, now)

            if track is None:
                new_track = self._create_track(obj, now)
                self._tracks[new_track['id']] = new_track
                matched_track_ids.add(new_track['id'])
            else:
                self._update_track(track, obj, now)
                matched_track_ids.add(track['id'])

        for track_id, track in self._tracks.items():
            if track_id not in matched_track_ids:
                track['misses'] = track.get('misses', 0) + 1

    def _update_track(self, track, obj, now):
        """
        Update track 
        Calculate velocity based on timestamp
        """
        prev = track['obj'].pose.position
        curr = obj.pose.position

        dt = now - track['last_seen']

        if dt > 0.0:
            new_vx = (curr.x - prev.x) / dt
            new_vy = (curr.y - prev.y) / dt
        else:
            new_vx, new_vy = 0.0, 0.0

        prev_vx, prev_vy, prev_vz = track.get('velocity', (0.0, 0.0, 0.0))
        alpha = self._velocity_alpha

        smoothed_vx = alpha * prev_vx + (1.0 - alpha) * new_vx
        smoothed_vy = alpha * prev_vy + (1.0 - alpha) * new_vy

        track['velocity'] = (smoothed_vx, smoothed_vy, 0.0)

        track['obj'] = obj
        track['last_seen'] = now
        track['hits'] = track.get('hits', 0) + 1
        track['misses'] = 0
        
        if track['state'] == TrackState.LOST:
            track['state'] = TrackState.ACTIVE

        if track['state'] == TrackState.TENTATIVE and track['hits'] >= self._min_hits:
            track['state'] = TrackState.ACTIVE  

    def _find_matching_track(self, obj, now):

        best_track = None
        best_dist = float('inf')

        for track in self._tracks.values():
            if track['state'] == TrackState.EXPIRED:
                continue

            if track['obj'].class_id != obj.class_id:
                continue

            dt = now - track['last_seen']
            if dt > self._reacquire_time:
                continue

            dist = self._distance(track['obj'], obj)
            if dist > self._match_distance_threshold:
                continue

            if dist < best_dist:
                best_dist = dist
                best_track = track

        return best_track

    def _create_track(self, obj, now):
        return {
            'id': self._next_id(),
            'obj': obj,
            'last_seen': now,
            'state': TrackState.TENTATIVE,
            'velocity': (0.0, 0.0, 0.0),
            'hits': 1,
            'misses': 0,
        }
            
    def _update_states(self, now):
        for track in self._tracks.values():
            dt = now - track['last_seen']
            misses = track.get('misses', 0)
            hits = track.get('hits', 0)

            if track['state'] == TrackState.TENTATIVE:
                if hits >= self._min_hits:
                    track['state'] = TrackState.ACTIVE
                elif misses > self._max_misses or dt > self._ttl:
                    track['state'] = TrackState.EXPIRED

            elif track['state'] == TrackState.ACTIVE:
                if dt <= self._active_timeout:
                    track['state'] = TrackState.ACTIVE
                elif dt <= self._ttl:
                    track['state'] = TrackState.LOST
                else:
                    track['state'] = TrackState.EXPIRED

            elif track['state'] == TrackState.LOST:
                if dt > self._ttl or misses > self._max_misses:
                    track['state'] = TrackState.EXPIRED

    def _filter_tracks(self):
        result = []

        for track in self._tracks.values():
            
            if track['state'] != TrackState.ACTIVE:
                continue

            if not self._within_distance(track['obj']):
                continue

            result.append(track)
        
        return result
    
    def _cleanup_tracks(self):
        self._tracks = {
            k: v for k, v in self._tracks.items()
            if v['state'] != TrackState.EXPIRED
        }

    def _publish(self, tracks, header):

        out = TrackedObjectArray()
        out.header = header

        for track in tracks:

            msg = TrackedObject()

            msg.id = track['id']
            msg.class_id = track['obj'].class_id
            msg.pose = track['obj'].pose

            vx, vy, vz = track.get('velocity', (0.0, 0.0, 0.0))
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.velocity.z = vz

            msg.is_dynamic = track['obj'].is_dynamic
            msg.motion_confidence = track['obj'].motion_confidence

            msg.last_seen_time = track['last_seen']

            out.objects.append(msg)

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

    def _next_id(self):
        self._id_counter += 1
        return self._id_counter
    
    def _distance(self, obj1, obj2):
        p1 = obj1.pose.position
        p2 = obj2.pose.position

        dx = p1.x - p2.x
        dy = p1.y - p2.y

        return math.sqrt(dx*dx + dy*dy)

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