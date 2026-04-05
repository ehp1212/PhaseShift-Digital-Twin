import rclpy

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesis, ObjectHypothesisWithPose

# TODO: Use custom msg
# For testing 
# hyp.class_id = f"{track_id}"


class YOLOTrackerNode(LifecycleNode):
    """
    Role:
    - Associate 2D detections across frames
    - Maintain short-term identity (tracking ID)
    - Provide temporally consistent detections

    Notes:
    - Lightweight tracker (IoU + distance)
    - No motion model (Kalman not used here)
    - No semantic or state reasoning
    """

    def __init__(self):
        super().__init__('yolo_tracker_node')

        self._sub = None
        self._pub = None

        self._tracks = []
        self._next_id = 0

        self._qos_result = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
    
    # -----------------------------
    # LIFECYCLE
    # -----------------------------
    def on_configure(self, state):
        self.get_logger().info("Configuring YOLO Tracker nodes...")
        try:
            qos = self._qos_result

            self._sub = self.create_subscription(
                Detection2DArray,
                '/perception/detections_2d',
                self._callback,
                self._qos_result
            )

            self._pub = self.create_publisher(
                Detection2DArray,
                '/perception/tracked_detections',
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
            pass
        except Exception as e:
            pass
        finally:
            # Clean up track list
            self._tracks = []
        return super().on_cleanup(state)
    
    # -----------------------------
    # PROCESS CALLBACK
    # -----------------------------
    def _callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        used_tracks = set()

        # -----------------------------
        # Update tracks
        # -----------------------------
        for det in msg.detections:

            best_idx = -1
            best_score = 0.0

            for i, track in enumerate(self._tracks):

                if i in used_tracks:
                    continue

                iou = self._iou(det, track['bbox'])
                dist = self._center_distance(det, track['bbox'])

                score = iou - (dist * 0.001)

                if iou > 0.3 or dist < 30:
                    if score > best_score:
                        best_score = score
                        best_idx = i

            if best_idx >= 0:
                track = self._tracks[best_idx]

                # bbox smoothing
                track['bbox'] = self._smooth_bbox(track['bbox'], det.bbox)

                track['last_seen'] = now
                track['hits'] += 1
                track['misses'] = 0

                used_tracks.add(best_idx)

            else:
                # new track
                self._tracks.append({
                    'id': self._next_id,
                    'bbox': det.bbox,
                    'last_seen': now,
                    'hits': 1,
                    'misses': 0
                })
                self._next_id += 1
        
        # -----------------------------
        # Update Misses
        # -----------------------------
        for track in self._tracks:
            if now - track['last_seen'] > 0.1:
                track['misses'] += 1        

        # -----------------------------
        # Remove Old tracks (TTL)
        # -----------------------------
        self._tracks = [
            t for t in self._tracks
            if t['misses'] < 5
        ]

        # -----------------------------
        # PUBLISH
        # -----------------------------
        out = Detection2DArray()
        out.header = msg.header

        for track in self._tracks:

            # minimum condition
            if track['hits'] < 2:
                continue

            det = Detection2D()
            det.bbox = track['bbox']

            hyp = ObjectHypothesis()
            # track id for testing
            hyp.class_id = str(track['id']) 
            hyp.score = 1.0

            obj = ObjectHypothesisWithPose()
            obj.hypothesis = hyp

            det.results.append(obj)
            out.detections.append(det)

        self._pub.publish(out)

    # -----------------------------
    # CHECK TRACKING
    # -----------------------------
    def _match(self, det, track) -> bool:
        iou_score = self._iou(det, track['bbox'])
        dist = self._center_distance(det, track['bbox'])

        return iou_score > 0.15 or dist < 20

    def _iou(self, det, track_bbox) -> float:
        """
        Compute Intersection-over-Union between two bounding boxes
        """

        # det bbox
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y

        x1_det = cx - w / 2
        y1_det = cy - h / 2
        x2_det = cx + w / 2
        y2_det = cy + h / 2

        # track bbox
        cx_t = track_bbox.center.position.x
        cy_t = track_bbox.center.position.y
        w_t = track_bbox.size_x
        h_t = track_bbox.size_y

        x1_tr = cx_t - w_t / 2
        y1_tr = cy_t - h_t / 2
        x2_tr = cx_t + w_t / 2
        y2_tr = cy_t + h_t / 2

        # Intersection
        xi1 = max(x1_det, x1_tr)
        yi1 = max(y1_det, y1_tr)
        xi2 = min(x2_det, x2_tr)
        yi2 = min(y2_det, y2_tr)

        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)

        # Union
        det_area = w*h
        tr_area = w_t*h_t

        union_area = det_area + tr_area - inter_area

        if union_area == 0:
            return 0.0
        
        return inter_area / union_area

    def _center_distance(self, det, track_bbox):
        """
        Euclidean distance between bbox centers
        """

        cx1 = det.bbox.center.position.x
        cy1 = det.bbox.center.position.y

        cx2 = track_bbox.center.position.x
        cy2 = track_bbox.center.position.y

        return ((cx1 - cx2) ** 2 + (cy1 - cy2) ** 2) ** 0.5
    
    def _smooth_bbox(self, old, new, alpha=0.7):
        """
        Apply exponential smoothing to reduce bbox jitter
        """

        new.center.position.x = alpha * old.center.position.x + (1 - alpha) * new.center.position.x
        new.center.position.y = alpha * old.center.position.y + (1 - alpha) * new.center.position.y

        new.size_x = alpha * old.size_x + (1 - alpha) * new.size_x
        new.size_y = alpha * old.size_y + (1 - alpha) * new.size_y

        return new

def main(args=None):
    rclpy.init(args=args)

    node = YOLOTrackerNode()
    
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