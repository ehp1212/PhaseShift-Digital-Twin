import os
import threading
import queue

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2
import numpy as np

from ultralytics import YOLO

class YoloDetectorNode(LifecycleNode):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self._bridge = CvBridge()
        self._model = None

        self._sub = None
        self._pub = None

        self._queue = queue.Queue(maxsize=2)
        self._thread = None
        self._running = False

        pkg_path = get_package_share_directory("phaseshift_perception")
        self._path = os.path.join(pkg_path, "models", "yolov8n.pt")

        self.get_logger().info("[YOLO] Lifecycle Node Created...")

    # -----------------------------
    # CONFIGURE
    # -----------------------------
    def on_configure(self, state: State):
        self.get_logger().info("Configuring YOLO nodes...")

        try:
            # Model loading 
            self._model = YOLO(self._path)

            qos_detection = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # Pub
            self._pub = self.create_publisher(
                Detection2DArray,
                '/perception/detections_2d',
                qos_detection
            )

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configure failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    # -----------------------------
    # ACTIVATE
    # -----------------------------
    def on_activate(self, state: State):
        self.get_logger().info("Activating YOLO nodes...")

        try:    
            # self._pub.on_activate()

            qos_camera = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )

            # Sub
            self._sub = self.create_subscription(
                CompressedImage,
                '/camera/rgb/image_raw/compressed',
                self._image_callback,
                qos_camera
            )

            # Interface thread
            self._running = True
            self._thread = threading.Thread(target=self._inference_loop, daemon=True)
            self._thread.start()

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activate failed: {e}")
            return TransitionCallbackReturn.FAILURE

    # -----------------------------
    # DEACTIVATE
    # -----------------------------
    def on_deactivate(self, state):
        self.get_logger().info("Deactivating YOLO nodes...")

        self._running = False

        if self._thread:
            self._thread.join()
        
        if self._sub:
            self.destroy_subscription(self._sub)
            self._sub = None
        
        # self._pub.on_deactivate()

        return TransitionCallbackReturn.SUCCESS
    
    # -----------------------------
    # CLEANUP
    # -----------------------------
    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up YOLO nodes...")

        self._model = None

        self.destroy_publisher(self._pub)
        self._pub = None

        return TransitionCallbackReturn.SUCCESS
    
    # -----------------------------
    # IMAGE CALLBACK (lightweight)
    # -----------------------------
    def _image_callback(self, msg: CompressedImage):
        # Drop frame
        if self._queue.full():
            return 
        
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self._queue.put((image, msg.header))
            
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")

    # -----------------------------
    # INFERENCE LOOP (GPU)
    # -----------------------------
    def _inference_loop(self):
        while self._running:
            if not rclpy.ok():
                return
            
            try:
                image, header = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            results = self._model(image, verbose=False)[0]

            msg = Detection2DArray()
            msg.header = header

            for box in results.boxes:
                
                confidence = box.conf[0]
                if confidence < 0.3:
                    continue
                
                det = Detection2D()

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                det.bbox.center.position.x = float((x1 + x2) / 2)
                det.bbox.center.position.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

       
                class_id = int(box.cls[0])
                class_name = self._model.names[class_id]

                valid_class = ["chair"]
                if class_name not in valid_class:
                    continue

                hyp = ObjectHypothesis()
                hyp.class_id = class_name
                hyp.score = float(box.conf[0])

                obj = ObjectHypothesisWithPose()
                obj.hypothesis = hyp

                det.results.append(obj)
                msg.detections.append(det)

            self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = YoloDetectorNode()
    
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