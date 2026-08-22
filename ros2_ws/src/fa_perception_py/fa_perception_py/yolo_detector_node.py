"""Camera-based 2D detection: runs a pretrained YOLO model on every frame
from the vehicle's camera and publishes the results as a Detection2DArray.

This is intentionally NOT fused with the LiDAR obstacle detector yet -- it
exists to prove the camera -> YOLO -> ROS2 path works on its own, using a
Gazebo Fuel model with real geometry (a plain colored box would never be
recognized by a COCO-trained model; see docs/learning-log.md).
"""
import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.4)

        model_name = self.get_parameter("model").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value

        self.get_logger().info(f"loading YOLO model '{model_name}' (first run downloads it)")
        self.model = YOLO(model_name)
        self.bridge = cv_bridge.CvBridge()

        self.publisher = self.create_publisher(Detection2DArray, "detections_2d", 10)
        self.subscription = self.create_subscription(Image, "camera", self.on_image, 10)

    def on_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(cv_image, conf=self.confidence_threshold, verbose=False)

        detections = Detection2DArray()
        detections.header = msg.header

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = self.model.names[class_id]

            det = Detection2D()
            det.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = confidence
            det.results.append(hypothesis)

            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            det.bbox = bbox

            detections.detections.append(det)
            self.get_logger().info(
                f"{class_name} ({confidence:.2f}) at [{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]"
            )

        self.publisher.publish(detections)
        if not results[0].boxes:
            self.get_logger().info("0 detections", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
