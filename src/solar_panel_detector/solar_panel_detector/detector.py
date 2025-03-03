#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO  
import numpy as np

class SolarPanelDetector(Node):
    def __init__(self):
        super().__init__('solar_panel_detector')

        # Declare ROS parameters with validation.
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('trigger_topic', '/detection_trigger')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('model_path', '/home/pride/AGV/src/solar_panel_detector/yolov8m.pt')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, trigger_topic, self.trigger_callback, 10)
        self.detection_pub = self.create_publisher(String, detection_topic, 10)

        # Load YOLOv8m model
        self.get_logger().info(f"Loading YOLOv8m model from: {model_path}")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            self.model = None

        self.run_inference = False  # Control flag for inference

    def trigger_callback(self, msg: Bool):
        """Handles the trigger signal for inference."""
        self.run_inference = msg.data
        if self.run_inference:
            self.get_logger().info("Trigger received: Running inference on next image.")

    def image_callback(self, msg: Image):
        """Processes incoming images and runs YOLO inference if triggered."""
        if not self.run_inference or self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        results = self.model(cv_image)
        if not results or len(results[0].boxes) == 0:
            self.get_logger().info("No solar panels detected.")
            self.detection_pub.publish(String(data="No solar panel detected."))
        else:
            num_detected = len(results[0].boxes)
            detection_info = f"Detected {num_detected} solar panel(s)."

            centers = []
            for box in results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                center_x, center_y = (xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2
                centers.append((center_x, center_y))

            detection_info += f" Centers: {centers}"
            self.get_logger().info(detection_info)
            self.detection_pub.publish(String(data=detection_info))

        self.run_inference = False  # Reset trigger flag

def main(args=None):
    rclpy.init(args=args)
    node = SolarPanelDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
