#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO  # Ensure you have installed ultralytics via pip
import numpy as np

class SolarPanelDetector(Node):
    def __init__(self):
        super().__init__('solar_panel_detector')
        # Declare parameters for topics and model path.
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('trigger_topic', '/detection_trigger')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('model_path', '\\wsl.localhost\Ubuntu-22.04\home\pride\AGV\src\solar_panel_detector\yolov8m.pt')  # <-- Update this path

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Initialize CvBridge for image conversion.
        self.bridge = CvBridge()

        # Subscribe to the image topic and the trigger topic.
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, trigger_topic, self.trigger_callback, 10)
        # Publisher for detection messages.
        self.detection_pub = self.create_publisher(String, detection_topic, 10)

        # Load the YOLOv8m model.
        self.get_logger().info(f"Loading YOLOv8m model from: {model_path}")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            self.model = None

        # This flag determines if the next image should be processed.
        self.run_inference = False

    def trigger_callback(self, msg: Bool):
        # When a True trigger is received, set the flag to run inference.
        if msg.data:
            self.get_logger().info("Trigger received: running inference on next image.")
            self.run_inference = True
        else:
            self.get_logger().info("Trigger reset (False).")
            self.run_inference = False

    def image_callback(self, msg: Image):
        # Only run inference if the trigger flag is set.
        if not self.run_inference:
            return

        if self.model is None:
            self.get_logger().warn("YOLO model not loaded. Skipping inference.")
            return

        # Convert the ROS image message to an OpenCV image.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Run the YOLO model on the image.
        results = self.model(cv_image)
        boxes = results[0].boxes  # Get bounding boxes from the first result

        if boxes is not None and len(boxes) > 0:
            count = len(boxes)
            detection_str = f"Detected {count} solar panel(s)."
            self.get_logger().info(detection_str)
            # Optionally, draw bounding boxes for visualization.
            for box in boxes:
                # box.xyxy: bounding box coordinates; box.conf: confidence.
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                confidence = float(box.conf.cpu().numpy()[0])
                cv2.rectangle(cv_image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{confidence:.2f}", (xyxy[0], xyxy[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Optionally, display the image.
            cv2.imshow("Solar Panel Detection", cv_image)
            cv2.waitKey(1)
        else:
            detection_str = "No solar panel detected."
            self.get_logger().info(detection_str)

        # Publish the detection message.
        self.detection_pub.publish(String(data=detection_str))
        # Reset the flag so that the node waits for the next trigger.
        self.run_inference = False

def main(args=None):
    rclpy.init(args=args)
    node = SolarPanelDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO  # Ensure you have installed ultralytics via pip
import numpy as np

class SolarPanelDetector(Node):
    def __init__(self):
        super().__init__('solar_panel_detector')
        # Declare parameters for topics and model path.
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('trigger_topic', '/detection_trigger')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('model_path', "\\wsl.localhost\Ubuntu-22.04\home\pride\AGV\src\solar_panel_detector\yolov8m.pt")  # <-- Update this path

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Initialize CvBridge for image conversion.
        self.bridge = CvBridge()

        # Subscribe to the image topic and the trigger topic.
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, trigger_topic, self.trigger_callback, 10)
        # Publisher for detection messages.
        self.detection_pub = self.create_publisher(String, detection_topic, 10)

        # Load the YOLOv8m model.
        self.get_logger().info(f"Loading YOLOv8m model from: {model_path}")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            self.model = None

        # This flag determines if the next image should be processed.
        self.run_inference = False

    def trigger_callback(self, msg: Bool):
        # When a True trigger is received, set the flag to run inference.
        if msg.data:
            self.get_logger().info("Trigger received: running inference on next image.")
            self.run_inference = True
        else:
            self.get_logger().info("Trigger reset (False).")
            self.run_inference = False

    def image_callback(self, msg: Image):
        # Only run inference if the trigger flag is set.
        if not self.run_inference:
            return

        if self.model is None:
            self.get_logger().warn("YOLO model not loaded. Skipping inference.")
            return

        # Convert the ROS image message to an OpenCV image.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Run the YOLO model on the image.
        results = self.model(cv_image)
        boxes = results[0].boxes  # Get bounding boxes from the first result

        if boxes is not None and len(boxes) > 0:
            count = len(boxes)
            detection_str = f"Detected {count} solar panel(s)."
            self.get_logger().info(detection_str)
            # Optionally, draw bounding boxes for visualization.
            for box in boxes:
                # box.xyxy: bounding box coordinates; box.conf: confidence.
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                confidence = float(box.conf.cpu().numpy()[0])
                cv2.rectangle(cv_image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{confidence:.2f}", (xyxy[0], xyxy[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Optionally, display the image.
            cv2.imshow("Solar Panel Detection", cv_image)
            cv2.waitKey(1)
        else:
            detection_str = "No solar panel detected."
            self.get_logger().info(detection_str)

        # Publish the detection message.
        self.detection_pub.publish(String(data=detection_str))
        # Reset the flag so that the node waits for the next trigger.
        self.run_inference = False

def main(args=None):
    rclpy.init(args=args)
    node = SolarPanelDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

