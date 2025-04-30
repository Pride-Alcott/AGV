#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import serial
import os
import time

class SolarPanelDetector(Node):
    def __init__(self):
        super().__init__('solar_panel_detector')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('trigger_topic', '/detection_trigger')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('angles_topic', '/gimbal_angles')
        self.declare_parameter('model_path', '/home/pride/AGV/src/solar_panel_detector/yolov8m.pt')
        self.declare_parameter('output_folder', '/home/pride/Downloads/inference_output')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        # Get parameters
        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        trig_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        det_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        ang_topic = self.get_parameter('angles_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Create output folder
        os.makedirs(self.output_folder, exist_ok=True)

        # ROS interfaces
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, trig_topic, self.trigger_callback, 10)
        self.detection_pub = self.create_publisher(String, det_topic, 10)
        self.angle_pub = self.create_publisher(Point, ang_topic, 10)
        self.create_timer(0.1, self.read_angles)  # 10 Hz timer to read serial

        # Load YOLOv8 model
        self.get_logger().info(f"Loading YOLOv8 model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("YOLOv8 model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            self.model = None

        # Setup serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Serial port opened: {serial_port} @ {baud_rate}bps")
        except Exception as e:
            self.get_logger().error(f"Serial connection error: {e}")
            self.serial_conn = None

        self.run_inference = False
        self.last_angles = (None, None)

    def trigger_callback(self, msg: Bool):
        if msg.data:
            self.run_inference = True
            self.get_logger().info("Inference triggered.")

    def image_callback(self, msg: Image):
        if not self.run_inference or self.model is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        t1 = time.time()
        results = self.model(frame)
        latency = (time.time() - t1) * 1000
        self.get_logger().info(f"Inference latency: {latency:.2f} ms")

        annotated = results[0].plot()
        timestamp = int(time.time())
        out_path = f"{self.output_folder}/inference_{timestamp}.jpg"
        cv2.imwrite(out_path, annotated)
        self.get_logger().info(f"Saved annotated image: {out_path}")

        # Publish and send first center
        if not results or len(results[0].boxes) == 0:
            self.detection_pub.publish(String(data="No solar panel detected."))
        else:
            xy = results[0].boxes[0].xyxy[0].cpu().numpy().astype(int)
            cx = (xy[0] + xy[2]) // 2
            cy = (xy[1] + xy[3]) // 2
            msg_str = f"Detected panel. Center: ({cx},{cy})"
            self.detection_pub.publish(String(data=msg_str))
            self.get_logger().info(msg_str)

            # Send to Arduino
            if self.serial_conn:
                out = f"{cx},{cy}\n".encode()
                try:
                    self.serial_conn.write(out)
                    self.get_logger().info(f"Sent to Arduino: {cx},{cy}")
                except Exception as e:
                    self.get_logger().error(f"Serial write failed: {e}")

        self.run_inference = False

    def read_angles(self):
        if self.serial_conn and self.serial_conn.in_waiting:
            try:
                line = self.serial_conn.readline().decode().strip()
                parts = line.split(',')
                if len(parts) == 2:
                    x_ang = int(parts[0])
                    y_ang = int(parts[1])
                    pt = Point(x=x_ang, y=y_ang, z=0.0)
                    self.angle_pub.publish(pt)
                    self.get_logger().debug(f"Published gimbal angles: {x_ang},{y_ang}")
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SolarPanelDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()