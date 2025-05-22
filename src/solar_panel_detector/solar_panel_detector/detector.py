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
import numpy as np
from threading import Lock

class SolarPanelDetector(Node):
    def __init__(self):
        super().__init__('solar_panel_detector')

        # Declare parameters with Docker-friendly defaults
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('trigger_topic', '/detection_trigger')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('angles_topic', '/gimbal_angles')
        self.declare_parameter('model_path', '/Solar-farm-AGV/src/best.pt')  # YOLOv8
        self.declare_parameter('output_folder', '/AGV/output')
        self.declare_parameter('serial_port', '/dev/arduino')  #from the udev rule
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_size', 640)  # Configurable for Pi performance
        self.declare_parameter('save_images', True)
        self.declare_parameter('max_detections', 5)  # Limit processing

        # Get parameters
        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        trig_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        det_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        ang_topic = self.get_parameter('angles_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.img_size = self.get_parameter('image_size').get_parameter_value().integer_value
        self.save_images = self.get_parameter('save_images').get_parameter_value().bool_value
        self.max_detections = self.get_parameter('max_detections').get_parameter_value().integer_value

        # Create output folder
        os.makedirs(self.output_folder, exist_ok=True)

        # ROS interfaces
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, trig_topic, self.trigger_callback, 10)
        self.detection_pub = self.create_publisher(String, det_topic, 10)
        self.angle_pub = self.create_publisher(Point, ang_topic, 10)
        
        # Reduce timer frequency to save CPU
        self.create_timer(0.2, self.read_angles)  # 5 Hz instead of 10 Hz

        # Thread safety
        self.serial_lock = Lock()

        # Load YOLOv8 model with Pi optimizations
        self.get_logger().info(f"Loading YOLOv8 model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            # Pre-warm the model with a dummy prediction for better first-inference performance
            dummy_img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
            self.model.predict(dummy_img, verbose=False, conf=self.conf_threshold, imgsz=self.img_size)
            self.get_logger().info("YOLOv8 model loaded and warmed up successfully.")
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            self.model = None

        # Setup serial connection with retry logic
        self.serial_conn = None
        self.setup_serial(serial_port, baud_rate)

        self.run_inference = False
        self.last_angles = (None, None)
        self.frame_count = 0

    def setup_serial(self, port, baud_rate, max_retries=5):
        """Setup serial connection with retry logic"""
        for attempt in range(max_retries):
            try:
                self.serial_conn = serial.Serial(port, baud_rate, timeout=1)
                self.get_logger().info(f"Serial port opened: {port} @ {baud_rate}bps")
                return
            except Exception as e:
                self.get_logger().warn(f"Serial connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(2)  # Wait before retry
        
        self.get_logger().error("Failed to establish serial connection after all retries")

    def trigger_callback(self, msg: Bool):
        if msg.data:
            self.run_inference = True
            self.get_logger().info("Inference triggered.")

    def image_callback(self, msg: Image):
        if not self.run_inference or self.model is None:
            return

        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        # Resize if image is larger than configured size (save computation)
        h, w = frame.shape[:2]
        if max(h, w) > self.img_size:
            scale = self.img_size / max(h, w)
            new_w, new_h = int(w * scale), int(h * scale)
            frame = cv2.resize(frame, (new_w, new_h))
            self.get_logger().debug(f"Resized frame from {w}x{h} to {new_w}x{new_h}")

        # Run inference with Pi-optimized settings
        t1 = time.time()
        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            imgsz=self.img_size,
            verbose=False,
            device='cpu',  # Force CPU on Pi
            half=False,    # Don't use FP16 on CPU
            max_det=self.max_detections
        )
        latency = (time.time() - t1) * 1000
        self.get_logger().info(f"Inference latency: {latency:.2f} ms")

        # Process results
        if not results or len(results[0].boxes) == 0:
            self.detection_pub.publish(String(data="No solar panel detected."))
            self.get_logger().info("No detections found")
        else:
            # Get the detection with highest confidence
            boxes = results[0].boxes
            confidences = boxes.conf.cpu().numpy()
            best_idx = np.argmax(confidences)
            
            xy = boxes[best_idx].xyxy[0].cpu().numpy().astype(int)
            confidence = confidences[best_idx]
            
            cx = (xy[0] + xy[2]) // 2
            cy = (xy[1] + xy[3]) // 2
            
            msg_str = f"Panel detected. Center: ({cx},{cy}), Conf: {confidence:.2f}"
            self.detection_pub.publish(String(data=msg_str))
            self.get_logger().info(msg_str)

            # Send to Arduino with thread safety
            self.send_to_arduino(cx, cy)

        # Save annotated image (optional, can disable to save storage/CPU)
        if self.save_images:
            annotated = results[0].plot()
            timestamp = int(time.time())
            out_path = f"{self.output_folder}/inference_{timestamp}_{self.frame_count}.jpg"
            cv2.imwrite(out_path, annotated)
            self.get_logger().debug(f"Saved annotated image: {out_path}")

        self.run_inference = False

    def send_to_arduino(self, cx, cy):
        """Thread-safe Arduino communication"""
        if not self.serial_conn:
            return
            
        with self.serial_lock:
            try:
                command = f"{cx},{cy}\n"
                self.serial_conn.write(command.encode())
                self.serial_conn.flush()  # Ensure data is sent
                self.get_logger().debug(f"Sent to Arduino: {cx},{cy}")
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {e}")
                # Try to reconnect
                self.reconnect_serial()

    def reconnect_serial(self):
        """Attempt to reconnect serial if connection is lost"""
        try:
            if self.serial_conn:
                self.serial_conn.close()
            port = self.get_parameter('serial_port').get_parameter_value().string_value
            baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
            self.setup_serial(port, baud, max_retries=2)
        except Exception as e:
            self.get_logger().error(f"Serial reconnection failed: {e}")

    def read_angles(self):
        """Read gimbal angles from Arduino"""
        if not self.serial_conn:
            return
            
        with self.serial_lock:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:  # Only process non-empty lines
                        parts = line.split(',')
                        if len(parts) == 2:
                            try:
                                x_ang = float(parts[0])  # Allow float angles
                                y_ang = float(parts[1])
                                pt = Point(x=x_ang, y=y_ang, z=0.0)
                                self.angle_pub.publish(pt)
                                self.get_logger().debug(f"Published gimbal angles: {x_ang:.1f},{y_ang:.1f}")
                            except ValueError:
                                self.get_logger().debug(f"Invalid angle data: {line}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.reconnect_serial()

    def destroy_node(self):
        """Clean shutdown"""
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SolarPanelDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
