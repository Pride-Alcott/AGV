import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')

        # Create a publisher for the image topic
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize the camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            return  # Exit initialization if the camera fails

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warn("Webcam is not available.")
            return

        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(image_msg)
            self.get_logger().info("Published an image frame.")
        else:
            self.get_logger().warn("Failed to capture image from webcam.")

    def destroy_node(self):
        self.get_logger().info("Shutting down ImageCaptureNode...")
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Released webcam resource.")
        super().destroy_node()


def main():
    rclpy.init()
    node = ImageCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

