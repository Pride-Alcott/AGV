import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node') #creating a publisher
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge() # a bridge to convert betwen ros image messages and  open cv
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(image_msg)

def main():
    rclpy.init()  # Initialize ROS 2
    node = ImageCaptureNode()  # Create the node
    rclpy.spin(node)  # Keep it running
    node.destroy_node()  # Cleanup when done
    rclpy.shutdown()  # Shutdown ROS 2

if __name__ == "__main__":
    main()

