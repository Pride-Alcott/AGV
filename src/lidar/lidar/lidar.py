#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lidar.msg import ObstacleInfo

from sensor_msgs.msg import LaserScan
class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Declare parameters.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_info_topic', '/obstacle_info')
        self.declare_parameter('detection_radius', 0.6)  # 60 cm detection radius
        
        # Retrieve parameters.
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        obstacle_info_topic = self.get_parameter('obstacle_info_topic').get_parameter_value().string_value
        self.detection_radius = self.get_parameter('detection_radius').get_parameter_value().double_value

        # Subscriber to the LaserScan data.
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        # Publisher for obstacle information.
        self.obstacle_pub = self.create_publisher(ObstacleInfo, obstacle_info_topic, 10)
    
    def scan_callback(self, msg):
        # Initialize values.
        obstacle_detected = False
        min_distance = float('inf')
        min_angle = 0.0

        # Process each range measurement.
        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings.
            if distance < msg.range_min or distance > msg.range_max:
                continue

            if distance < self.detection_radius:
                obstacle_detected = True
                angle = msg.angle_min + i * msg.angle_increment
                # Save the closest obstacle.
                if distance < min_distance:
                    min_distance = distance
                    min_angle = angle

        # Create the obstacle information message.
        obstacle_msg = ObstacleInfo()
        if obstacle_detected:
            obstacle_msg.obstacle_detected = True
            obstacle_msg.distance = float(min_distance)
            obstacle_msg.angle = float(min_angle)
        else:
            obstacle_msg.obstacle_detected = False
            obstacle_msg.distance = 0.0
            obstacle_msg.angle = 0.0

        self.obstacle_pub.publish(obstacle_msg)
        self.get_logger().info(
            f"Obstacle Info - Detected: {obstacle_msg.obstacle_detected}, "
            f"Distance: {obstacle_msg.distance:.2f} m, Angle: {obstacle_msg.angle:.2f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

