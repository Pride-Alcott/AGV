#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool

class MavrosServices(Node):
    def __init__(self):
        super().__init__('mavros_services')
        self.create_service(SetMode, '/mavros/set_mode', self.handle_set_mode)
        self.create_service(CommandBool, '/mavros/cmd/arming', self.handle_arming)
        self.get_logger().info("MAVROS service servers are up.")

    def handle_set_mode(self, request, response):
        response.mode_sent = True  # Simulate successful mode change
        self.get_logger().info(f"SetMode called with mode: {request.custom_mode}")
        return response

    def handle_arming(self, request, response):
        response.success = True  # Simulate successful arming/disarming
        state = "arming" if request.value else "disarming"
        self.get_logger().info(f"Arming service called: {state} vehicle")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MavrosServices()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

