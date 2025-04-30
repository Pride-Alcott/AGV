import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class GRBLReader(Node):
    def __init__(self):
        super().__init__('grbl_reader')
        
        # Connect to Arduino (adjust port)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # ROS publisher for GRBL position
        self.pub_position = self.create_publisher(String, '/grbl_position', 10)

        # Timer to poll GRBL every second
        self.create_timer(1.0, self.request_position)

    def request_position(self):
        self.serial_port.write(b'?\n')  # Send status request
        response = self.serial_port.readline().decode().strip()
        
        if response.startswith('<'):
            position = self.parse_position(response)
            self.pub_position.publish(String(data=position))

    def parse_position(self, response):
        try:
            fields = response.split('|')
            pos = fields[1].split(':')[1].split(',')
            x, y, z = pos[0], pos[1], pos[2]
            return f'X:{x} Y:{y} Z:{z}'
        except Exception as e:
            self.get_logger().error(f'Error parsing GRBL response: {e}')
            return 'Error'

def main(args=None):
    rclpy.init(args=args)
    node = GRBLReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
