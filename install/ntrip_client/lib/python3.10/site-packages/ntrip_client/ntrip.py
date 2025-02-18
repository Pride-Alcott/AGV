#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import base64
import threading
import time

# Import pymavlink for MAVLink messaging
from pymavlink import mavutil
from std_msgs.msg import Bool

class NTRIPCorrectionNode(Node):
    def __init__(self):
        super().__init__('ntrip_correction_node')
        
        # Declare NTRIP parameters
        self.declare_parameter('caster_host', 'www.rtk2go.com')
        self.declare_parameter('caster_port', 2101)
        self.declare_parameter('mountpoint', 'Renewables')
        self.declare_parameter('username', 's229701698-at-mandela.ac.za')
        self.declare_parameter('password', 'mukandagumbo')
        self.declare_parameter('retry_interval', 5)  # seconds
        
        # Declare Pixhawk (MAVLink) output parameters
        self.declare_parameter('pixhawk_port', '/dev/ttyUSB0')
        self.declare_parameter('pixhawk_baud', 57600)
        
        # Declare mission trigger topic (to start/stop NTRIP corrections)
        self.declare_parameter('mission_start_topic', '/mission_start')
        mission_start_topic = self.get_parameter('mission_start_topic').get_parameter_value().string_value

        # Read parameters
        self.caster_host = self.get_parameter('caster_host').get_parameter_value().string_value
        self.caster_port = self.get_parameter('caster_port').get_parameter_value().integer_value
        self.mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.retry_interval = self.get_parameter('retry_interval').get_parameter_value().integer_value
        self.pixhawk_port = self.get_parameter('pixhawk_port').get_parameter_value().string_value
        self.pixhawk_baud = self.get_parameter('pixhawk_baud').get_parameter_value().integer_value

        # Subscribe to mission start trigger.
        self.mission_start_sub = self.create_subscription(Bool, mission_start_topic, self.mission_callback, 10)
        self.mission_active = False

        # Initialize MAVLink connection to Pixhawk.
        try:
            self.get_logger().info(f"Connecting to Pixhawk on {self.pixhawk_port} at {self.pixhawk_baud} baud...")
            self.pixhawk_conn = mavutil.mavlink_connection(self.pixhawk_port, baud=self.pixhawk_baud)
            self.get_logger().info("Connected to Pixhawk via MAVLink.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pixhawk: {e}")
            self.pixhawk_conn = None

        # Start a thread for the NTRIP connection; it will run when mission_active is True.
        self.thread = threading.Thread(target=self.connect_and_publish)
        self.thread.daemon = True
        self.thread.start()

    def mission_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Mission start signal received. Activating NTRIP client.")
            self.mission_active = True
        else:
            self.get_logger().info("Mission end signal received. Deactivating NTRIP client.")
            self.mission_active = False

    def send_mavlink_rtcm(self, data_chunk: bytes):
        """
        Splits data_chunk into segments (max 200 bytes), packs them into GPS_RTCM_DATA MAVLink messages, and sends them.
        """
        chunk_size = 200
        for i in range(0, len(data_chunk), chunk_size):
            segment = data_chunk[i:i+chunk_size]
            segment_length = len(segment)
            # Pad segment to 200 bytes if needed.
            if segment_length < chunk_size:
                segment_padded = segment + b'\x00' * (chunk_size - segment_length)
            else:
                segment_padded = segment
            segment_list = list(segment_padded)
            try:
                msg = self.pixhawk_conn.mav.gps_rtcm_data_encode(
                    flags=0,
                    len=segment_length,
                    data=segment_list
                )
                self.pixhawk_conn.mav.send(msg)
                self.get_logger().debug(f"Sent MAVLink RTCM message with {segment_length} bytes.")
            except Exception as e:
                self.get_logger().error(f"Failed to send MAVLink RTCM message: {e}")

    def connect_and_publish(self):
        """
        Continuously attempts to connect to the NTRIP caster. When connected and mission_active is True,
        it reads RTCM data and forwards it via MAVLink.
        """
        while rclpy.ok():
            if not self.mission_active:
                time.sleep(1)
                continue
            try:
                self.get_logger().info(f"Connecting to NTRIP caster at {self.caster_host}:{self.caster_port}")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.caster_host, self.caster_port))
                
                credentials = f"{self.username}:{self.password}"
                base64_credentials = base64.b64encode(credentials.encode('utf-8')).decode('utf-8')
                request = (f"GET /{self.mountpoint} HTTP/1.0\r\n"
                           f"User-Agent: NTRIP ROS2 Client\r\n"
                           f"Authorization: Basic {base64_credentials}\r\n\r\n")
                s.sendall(request.encode('utf-8'))
                
                response = s.recv(1024)
                if b"200 OK" not in response:
                    self.get_logger().error("Did not receive 200 OK from NTRIP caster")
                    s.close()
                    time.sleep(self.retry_interval)
                    continue

                self.get_logger().info("Connected to NTRIP caster; receiving RTCM data...")
                while rclpy.ok() and self.mission_active:
                    data = s.recv(4096)
                    if not data:
                        self.get_logger().warn("No data received, connection lost.")
                        break
                    if self.pixhawk_conn is not None:
                        self.send_mavlink_rtcm(data)
                    else:
                        self.get_logger().error("Pixhawk connection unavailable; cannot forward MAVLink messages.")
                s.close()
            except Exception as e:
                self.get_logger().error(f"Exception in NTRIP connection: {e}")
            time.sleep(self.retry_interval)

def main(args=None):
    rclpy.init(args=args)
    node = NTRIPCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


