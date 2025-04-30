#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import base64
import threading
import time
from pymavlink import mavutil
from std_msgs.msg import Bool

class NTRIPCorrectionNode(Node):
    def __init__(self):
        super().__init__('ntrip_correction_node')
        
        # Declare and retrieve parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('caster_host', 'www.rtk2go.com'),
                ('caster_port', 2101),
                ('mountpoint', 'Renewables'),
                ('username', 's229701698-at-mandela.ac.za'),
                ('password', 'mukandagumbo'),
                ('retry_interval', 5),
                ('pixhawk_port', '/dev/ttyUSB0'),
                ('pixhawk_baud', 57600),
                ('mission_start_topic', '/mission_start')
            ]
        )
        
        self.caster_host = self.get_parameter('caster_host').value
        self.caster_port = self.get_parameter('caster_port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.retry_interval = self.get_parameter('retry_interval').value
        self.pixhawk_port = self.get_parameter('pixhawk_port').value
        self.pixhawk_baud = self.get_parameter('pixhawk_baud').value
        mission_start_topic = self.get_parameter('mission_start_topic').value

        # Subscribe to mission start trigger
        self.mission_start_sub = self.create_subscription(Bool, mission_start_topic, self.mission_callback, 10)
        self.mission_active = False

        # Initialize MAVLink connection to Pixhawk
        self.pixhawk_conn = None
        self.connect_pixhawk()

        # Sequence ID for RTCM messages (5 bits: 0-31)
        self.sequence_id = 0

        # Start the NTRIP thread
        self.thread = threading.Thread(target=self.connect_and_publish)
        self.thread.daemon = True
        self.thread.start()

    def connect_pixhawk(self):
        """Attempt to connect to Pixhawk via MAVLink."""
        try:
            self.get_logger().info(f"Connecting to Pixhawk on {self.pixhawk_port} at {self.pixhawk_baud} baud...")
            self.pixhawk_conn = mavutil.mavlink_connection(self.pixhawk_port, baud=self.pixhawk_baud)
            self.get_logger().info("Connected to Pixhawk via MAVLink.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pixhawk: {e}")
            self.pixhawk_conn = None

    def mission_callback(self, msg: Bool):
        """Toggle NTRIP client based on mission start/stop signals."""
        self.mission_active = msg.data
        status = "active" if self.mission_active else "inactive"
        self.get_logger().info(f"NTRIP client is now {status}.")

    def send_mavlink_rtcm(self, data_chunk: bytes):
        """Send RTCM data via MAVLink GPS_RTCM_DATA messages with fragmentation handling."""
        if not self.pixhawk_conn:
            self.get_logger().error("Pixhawk connection not available.")
            return

        # Use 180 bytes per fragment (as per MAVLink RTCM specification)
        chunk_size = 180
        parts = [data_chunk[i:i+chunk_size] for i in range(0, len(data_chunk), chunk_size)]
        num_parts = len(parts)
        self.sequence_id = (self.sequence_id + 1) % 32  # 5-bit sequence ID

        for fragment_id, part in enumerate(parts):
            # If more than one part, mark as fragmented (bit0 = 1)
            fragmented = 1 if num_parts > 1 else 0
            # Fragment ID (2 bits: 0-3)
            fragment_id_bits = fragment_id % 4
            # Combine flags: bit0: fragmentation flag, bits1-2: fragment ID, bits3-7: sequence ID
            flags = fragmented | (fragment_id_bits << 1) | (self.sequence_id << 3)
            part_len = len(part)
            part_padded = part.ljust(chunk_size, b'\x00')  # pad to exactly 180 bytes

            try:
                msg = self.pixhawk_conn.mav.gps_rtcm_data_encode(
                    flags=flags,
                    len=part_len,
                    data=list(part_padded)
                )
                self.pixhawk_conn.mav.send(msg)
                self.get_logger().debug(f"Sent RTCM part {fragment_id+1}/{num_parts} (seq: {self.sequence_id})")
            except Exception as e:
                self.get_logger().error(f"Failed to send RTCM data: {e}")

    def connect_and_publish(self):
        """Manage NTRIP connection and stream data to Pixhawk."""
        while rclpy.ok():
            if not self.mission_active:
                time.sleep(1)
                continue

            try:
                self.get_logger().info(f"Connecting to NTRIP caster: {self.caster_host}:{self.caster_port}")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(1.0)  # Allow periodic mission_active check
                    s.connect((self.caster_host, self.caster_port))
                    
                    # Build and send the NTRIP request
                    credentials = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
                    request = (
                        f"GET /{self.mountpoint} HTTP/1.0\r\n"
                        f"User-Agent: ROS2 NTRIP Client\r\n"
                        f"Authorization: Basic {credentials}\r\n\r\n"
                    )
                    s.sendall(request.encode())

                    # Check the caster's response
                    response = s.recv(1024)
                    if b"200 OK" not in response:
                        self.get_logger().error(f"NTRIP caster response: {response.decode().strip()}")
                        continue

                    self.get_logger().info("Connected to NTRIP caster. Streaming RTCM data...")
                    while rclpy.ok() and self.mission_active:
                        try:
                            data = s.recv(4096)
                            if not data:
                                self.get_logger().warning("NTRIP connection closed by server.")
                                break
                            self.send_mavlink_rtcm(data)
                        except socket.timeout:
                            continue  # check mission_active status again
            except Exception as e:
                self.get_logger().error(f"NTRIP connection error: {e}")
                time.sleep(self.retry_interval)

def main(args=None):
    rclpy.init(args=args)
    node = NTRIPCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()