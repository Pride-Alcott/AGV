#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import base64
import threading
import time
from pymavlink import mavutil
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
import queue
import select

class NTRIPCorrectionNode(Node):
    def __init__(self):
        super().__init__('ntrip_correction_node')
        
        # Declare and retrieve parameters with Docker-friendly defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('caster_host', 'www.rtk2go.com'),
                ('caster_port', 2101),
                ('mountpoint', 'Renewables'),
                ('username', 's229701698-at-mandela.ac.za'),
                ('password', 'mukandagumbo'),
                ('retry_interval', 5),
                ('pixhawk_port', '/dev/pixhawk0'),  # Your udev rule
                ('pixhawk_baud', 57600),
                ('mission_start_topic', '/mission_start'),
                ('gps_status_topic', '/gps_status'),
                ('connection_timeout', 10),
                ('data_timeout', 3),
                ('max_reconnect_attempts', 5)
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
        gps_status_topic = self.get_parameter('gps_status_topic').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        self.data_timeout = self.get_parameter('data_timeout').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value

        # ROS subscribers and publishers
        self.mission_start_sub = self.create_subscription(Bool, mission_start_topic, self.mission_callback, 10)
        self.gps_status_pub = self.create_publisher(String, gps_status_topic, 10)
        self.gps_position_pub = self.create_publisher(Point, '/gps_position', 10)
        
        # State management
        self.mission_active = False
        self.connection_active = False
        self.reconnect_count = 0
        self.last_data_time = time.time()
        self.bytes_received = 0
        self.rtcm_messages_sent = 0

        # Initialize MAVLink connection to Pixhawk
        self.pixhawk_conn = None
        self.mavlink_lock = threading.Lock()
        self.connect_pixhawk()

        # Sequence ID for RTCM messages (5 bits: 0-31)
        self.sequence_id = 0

        # Data queue for buffering RTCM data
        self.rtcm_queue = queue.Queue(maxsize=100)

        # Start threads
        self.ntrip_thread = threading.Thread(target=self.ntrip_worker, daemon=True)
        self.mavlink_thread = threading.Thread(target=self.mavlink_worker, daemon=True)
        self.status_thread = threading.Thread(target=self.status_worker, daemon=True)
        
        self.ntrip_thread.start()
        self.mavlink_thread.start()
        self.status_thread.start()

        # Create timer for periodic GPS position reading
        self.create_timer(1.0, self.read_gps_position)

    def connect_pixhawk(self, max_retries=3):
        """Attempt to connect to Pixhawk via MAVLink with retry logic."""
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Connecting to Pixhawk on {self.pixhawk_port} at {self.pixhawk_baud} baud... (attempt {attempt + 1})")
                self.pixhawk_conn = mavutil.mavlink_connection(
                    self.pixhawk_port, 
                    baud=self.pixhawk_baud,
                    timeout=5
                )
                # Wait for heartbeat to confirm connection
                self.pixhawk_conn.wait_heartbeat(timeout=10)
                self.get_logger().info("Connected to Pixhawk via MAVLink.")
                return True
            except Exception as e:
                self.get_logger().warn(f"Pixhawk connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(2)
        
        self.get_logger().error("Failed to connect to Pixhawk after all retries")
        self.pixhawk_conn = None
        return False

    def mission_callback(self, msg: Bool):
        """Toggle NTRIP client based on mission start/stop signals."""
        self.mission_active = msg.data
        status = "active" if self.mission_active else "inactive"
        self.get_logger().info(f"Mission is now {status}. NTRIP client will follow.")
        
        if not self.mission_active:
            self.connection_active = False
            # Clear the queue when mission stops
            while not self.rtcm_queue.empty():
                try:
                    self.rtcm_queue.get_nowait()
                except queue.Empty:
                    break

    def send_mavlink_rtcm(self, data_chunk: bytes):
        """Send RTCM data via MAVLink GPS_RTCM_DATA messages with fragmentation handling."""
        if not self.pixhawk_conn:
            return False

        with self.mavlink_lock:
            try:
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

                    msg = self.pixhawk_conn.mav.gps_rtcm_data_encode(
                        flags=flags,
                        len=part_len,
                        data=list(part_padded)
                    )
                    self.pixhawk_conn.mav.send(msg)

                self.rtcm_messages_sent += 1
                self.get_logger().debug(f"Sent RTCM message {self.rtcm_messages_sent} ({len(data_chunk)} bytes, {num_parts} fragments)")
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to send RTCM data: {e}")
                return False

    def ntrip_worker(self):
        """NTRIP connection worker thread."""
        while rclpy.ok():
            if not self.mission_active:
                self.connection_active = False
                time.sleep(1)
                continue

            try:
                self.get_logger().info(f"Connecting to NTRIP caster: {self.caster_host}:{self.caster_port}")
                
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(self.connection_timeout)
                    s.connect((self.caster_host, self.caster_port))
                    
                    # Build and send the NTRIP request
                    credentials = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
                    request = (
                        f"GET /{self.mountpoint} HTTP/1.0\r\n"
                        f"User-Agent: ROS2-AGV-NTRIP-Client/1.0\r\n"
                        f"Authorization: Basic {credentials}\r\n"
                        f"Connection: close\r\n\r\n"
                    )
                    s.sendall(request.encode())

                    # Check the caster's response
                    response = s.recv(1024)
                    if b"200 OK" not in response:
                        self.get_logger().error(f"NTRIP caster response: {response.decode().strip()}")
                        self.reconnect_count += 1
                        time.sleep(self.retry_interval)
                        continue

                    self.get_logger().info("Connected to NTRIP caster. Streaming RTCM data...")
                    self.connection_active = True
                    self.reconnect_count = 0
                    self.last_data_time = time.time()

                    # Set socket to non-blocking for periodic checks
                    s.setblocking(0)
                    
                    while rclpy.ok() and self.mission_active:
                        # Use select to check for data with timeout
                        ready = select.select([s], [], [], 1.0)
                        
                        if ready[0]:
                            try:
                                data = s.recv(4096)
                                if not data:
                                    self.get_logger().warning("NTRIP connection closed by server.")
                                    break
                                
                                self.bytes_received += len(data)
                                self.last_data_time = time.time()
                                
                                # Queue data for MAVLink transmission
                                try:
                                    self.rtcm_queue.put_nowait(data)
                                except queue.Full:
                                    self.get_logger().warn("RTCM queue full, dropping data")
                                    
                            except socket.error as e:
                                if e.errno != socket.EAGAIN:  # EAGAIN is expected for non-blocking
                                    self.get_logger().error(f"Socket error: {e}")
                                    break
                        
                        # Check for data timeout
                        if time.time() - self.last_data_time > 30:  # 30 second timeout
                            self.get_logger().warning("No RTCM data received for 30 seconds")
                            break

            except Exception as e:
                self.get_logger().error(f"NTRIP connection error: {e}")
                self.reconnect_count += 1
                
            self.connection_active = False
            
            # Exponential backoff for reconnection
            backoff_time = min(self.retry_interval * (2 ** min(self.reconnect_count, 4)), 60)
            self.get_logger().info(f"Reconnecting in {backoff_time} seconds...")
            time.sleep(backoff_time)

    def mavlink_worker(self):
        """MAVLink transmission worker thread."""
        while rclpy.ok():
            try:
                # Wait for RTCM data with timeout
                data = self.rtcm_queue.get(timeout=1.0)
                if self.mission_active and self.pixhawk_conn:
                    self.send_mavlink_rtcm(data)
                self.rtcm_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"MAVLink worker error: {e}")

    def status_worker(self):
        """Periodic status reporting."""
        while rclpy.ok():
            try:
                if self.mission_active:
                    status = f"Mission: Active, NTRIP: {'Connected' if self.connection_active else 'Disconnected'}, "
                    status += f"Data: {self.bytes_received} bytes, Messages: {self.rtcm_messages_sent}, "
                    status += f"Reconnects: {self.reconnect_count}"
                else:
                    status = "Mission: Inactive"
                
                self.gps_status_pub.publish(String(data=status))
                self.get_logger().debug(status)
                
                # Reset counters periodically
                if time.time() % 300 < 10:  # Every 5 minutes
                    self.bytes_received = 0
                    self.rtcm_messages_sent = 0
                
                time.sleep(10)  # Status every 10 seconds
            except Exception as e:
                self.get_logger().error(f"Status worker error: {e}")

    def read_gps_position(self):
        """Read GPS position from Pixhawk and publish."""
        if not self.pixhawk_conn:
            return
            
        try:
            with self.mavlink_lock:
                # Request GPS position
                msg = self.pixhawk_conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
                if msg:
                    # Convert from 1e7 degrees to degrees
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0  # Convert mm to meters
                    
                    position = Point(x=lat, y=lon, z=alt)
                    self.gps_position_pub.publish(position)
                    self.get_logger().debug(f"GPS Position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")
        except Exception as e:
            self.get_logger().debug(f"GPS position read error: {e}")

    def destroy_node(self):
        """Clean shutdown."""
        self.mission_active = False
        self.connection_active = False
        
        if self.pixhawk_conn:
            self.pixhawk_conn.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NTRIPCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NTRIP correction node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
