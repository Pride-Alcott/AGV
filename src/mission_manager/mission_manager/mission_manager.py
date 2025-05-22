#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import datetime
import time
import threading

from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, WaypointReached
from geometry_msgs.msg import TwistStamped, Point

# Service Imports
from mavros_msgs.srv import SetMode, CommandBool

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # QoS Profile for MAVROS topics (Best-Effort)
        self.mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Parameters with Docker-friendly defaults
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('waypoint_topic', '/mavros/mission/reached')
        self.declare_parameter('navsat_topic', '/mavros/global_position/global')
        self.declare_parameter('heading_topic', '/mavros/global_position/compass_hdg')
        self.declare_parameter('mission_start_topic', '/mission/start')
        self.declare_parameter('mission_end_topic', '/mission/end')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('detection_timeout', 60.0)  # Timeout for detection at waypoint
        self.declare_parameter('waypoint_hold_time', 60.0)  # Time to hold at waypoint
        self.declare_parameter('auto_arm', True)  # Auto-arm when mission starts

        # Retrieve parameters
        self.state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value
        self.navsat_topic = self.get_parameter('navsat_topic').get_parameter_value().string_value
        self.heading_topic = self.get_parameter('heading_topic').get_parameter_value().string_value
        self.mission_start_topic = self.get_parameter('mission_start_topic').get_parameter_value().string_value
        self.mission_end_topic = self.get_parameter('mission_end_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.detection_timeout = self.get_parameter('detection_timeout').get_parameter_value().double_value
        self.waypoint_hold_time = self.get_parameter('waypoint_hold_time').get_parameter_value().double_value
        self.auto_arm = self.get_parameter('auto_arm').get_parameter_value().bool_value

        # Subscribers
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.waypoint_sub = self.create_subscription(WaypointReached, self.waypoint_topic, self.waypoint_callback, 10)
        
        # Use MAVROS QoS for sensor topics
        self.navsat_sub = self.create_subscription(NavSatFix, self.navsat_topic, self.navsat_callback, qos_profile=self.mavros_qos)
        self.heading_sub = self.create_subscription(Float64, self.heading_topic, self.heading_callback, qos_profile=self.mavros_qos)
        
        self.mission_start_sub = self.create_subscription(Bool, self.mission_start_topic, self.mission_start_callback, 10)
        self.mission_end_sub = self.create_subscription(Bool, self.mission_end_topic, self.mission_end_callback, 10)
        
        # Subscribe to detection results
        self.detection_sub = self.create_subscription(String, '/detections', self.detection_callback, 10)
        self.gps_status_sub = self.create_subscription(String, '/gps_status', self.gps_status_callback, 10)

        # Publishers
        self.trigger_pub = self.create_publisher(Bool, '/detection_trigger', 10)
        self.mission_start_pub = self.create_publisher(Bool, '/mission_start', 10)  # For NTRIP
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Service clients with connection verification
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Internal state variables
        self.connected = False
        self.armed = False
        self.current_mode = ""
        self.current_position = None
        self.current_heading = None
        self.mission_active = False
        self.current_waypoint = -1
        self.waypoint_start_time = None
        self.detection_received = False
        self.gps_rtk_status = "Unknown"
        
        # Thread safety
        self.state_lock = threading.Lock()

        # Status reporting timer
        self.create_timer(2.0, self.publish_status)
        
        # Waypoint timeout timer
        self.waypoint_timer = None

        self.get_logger().info("Mission Manager initialized")

    def state_callback(self, msg: State):
        with self.state_lock:
            self.connected = msg.connected
            self.armed = msg.armed
            self.current_mode = msg.mode
            
        if not self.connected:
            self.get_logger().warn("MAVROS not connected to flight controller")

    def navsat_callback(self, msg: NavSatFix):
        self.current_position = msg
        # Log GPS fix quality periodically
        if hasattr(msg, 'status') and msg.status.status >= 0:
            fix_type = ['No Fix', 'GPS Fix', 'DGPS Fix', 'RTK Float', 'RTK Fixed'][min(msg.status.status, 4)]
            self.get_logger().debug(f"GPS Status: {fix_type}")

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data

    def gps_status_callback(self, msg: String):
        self.gps_rtk_status = msg.data
        if "RTK Fixed" in msg.data:
            self.get_logger().debug("RTK Fixed - High precision GPS available")

    def detection_callback(self, msg: String):
        if self.mission_active and self.current_waypoint >= 0:
            self.detection_received = True
            self.get_logger().info(f"Detection result at waypoint {self.current_waypoint}: {msg.data}")

    def mission_start_callback(self, msg: Bool):
        if msg.data and not self.mission_active:
            self.start_mission()

    def mission_end_callback(self, msg: Bool):
        if msg.data and self.mission_active:
            self.end_mission()

    def start_mission(self):
        """Start the autonomous mission"""
        if not self.connected:
            self.get_logger().error("Cannot start mission - MAVROS not connected")
            return
            
        self.mission_active = True
        self.current_waypoint = -1
        self.get_logger().info("Starting autonomous mission...")
        
        # Notify NTRIP node to start RTK corrections
        ntrip_msg = Bool()
        ntrip_msg.data = True
        self.mission_start_pub.publish(ntrip_msg)
        
        # Set mission parameters and start
        if self.auto_arm:
            self.set_vehicle_mode("AUTO.MISSION")
            time.sleep(1)  # Allow mode to set
            self.arm_vehicle(True)
        else:
            self.get_logger().info("Auto-arm disabled. Please arm manually.")

    def end_mission(self):
        """End the autonomous mission"""
        if not self.mission_active:
            return
            
        self.mission_active = False
        self.get_logger().info("Ending autonomous mission...")
        
        # Stop NTRIP corrections
        ntrip_msg = Bool()
        ntrip_msg.data = False
        self.mission_start_pub.publish(ntrip_msg)
        
        # Set to hold mode and disarm
        self.set_vehicle_mode("HOLD")
        time.sleep(2)
        self.arm_vehicle(False)
        
        # Cancel any pending waypoint operations
        if self.waypoint_timer:
            self.waypoint_timer.cancel()

    def waypoint_callback(self, msg: WaypointReached):
        if not self.mission_active:
            return
            
        self.current_waypoint = msg.wp_seq
        self.waypoint_start_time = time.time()
        self.detection_received = False
        
        self.get_logger().info(f"Reached waypoint {msg.wp_seq} - Starting detection sequence")
        
        # Trigger detection
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        # Start waypoint timeout timer
        if self.waypoint_timer:
            self.waypoint_timer.cancel()
        self.waypoint_timer = threading.Timer(self.detection_timeout, self.waypoint_timeout)
        self.waypoint_timer.start()

    def waypoint_timeout(self):
        """Handle detection timeout at waypoint"""
        if self.mission_active and self.current_waypoint >= 0:
            if not self.detection_received:
                self.get_logger().warn(f"Detection timeout at waypoint {self.current_waypoint}")
            else:
                self.get_logger().info(f"Detection completed at waypoint {self.current_waypoint}")
        
        # Continue to next waypoint after hold time
        time.sleep(self.waypoint_hold_time)
        self.get_logger().info("Continuing to next waypoint...")

    def set_vehicle_mode(self, mode: str):
        """Set vehicle flight mode with retry logic"""
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetMode service not available")
            return False
            
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().mode_sent:
                self.get_logger().info(f"Vehicle mode set to: {mode}")
                return True
            else:
                self.get_logger().error(f"Failed to set vehicle mode to: {mode}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error setting vehicle mode: {e}")
            return False

    def arm_vehicle(self, arm: bool):
        """Arm or disarm vehicle with retry logic"""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return False
            
        request = CommandBool.Request()
        request.value = arm
        
        try:
            future = self.arming_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().success:
                state = "armed" if arm else "disarmed"
                self.get_logger().info(f"Vehicle {state} successfully")
                return True
            else:
                state = "arm" if arm else "disarm"
                self.get_logger().error(f"Failed to {state} vehicle")
                return False
        except Exception as e:
            self.get_logger().error(f"Error arming/disarming vehicle: {e}")
            return False

    def publish_status(self):
        """Publish comprehensive mission status"""
        with self.state_lock:
            status_parts = [
                f"Mission: {'Active' if self.mission_active else 'Inactive'}",
                f"Connected: {self.connected}",
                f"Armed: {self.armed}",
                f"Mode: {self.current_mode}",
                f"Waypoint: {self.current_waypoint if self.current_waypoint >= 0 else 'N/A'}",
            ]
        
        if self.current_position:
            status_parts.append(f"GPS: {self.current_position.latitude:.6f}, {self.current_position.longitude:.6f}")
        
        if self.current_heading is not None:
            status_parts.append(f"Heading: {self.current_heading:.1f}°")
            
        status_parts.append(f"RTK: {self.gps_rtk_status}")
        
        status_msg = String()
        status_msg.data = " | ".join(status_parts)
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean shutdown"""
        if self.mission_active:
            self.end_mission()
        
        if self.waypoint_timer:
            self.waypoint_timer.cancel()
            
        super().destroy_node()


def main(args=None):
    print("Mission Manager node starting...")
    rclpy.init(args=args)
    node = MissionManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Mission Manager...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
