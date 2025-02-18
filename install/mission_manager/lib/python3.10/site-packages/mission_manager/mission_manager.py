#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import datetime

# Message Imports
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, WaypointReached, OverrideRCIn
from geometry_msgs.msg import TwistStamped

# Service Imports
from mavros_msgs.srv import SetMode, CommandBool

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Declare parameters for topic names.
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('waypoint_topic', '/mavros/mission/reached')
        self.declare_parameter('navsat_topic', '/mavros/global_position/navsat_fix')
        self.declare_parameter('heading_topic', '/mavros/global_position/compass_hdg')
        self.declare_parameter('mission_start_topic', '/mission_start')
        self.declare_parameter('mission_end_topic', '/mission_end')
        self.declare_parameter('log_topic', '/mission_log')
        self.declare_parameter('velocity_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')

        # Retrieve parameter values.
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value
        navsat_topic = self.get_parameter('navsat_topic').get_parameter_value().string_value
        heading_topic = self.get_parameter('heading_topic').get_parameter_value().string_value
        mission_start_topic = self.get_parameter('mission_start_topic').get_parameter_value().string_value
        mission_end_topic = self.get_parameter('mission_end_topic').get_parameter_value().string_value
        log_topic = self.get_parameter('log_topic').get_parameter_value().string_value
        velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        rc_override_topic = self.get_parameter('rc_override_topic').get_parameter_value().string_value

        # Subscribers for telemetry and mission flow.
        self.state_sub = self.create_subscription(State, state_topic, self.state_callback, 10)
        self.waypoint_sub = self.create_subscription(WaypointReached, waypoint_topic, self.waypoint_callback, 10)
        self.navsat_sub = self.create_subscription(NavSatFix, navsat_topic, self.navsat_callback, 10)
        self.heading_sub = self.create_subscription(Float64, heading_topic, self.heading_callback, 10)
        self.mission_start_sub = self.create_subscription(Bool, mission_start_topic, self.mission_start_callback, 10)
        self.mission_end_sub = self.create_subscription(Bool, mission_end_topic, self.mission_end_callback, 10)

        # Publisher for mission logs.
        self.log_pub = self.create_publisher(String, log_topic, 10)
        # Publishers for command messages.
        self.velocity_pub = self.create_publisher(TwistStamped, velocity_topic, 10)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, rc_override_topic, 10)

        # Service clients for mode change and arming.
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Internal state variables.
        self.connected = False
        self.current_position = None  # NavSatFix message
        self.current_heading = None   # Float64 value
        self.mission_active = False

    def state_callback(self, msg: State):
        self.connected = msg.connected
        if self.connected:
            self.get_logger().info(f"Heartbeat received from Pixhawk. Mode: {msg.mode}, Armed: {msg.armed}")
        else:
            self.get_logger().warn("Pixhawk not connected.")

    def navsat_callback(self, msg: NavSatFix):
        self.current_position = msg

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data

    def mission_start_callback(self, msg: Bool):
        if msg.data:
            self.mission_active = True
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            lat = self.current_position.latitude if self.current_position else "N/A"
            lon = self.current_position.longitude if self.current_position else "N/A"
            heading = self.current_heading if self.current_heading is not None else "N/A"
            log_str = f"Mission Start at {timestamp} | Lat: {lat}, Lon: {lon}, Heading: {heading}"
            self.get_logger().info(log_str)
            self.log_pub.publish(String(data=log_str))
            # Call services to arm the vehicle and change flight mode.
            self.arm_vehicle(True)
            self.set_vehicle_mode("GUIDED_NOGPS")
        else:
            self.get_logger().info("Mission start signal received as False.")

    def waypoint_callback(self, msg: WaypointReached):
        if self.mission_active:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            lat = self.current_position.latitude if self.current_position else "N/A"
            lon = self.current_position.longitude if self.current_position else "N/A"
            heading = self.current_heading if self.current_heading is not None else "N/A"
            log_str = f"Waypoint {msg.wp_seq} reached at {timestamp} | Lat: {lat}, Lon: {lon}, Heading: {heading}"
            self.get_logger().info(log_str)
            self.log_pub.publish(String(data=log_str))
        else:
            self.get_logger().warn("Waypoint reached, but mission is not active.")

    def mission_end_callback(self, msg: Bool):
        if msg.data:
            self.mission_active = False
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            lat = self.current_position.latitude if self.current_position else "N/A"
            lon = self.current_position.longitude if self.current_position else "N/A"
            heading = self.current_heading if self.current_heading is not None else "N/A"
            log_str = f"Mission End at {timestamp} | Lat: {lat}, Lon: {lon}, Heading: {heading}"
            self.get_logger().info(log_str)
            self.log_pub.publish(String(data=log_str))
            # Disarm at mission end.
            self.arm_vehicle(False)
        else:
            self.get_logger().info("Mission end signal received as False.")

    def set_vehicle_mode(self, mode: str):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set mode service not available.")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.get_logger().info(f"Changing mode to {mode}")
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Mode change response: {future.result().mode_sent}")
        else:
            self.get_logger().error("Failed to change mode.")

    def arm_vehicle(self, arm: bool):
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available.")
            return
        req = CommandBool.Request()
        req.value = arm
        self.get_logger().info("Sending arming command" if arm else "Sending disarming command")
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Arming response: {future.result().success}")
        else:
            self.get_logger().error("Failed to change arming state.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

