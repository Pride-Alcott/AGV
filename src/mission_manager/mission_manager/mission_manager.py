#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import datetime

# Message Imports
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, WaypointReached, RCIn
from geometry_msgs.msg import TwistStamped

# Service Imports
from mavros_msgs.srv import SetMode, CommandBool

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Parameters
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('waypoint_topic', '/mavros/mission/reached')
        self.declare_parameter('navsat_topic', '/mavros/global_position/global')
        self.declare_parameter('heading_topic', '/mavros/global_position/compass_hdg')
        self.declare_parameter('mission_start_topic', '/mission/start')
        self.declare_parameter('mission_end_topic', '/mission/end')
        self.declare_parameter('manual_control_channel', 5) # might need to change depending on my rc input for arming.
        self.declare_parameter('manual_control_threshold', 1500) # the limit to which the system operates under manual mode, input below this means manual intervention is required
        self.declare_parameter('auto_control_threshold', 1400) #the input conditions exceeds this threshols, the system autonomously manages without manual control

        # Retrieve parameters
        self.state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value
        self.navsat_topic = self.get_parameter('navsat_topic').get_parameter_value().string_value
        self.heading_topic = self.get_parameter('heading_topic').get_parameter_value().string_value
        self.mission_start_topic = self.get_parameter('mission_start_topic').get_parameter_value().string_value
        self.mission_end_topic = self.get_parameter('mission_end_topic').get_parameter_value().string_value
        self.manual_control_channel = self.get_parameter('manual_control_channel').get_parameter_value().integer_value
        self.manual_control_threshold = self.get_parameter('manual_control_threshold').get_parameter_value().integer_value
        self.auto_control_threshold = self.get_parameter('auto_control_threshold').get_parameter_value().integer_value

        # Subscribers
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.waypoint_sub = self.create_subscription(WaypointReached, self.waypoint_topic, self.waypoint_callback, 10)
        self.navsat_sub = self.create_subscription(NavSatFix, self.navsat_topic, self.navsat_callback, 10)
        self.heading_sub = self.create_subscription(Float64, self.heading_topic, self.heading_callback, 10)
        self.mission_start_sub = self.create_subscription(Bool, self.mission_start_topic, self.mission_start_callback, 10)
        self.mission_end_sub = self.create_subscription(Bool, self.mission_end_topic, self.mission_end_callback, 10)
        self.rc_in_sub = self.create_subscription(RCIn, '/mavros/rc/in', self.rc_in_callback, 10)

        # Publisher for trigger messages (for the solar panel detector)
        self.trigger_pub = self.create_publisher(Bool, '/detection_trigger', 10)

        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Internal state variables
        self.connected = False
        self.current_position = None
        self.current_heading = None
        self.mission_active = False
        self.manual_control_enabled = False

    def state_callback(self, msg: State):
        self.connected = msg.connected

    def navsat_callback(self, msg: NavSatFix):
        self.current_position = msg

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data

    def rc_in_callback(self, msg: RCIn):
        try:
            channel_value = msg.channels[self.manual_control_channel - 1]
        except IndexError:
            self.get_logger().warn(f"RC channel {self.manual_control_channel} not found in message. Check your RC configuration.")
            return

        if channel_value > self.manual_control_threshold and not self.manual_control_enabled:
            self.get_logger().info("Manual control enabled by RC override.")
            self.manual_control_enabled = True
            self.arm_vehicle(False)
            self.set_vehicle_mode("MANUAL")
        elif channel_value < self.auto_control_threshold and self.manual_control_enabled:
            self.get_logger().info("Automated control re-enabled by RC.")
            self.manual_control_enabled = False
            if self.mission_active:
                
                pass

    def mission_start_callback(self, msg: Bool):
        if self.manual_control_enabled:
            self.get_logger().info("Mission Start Ignored: Manual Control Active")
            return
        if msg.data and not self.mission_active:
            self.mission_active = True
            self.set_vehicle_mode("AUTO.MISSION")
            self.arm_vehicle(True)
            self.get_logger().info("Mission started.")

    def waypoint_callback(self, msg: WaypointReached):
        if self.manual_control_enabled:
            self.get_logger().info("Waypoint Reached Ignored: Manual Control Active")
            return
        self.get_logger().info(f"Waypoint {msg.wp_seq} reached.")
        # Publish a trigger to run solar panel detection at this waypoint.
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)

    def mission_end_callback(self, msg: Bool):
        if self.manual_control_enabled:
            self.get_logger().info("Mission End Ignored: Manual Control Active")
            return
        if msg.data and self.mission_active:
            self.mission_active = False
            self.set_vehicle_mode("HOLD")
            self.arm_vehicle(False)
            self.get_logger().info("Mission ended.")

    def set_vehicle_mode(self, mode: str):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            request = SetMode.Request()
            request.custom_mode = mode
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().mode_sent:
                self.get_logger().info(f"Vehicle mode set to {mode}.")
            else:
                self.get_logger().warn(f"Failed to set vehicle mode to {mode}.")
        else:
            self.get_logger().warn("SetMode service not available.")

    def arm_vehicle(self, arm: bool):
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            request = CommandBool.Request()
            request.value = arm
            future = self.arming_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().success:
                state = "armed" if arm else "disarmed"
                self.get_logger().info(f"Vehicle {state} successfully.")
            else:
                self.get_logger().warn(f"Failed to {'arm' if arm else 'disarm'} vehicle.")
        else:
            self.get_logger().warn("Arming service not available.")

def main(args=None):
    print("The mission managr node has been started")
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
