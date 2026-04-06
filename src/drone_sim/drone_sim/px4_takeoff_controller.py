#!/usr/bin/env python3
"""
PX4 Drone Takeoff Controller using ROS2

This node sends commands to PX4 autopilot to:
1. Arm the drone
2. Switch to OFFBOARD mode
3. Takeoff to specified altitude
4. Hold position

Topics published:
- /fmu/in/offboard_control_mode
- /fmu/in/trajectory_setpoint
- /fmu/in/vehicle_command

Topics subscribed:
- /fmu/out/vehicle_status
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus


class PX4TakeoffController(Node):
    """ROS2 node to control PX4 drone takeoff"""

    def __init__(self):
        super().__init__('px4_takeoff_controller')
        
        # QoS profile for PX4 topics (Best Effort, Transient Local)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # Subscriber
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.offboard_setpoint_counter = 0
        self.takeoff_altitude = 5.0  # meters
        
        # Timer for control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('PX4 Takeoff Controller initialized')
        self.get_logger().info(f'Target takeoff altitude: {self.takeoff_altitude}m')

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def control_loop(self):
        """Main control loop (50 Hz)"""
        
        # Publish offboard control mode
        self.publish_offboard_control_mode()
        
        # Publish trajectory setpoint
        self.publish_trajectory_setpoint()
        
        # Handle state transitions
        if self.offboard_setpoint_counter == 10:
            # After 10 cycles, engage offboard mode
            self.engage_offboard_mode()
            self.get_logger().info('Engaging OFFBOARD mode...')
            
        if self.offboard_setpoint_counter == 20:
            # After 20 cycles, arm the vehicle
            self.arm()
            self.get_logger().info('Arming vehicle...')
        
        # Increment counter (will stop at max int)
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        """Publish offboard control mode (position control)"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint (target position)"""
        msg = TrajectorySetpoint()
        
        # Position setpoint (NED frame: North-East-Down)
        # Note: Z is negative for going UP in NED frame
        msg.position = [0.0, 0.0, -self.takeoff_altitude]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = 0.0  # 0 degrees (North)
        msg.yawspeed = float('nan')
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def engage_offboard_mode(self):
        """Send command to engage OFFBOARD mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )

    def arm(self):
        """Send command to arm the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0  # 1 to arm, 0 to disarm
        )

    def disarm(self):
        """Send command to disarm the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0  # 1 to arm, 0 to disarm
        )

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = PX4TakeoffController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
