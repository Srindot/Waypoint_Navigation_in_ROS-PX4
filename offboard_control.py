#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        
        super().__init__('offboard_control')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers (Inputs to PX4 do NOT usually have _v1)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Create subscribers (Outputs from PX4 DO have _v1 in your version)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', # <--- FIXED
            self.vehicle_local_position_callback, 
            qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status_v1',         # <--- FIXED
            self.vehicle_status_callback, 
            qos_profile)

        # Initialize variables to prevent startup crashes
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Set Waypoints
        self.waypoints = [[0.0, 0.0, -5.0], [30.0, 100.0, -5.0], [20.0, -30.0, -5.0], [0.0, 0.0, -5.0]]
        
        self.waypoint = self.waypoints[0]
        self.offboard_setpoint_counter = 0
        
        self.waypoint_change_counter = 0
        self.delay_at_waypoint_counter = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # Extract and print position data
        x = vehicle_local_position.x
        y = vehicle_local_position.y
        z = vehicle_local_position.z
        # self.get_logger().info(f"Vehicle Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = self.waypoint 
        msg.yaw = 0.0 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            
        self.change_waypoint()

    def change_waypoint(self):
        """Change the waypoint to a new position."""
        if self.check_pos():
            if self.waypoint_change_counter < len(self.waypoints)-1:
                self.waypoint_change_counter += 1
                self.waypoint = self.waypoints[self.waypoint_change_counter]
                self.get_logger().info(f"Waypoint Reached! Moving to: {self.waypoint}")
    
    def check_pos(self):
        tolerance = 0.5 # Increased slightly for robustness

        # Check if we have valid data first (avoid 0,0,0 glitches at startup)
        if self.vehicle_local_position.timestamp == 0:
            return False

        if (abs(self.vehicle_local_position.x - self.waypoint[0]) < tolerance and
                abs(self.vehicle_local_position.y - self.waypoint[1]) < tolerance and
                abs(self.vehicle_local_position.z - self.waypoint[2]) < tolerance):
            
            if self.delay_at_waypoint_counter < 3:
                self.delay_at_waypoint_counter += 1
                return False # Stay here
            else:
                self.delay_at_waypoint_counter = 0
                return True
        else:
            return False

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)