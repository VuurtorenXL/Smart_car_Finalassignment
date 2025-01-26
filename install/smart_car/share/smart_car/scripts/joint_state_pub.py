#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from smartcar_msgs.msg import Status  # Assuming you have this custom message for vehicle status
from sensor_msgs.msg import JointState  # For joint state publishing

class VehicleStatusAndJointStatePublisher(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('vehicle_status_and_joint_state_publisher')

        # Vehicle parameters
        self.steering_angle_rad = 0.0
        self.engine_speed_rpm = 0.0
        self.battery_voltage_mv = 12000  # Example value in millivolts
        self.battery_current_ma = 500  # Example value in milliamps
        self.battery_percentage = 80.0  # Example value for battery percentage

        self.wheel_radius = 0.032  # Radius of the wheel in meters
        self.wheel_base = 0.2535   # Distance between front and back wheels in meters

        self.rate = 0.1  # Rate of publishing in seconds (every 0.1 seconds)

        # Publishers for vehicle status and joint states
        self.status_publisher = self.create_publisher(Status, '/smart_car/vehicle_status', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/smart_car/joint_states', 10)

        # Set up the timer for periodic publishing
        self.timer = self.create_timer(self.rate, self.publish_vehicle_status_and_joint_states)

    def steering_angle_callback(self, steering_angle_rad):
        """Manually set the steering angle in radians"""
        self.steering_angle_rad = steering_angle_rad

    def engine_speed_callback(self, engine_speed_rpm):
        """Manually set the engine speed in RPM"""
        self.engine_speed_rpm = engine_speed_rpm

    def battery_voltage_callback(self, battery_voltage_mv):
        """Manually set the battery voltage in millivolts"""
        self.battery_voltage_mv = battery_voltage_mv

    def battery_current_callback(self, battery_current_ma):
        """Manually set the battery current in milliamps"""
        self.battery_current_ma = battery_current_ma

    def battery_percentage_callback(self, battery_percentage):
        """Manually set the battery percentage"""
        self.battery_percentage = battery_percentage

    def publish_vehicle_status_and_joint_states(self):
        """Publish vehicle status and joint states"""
        # Publish Vehicle Status
        status_msg = Status()
        status_msg.battery_voltage_mv = self.battery_voltage_mv
        status_msg.battery_current_ma = self.battery_current_ma
        status_msg.battery_percentage = self.battery_percentage
        status_msg.steering_angle_rad = self.steering_angle_rad
        status_msg.engine_speed_rpm = self.engine_speed_rpm

        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"Vehicle Status Published: Battery Voltage={status_msg.battery_voltage_mv} mV, "
                               f"Battery Current={status_msg.battery_current_ma} mA, "
                               f"Battery Percentage={status_msg.battery_percentage} %, "
                               f"Steering Angle={status_msg.steering_angle_rad} rad, "
                               f"Engine Speed={status_msg.engine_speed_rpm} RPM")

        # Publish Joint States
        angular_velocity = self.engine_speed_rpm * (2 * math.pi / 60)  # Convert RPM to rad/s
        wheel_rotation_speed = angular_velocity / self.wheel_radius

        # Simulate joint state for four wheels (assuming the same speed for all wheels)
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['front_left_wheel', 'front_right_wheel', 'back_left_wheel', 'back_right_wheel']
        joint_state.position = [0.0, 0.0, 0.0, 0.0]  # No displacement, just rotating at the same speed
        joint_state.velocity = [wheel_rotation_speed, wheel_rotation_speed, wheel_rotation_speed, wheel_rotation_speed]
        joint_state.effort = [0.0, 0.0, 0.0, 0.0]  # No force applied

        self.joint_state_publisher.publish(joint_state)
        self.get_logger().info(f"Joint States Published: Wheel Speed={wheel_rotation_speed} rad/s")

    def run(self):
        """Run the node loop"""
        rclpy.spin(self)  # Keep the node running and publishing periodically

def main(args=None):
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    # Create and run the vehicle status and joint state publisher node
    node = VehicleStatusAndJointStatePublisher()
    node.run()

    # Shutdown ROS2 once the node is finished
    rclpy.shutdown()

if __name__ == '__main__':
    main()
