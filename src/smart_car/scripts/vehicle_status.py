#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smartcar_msgs.msg import Status  # Ensure correct import for your custom Status message
import time

class VehicleStatusPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_status_publisher')

        # Publisher for vehicle status
        self.vehicle_status_publisher = self.create_publisher(Status, '/smart_car/vehicle_status', 10)
        
        # Example car parameters
        self.steering_angle_rad = 0.0
        self.engine_speed_rpm = 1000  # Example RPM
        self.battery_voltage_mv = 12000  # Example value for battery voltage in millivolts
        self.battery_current_ma = 500    # Example value for battery current in milliamps
        self.battery_percentage = 80.0   # Example battery percentage

    def publish_vehicle_status(self):
        """Publishes vehicle status message."""
        msg = Status()
        msg.battery_voltage_mv = self.battery_voltage_mv
        msg.battery_current_ma = self.battery_current_ma
        msg.battery_percentage = self.battery_percentage
        msg.steering_angle_rad = self.steering_angle_rad
        msg.engine_speed_rpm = self.engine_speed_rpm
        
        # Publish the message
        self.vehicle_status_publisher.publish(msg)
       # self.get_logger().info(f"Vehicle status published: Battery Voltage: {msg.battery_voltage_mv}mV, Battery Current: {msg.battery_current_ma}mA, Battery: {msg.battery_percentage}%")

    def run(self):
        """Simulates periodic vehicle status publishing."""
        try:
            while rclpy.ok():
                # Here, you could update your status parameters dynamically if needed
                self.publish_vehicle_status()
                time.sleep(1)  # Simulate periodic publishing every second
        except KeyboardInterrupt:
            self.get_logger().info('Node stopped manually')

def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
