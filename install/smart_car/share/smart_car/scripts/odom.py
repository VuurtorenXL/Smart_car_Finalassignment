#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np
from smartcar_msgs.msg import Status



def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class WheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('wheel_odom_publisher')

        # Subscriber to vehicle status topic
        self.subscription = self.create_subscription(
            Status,
            '/smart_car/vehicle_status',
            self.vehicle_status_callback,
            10
        )

        # Publisher for wheel odometry
        self.odom_publisher = self.create_publisher(Odometry, '/smart_car/wheel/odom', 10)

        # Set up TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initial pose and velocity
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.phi_prev = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Store last time for position calculations
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def vehicle_status_callback(self, msg: Status):
        # Update car's parameters from the Status message
        self.linear_velocity = msg.engine_speed_rpm * math.pi * 0.032 / 60  # Example conversion
        self.angular_velocity = (self.linear_velocity / 0.2535) * math.tan(msg.steering_angle_rad)

        # Current time and delta time calculation
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.last_time
        if dt > 0:
            self.update_pose(dt)

        self.last_time = current_time

    def update_pose(self, dt):
        # Update robot's pose based on velocity and steering
        self.phi_prev += self.angular_velocity * dt
        self.phi_prev = np.arctan2(np.sin(self.phi_prev), np.cos(self.phi_prev))
        self.x_prev += self.linear_velocity * math.cos(self.phi_prev) * dt
        self.y_prev += self.linear_velocity * math.sin(self.phi_prev) * dt

        # Publish odometry and transform
        self.publish_wheel_odometry(self.linear_velocity, self.angular_velocity)
        self.broadcast_odom_to_base_link_transform()

    def publish_wheel_odometry(self, linear_velocity, angular_velocity):
        # Publish wheel odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position and orientation
        odom_msg.pose.pose.position.x = self.x_prev
        odom_msg.pose.pose.position.y = self.y_prev

        # Convert the orientation from radians to quaternion
        q_ = quaternion_from_euler(0, 0, self.phi_prev)
        q = Quaternion()
        q.x = q_[0]
        q.y = q_[1]
        q.z = q_[2]
        q.w = q_[3]
        odom_msg.pose.pose.orientation = q

        # Set linear and angular velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

    def broadcast_odom_to_base_link_transform(self):
        # Broadcast transform between odom and base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # Set translation
        transform.transform.translation.x = self.x_prev
        transform.transform.translation.y = self.y_prev
        transform.transform.translation.z = 0.0

        # Set rotation
        q = Quaternion()
        q.x = np.sin(self.phi_prev / 2)
        q.y = 0.0
        q.z = 0.0
        q.w = np.cos(self.phi_prev / 2)
        transform.transform.rotation = q

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
