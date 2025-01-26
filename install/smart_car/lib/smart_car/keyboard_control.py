#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Keyboard control initialized. Use WASD to move, Q to quit.")

    def get_key(self):
        """Reads a single key press from the keyboard."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()
            if key == 'w':  # Forward
                twist.linear.x = 0.5
                twist.angular.z = 0.0
            elif key == 's':  # Backward
                twist.linear.x = -0.5
                twist.angular.z = 0.0
            elif key == 'a':  # Turn left
                twist.linear.x = 0.0
                twist.angular.z = 1.0
            elif key == 'd':  # Turn right
                twist.linear.x = 0.0
                twist.angular.z = -1.0
            elif key == 'q':  # Quit
                self.get_logger().info("Exiting keyboard control.")
                break
            else:  # Stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
