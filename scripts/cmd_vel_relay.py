#!/usr/bin/env python3
"""Forward /cmd_vel (Nav2, teleop) to diff_drive_controller's Humble input topic."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.create_subscription(Twist, '/cmd_vel', self.pub.publish, 10)
        self.get_logger().info('Relaying /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped')


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
