#!/usr/bin/env python3
"""Stamp Gazebo GUI velocity commands with simulation time."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class StampCmdVel(Node):
    def __init__(self):
        super().__init__('stamp_cmd_vel')
        self.publisher = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 1)
        self.subscription = self.create_subscription(
            Twist, '/gui/cmd_vel', self.on_cmd_vel, 1)

    def on_cmd_vel(self, msg):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = StampCmdVel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()