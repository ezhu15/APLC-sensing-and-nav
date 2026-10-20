#!/usr/bin/env python3
"""Simulated quadrature wheel encoders.

Gazebo's /joint_states are exact wheel angles and velocities. A real encoder only
reports whole ticks, and a motor controller estimates speed from how many ticks
arrived in a time window. This node turns /joint_states into that:

  ticks    = floor(angle * ticks_per_rev / 2 pi + noise),  noise ~ N(0, tick_noise_stddev)
  position = ticks * 2 pi / ticks_per_rev                  (rad, a whole number of ticks)
  velocity = (ticks_now - ticks_window_ago) * 2 pi / ticks_per_rev / window   (rad/s)

and publishes sensor_msgs/JointState on /wheel_encoders with the source timestamps.
The noise models edge jitter: near a tick boundary the count flickers by one.
"""

import collections
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelEncoders(Node):

    def __init__(self):
        super().__init__('wheel_encoders')
        self.tpr = self.declare_parameter('ticks_per_rev', 1024).value
        self.noise = self.declare_parameter('tick_noise_stddev', 0.25).value
        self.window = self.declare_parameter('velocity_window', 0.1).value
        self.joints = list(self.declare_parameter('joints', [
            'front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint',
        ]).value)
        if self.tpr <= 0 or self.window <= 0:
            raise ValueError('ticks_per_rev and velocity_window must be positive')
        self.rad_per_tick = 2.0 * math.pi / self.tpr
        self.history = collections.deque()   # (stamp_s, [ticks per joint])
        self.pub = self.create_publisher(JointState, 'wheel_encoders', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.get_logger().info(
            f'{self.tpr} ticks/rev ({math.degrees(self.rad_per_tick):.3f} deg/tick), '
            f'noise {self.noise} ticks, velocity window {self.window} s')

    def on_joint_states(self, msg):
        angle = dict(zip(msg.name, msg.position))
        if not all(j in angle for j in self.joints):
            return
        ticks = [math.floor(angle[j] / self.rad_per_tick + random.gauss(0.0, self.noise))
                 for j in self.joints]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.history.append((stamp, ticks))
        while len(self.history) > 1 and stamp - self.history[1][0] >= self.window:
            self.history.popleft()
        old_stamp, old_ticks = self.history[0]
        dt = stamp - old_stamp

        out = JointState()
        out.header = msg.header
        out.name = self.joints
        out.position = [t * self.rad_per_tick for t in ticks]
        out.velocity = ([(t - o) * self.rad_per_tick / dt for t, o in zip(ticks, old_ticks)]
                        if dt > 0 else [0.0] * len(ticks))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = WheelEncoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
