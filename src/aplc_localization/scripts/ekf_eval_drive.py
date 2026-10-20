#!/usr/bin/env python3
"""Drive a repeatable EKF evaluation motion by publishing TwistStamped commands.

Tests (simulation time throughout):
  stationary  rest for --duration seconds
  square      2 m square (left turns) then a 360 deg in-place spin
  replay      re-publish the cmd_vel commands from a rosbag with their original timing

Every test waits until wheel odometry, the EKF and ground truth publish, holds the robot
at rest for --lead-in seconds, runs, then rests for --settle seconds so it stops.

The square is open loop. Its turns are timed with --turn-scale (default 1.49, the
measured in-place slip ratio, docs/sensors/wheel_encoders.md) so the true turns are close
to 90 deg; this uses no ground truth.
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

CMD_TOPIC = '/diff_drive_controller/cmd_vel'
READY_TOPICS = ['/diff_drive_controller/odom', '/odometry/filtered', '/ground_truth/odom']


class Driver(Node):

    def __init__(self):
        super().__init__('ekf_eval_drive', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.seen = set()
        for topic in READY_TOPICS:
            self.create_subscription(Odometry, topic, lambda _m, t=topic: self.seen.add(t), 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_ready(self, timeout=120.0):
        while self.get_clock().now().nanoseconds == 0 or len(self.seen) < len(READY_TOPICS):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('all inputs publishing')

    def send(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.pub.publish(msg)

    def hold(self, seconds, vx=0.0, wz=0.0, rate=50.0):
        """Command a constant velocity for a span of simulation time."""
        end = self.now() + seconds
        next_send = 0.0
        while self.now() < end:
            if self.now() >= next_send:
                self.send(vx, wz)
                next_send = self.now() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.002)

    def square(self, side, speed, turn_rate, turn_scale):
        # The acceleration limit (controllers.yaml) loses distance while speeding up and
        # gives the same back while slowing down after the command ends, so side / speed.
        drive_time = side / speed
        turn_time = turn_scale * (math.pi / 2) / turn_rate
        for _ in range(4):
            self.hold(drive_time, speed, 0.0)
            self.hold(1.0)
            self.hold(turn_time, 0.0, turn_rate)
            self.hold(1.0)
        self.hold(turn_scale * 2 * math.pi / turn_rate, 0.0, turn_rate)

    def replay(self, bag_path):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag_path),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[CMD_TOPIC]))
        cmds = []
        while reader.has_next():
            _topic, data, t_ns = reader.read_next()
            msg = deserialize_message(data, TwistStamped)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            cmds.append((stamp if stamp > 0 else t_ns * 1e-9, msg.twist.linear.x, msg.twist.angular.z))
        if not cmds:
            raise RuntimeError(f'no {CMD_TOPIC} messages in {bag_path}')
        t_first = cmds[0][0]
        start = self.now()
        self.get_logger().info(f'replaying {len(cmds)} commands over {cmds[-1][0] - t_first:.1f} s')
        for t, vx, wz in cmds:
            while self.now() - start < t - t_first:
                rclpy.spin_once(self, timeout_sec=0.001)
            self.send(vx, wz)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=['stationary', 'square', 'replay'])
    ap.add_argument('--duration', type=float, default=30.0, help='stationary: seconds at rest')
    ap.add_argument('--side', type=float, default=2.0)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--turn-rate', type=float, default=0.5)
    ap.add_argument('--turn-scale', type=float, default=1.49)
    ap.add_argument('--bag', help='replay: rosbag with ' + CMD_TOPIC)
    ap.add_argument('--lead-in', type=float, default=2.0)
    ap.add_argument('--settle', type=float, default=3.0)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Driver()
    try:
        node.wait_ready()
        node.hold(args.lead_in)
        if args.test == 'stationary':
            node.hold(args.duration)
        elif args.test == 'square':
            node.square(args.side, args.speed, args.turn_rate, args.turn_scale)
        else:
            if not args.bag:
                ap.error('replay needs --bag')
            node.replay(args.bag)
        node.hold(args.settle)
        node.get_logger().info('done')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
