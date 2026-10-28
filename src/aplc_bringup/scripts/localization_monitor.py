#!/usr/bin/env python3
"""Live localization error against Gazebo ground truth, for the demo.

- Estimate: TF map -> base_footprint (RTAB-Map's map -> odom on top of the EKF's
  odom -> base_footprint).
- Truth: /ground_truth/odom (Gazebo world frame), converted into the map frame with the
  map_alignment.yaml recorded at the start of the mapping run.
- Publishes the position error (m) and heading error (deg) as std_msgs/Float64 on
  /localization_error/position and /localization_error/yaw, and two nav_msgs/Path in the
  map frame for RViz: /localization_monitor/path_estimated and
  /localization_monitor/path_ground_truth.
- Counts RTAB-Map corrections: every /info message with a loop closure id is a match
  with the map, and reports how far it moved map -> odom.
- Prints the error at print_rate Hz (default 2) and a summary on shutdown; optionally
  writes a CSV of the error over time.

Ground truth is only compared here; it is never sent to RTAB-Map or the EKF.
"""

import csv
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_stamped(frame, stamp, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class LocalizationMonitor(Node):

    def __init__(self):
        super().__init__('localization_monitor', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        alignment_file = self.declare_parameter('alignment_file', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        print_rate = self.declare_parameter('print_rate', 2.0).value
        self.path_step = self.declare_parameter('path_step', 0.05).value   # m between path poses
        csv_dir = self.declare_parameter('csv_dir', '').value

        al = yaml.safe_load(open(alignment_file))['map_alignment']
        # world = R(yaw) * map + t  =>  map = R(-yaw) * (world - t)
        self.al_x, self.al_y, self.al_yaw = al['x'], al['y'], al['yaw']
        self.get_logger().info(f'map alignment from {alignment_file}: x {self.al_x:.3f} y {self.al_y:.3f} '
                               f'yaw {math.degrees(self.al_yaw):.2f} deg')

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub_pos = self.create_publisher(Float64, '/localization_error/position', 10)
        self.pub_yaw = self.create_publisher(Float64, '/localization_error/yaw', 10)
        # Latched, so RViz (or anything) subscribing later gets the whole path at once.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_est = self.create_publisher(Path, '~/path_estimated', latched)
        self.pub_gt = self.create_publisher(Path, '~/path_ground_truth', latched)
        self.path_est = Path()
        self.path_gt = Path()
        self.path_est.header.frame_id = self.path_gt.header.frame_id = self.map_frame
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 20)
        self.create_subscription(Info, '/info', self.on_info, 20)

        self.errors = []            # (sim time, position error m, yaw error deg)
        self.latest = None
        self.corrections = 0
        self.last_correction = None
        self.last_map_odom = None
        self.create_timer(1.0 / print_rate, self.print_status)

        self.csv = None
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
            path = os.path.join(csv_dir, time.strftime('localization_error_%Y%m%d_%H%M%S.csv'))
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['sim_time_s', 'position_error_m', 'yaw_error_deg', 'est_x', 'est_y', 'est_yaw_deg',
                               'truth_x', 'truth_y', 'truth_yaw_deg', 'corrections'])
            self.get_logger().info(f'logging the error to {path}')

    def world_to_map(self, x, y, yaw):
        dx, dy = x - self.al_x, y - self.al_y
        c, s = math.cos(-self.al_yaw), math.sin(-self.al_yaw)
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - self.al_yaw)

    def on_truth(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, stamp, timeout=Duration(seconds=0.0))
        except Exception:
            return   # map frame not up yet, or no transform at this exact time
        p = msg.pose.pose
        gx, gy, gyaw = self.world_to_map(p.position.x, p.position.y, yaw_of(p.orientation))
        ex, ey, eyaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        pos_err = math.hypot(ex - gx, ey - gy)
        yaw_err = math.degrees(wrap(eyaw - gyaw))
        now_s = stamp.nanoseconds * 1e-9
        self.errors.append((now_s, pos_err, yaw_err))
        self.latest = (now_s, pos_err, yaw_err)
        self.pub_pos.publish(Float64(data=pos_err))
        self.pub_yaw.publish(Float64(data=yaw_err))
        self.append_path(self.path_est, self.pub_est, msg.header.stamp, ex, ey, eyaw)
        self.append_path(self.path_gt, self.pub_gt, msg.header.stamp, gx, gy, gyaw)
        if self.csv:
            self.csv.writerow([f'{now_s:.3f}', f'{pos_err:.4f}', f'{yaw_err:.3f}', f'{ex:.4f}', f'{ey:.4f}',
                               f'{math.degrees(eyaw):.3f}', f'{gx:.4f}', f'{gy:.4f}', f'{math.degrees(gyaw):.3f}',
                               self.corrections])

    def append_path(self, path, pub, stamp, x, y, yaw):
        if path.poses:
            last = path.poses[-1].pose.position
            if math.hypot(x - last.x, y - last.y) < self.path_step:
                return
        path.poses.append(pose_stamped(self.map_frame, stamp, x, y, yaw))
        path.header.stamp = stamp
        pub.publish(path)

    def on_info(self, msg):
        if msg.loop_closure_id <= 0:
            return
        self.corrections += 1
        self.last_correction = self.get_clock().now().nanoseconds * 1e-9
        jump = ''
        try:
            t = self.tf.lookup_transform(self.map_frame, 'odom', Time(), timeout=Duration(seconds=0.0))
            cur = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
            if self.last_map_odom is not None:
                d = math.hypot(cur[0] - self.last_map_odom[0], cur[1] - self.last_map_odom[1])
                dyaw = math.degrees(wrap(cur[2] - self.last_map_odom[2]))
                jump = f', map -> odom moved {d * 100:.1f} cm / {dyaw:+.2f} deg since the last one'
            self.last_map_odom = cur
        except Exception:
            pass
        self.get_logger().info(f'correction #{self.corrections}: matched map node {msg.loop_closure_id}{jump}')

    def print_status(self):
        # Republish the paths too, so a display that missed an update catches up.
        if self.path_est.poses:
            self.pub_est.publish(self.path_est)
            self.pub_gt.publish(self.path_gt)
        if self.latest is None:
            self.get_logger().info('waiting for map -> base_footprint and /ground_truth/odom ...')
            return
        _, pos, yaw = self.latest
        ago = ('none yet' if self.last_correction is None else
               f'last {self.get_clock().now().nanoseconds * 1e-9 - self.last_correction:.1f} s ago')
        self.get_logger().info(f'error: position {pos:.3f} m, yaw {yaw:+.2f} deg | corrections {self.corrections} ({ago})')

    def summary(self):
        if not self.errors:
            return 'no error samples'
        pos = [e[1] for e in self.errors]
        yaw = [abs(e[2]) for e in self.errors]
        return (f'summary over {self.errors[-1][0] - self.errors[0][0]:.0f} s: position error mean '
                f'{sum(pos) / len(pos):.3f} m, max {max(pos):.3f} m, final {pos[-1]:.3f} m; yaw error mean '
                f'{sum(yaw) / len(yaw):.2f} deg, max {max(yaw):.2f} deg, final {self.errors[-1][2]:+.2f} deg; '
                f'corrections {self.corrections}')


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'[localization_monitor] {node.summary()}', flush=True)
        if node.csv:
            node.csv_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
