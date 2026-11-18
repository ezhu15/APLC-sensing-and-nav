#!/usr/bin/env python3
"""Simulated Meta Quest headset: map-frame pose with realistic error, and map -> odom.

Stands in for aplc_scene_ros's quest_bridge while no headset is available. It reads
Gazebo's exact robot pose (/ground_truth/odom, map = Gazebo world), adds the error a
Quest's inside-out tracking has, and publishes what the real bridge publishes:

  /quest/pose     geometry_msgs/PoseStamped, frame map, headset pose
  /quest/status   std_msgs/String, JSON with "status": "live" (same key as quest_bridge)
  TF map -> quest_world (identity alignment) -> quest_headset

and, because Nav2 needs it, the localization transform

  TF map -> odom   (planar: x, y, yaw), computed so that
                   map -> odom -> base_footprint equals the Quest's estimate of the robot.

The EKF keeps publishing odom -> base_footprint (smooth, drifting); this node corrects
the drift through map -> odom, the same split as AMCL or slam_toolbox (REP 105).

Error model (all parameters, defaults in brackets):
  position_noise_std [0.004 m], yaw_noise_std [0.004 rad]: white noise per sample.
  drift_xy_rate [0.004 m/sqrt(s)], drift_yaw_rate [0.0015 rad/sqrt(s)]: random walk,
      clamped to drift_xy_max [0.10 m] and drift_yaw_max [0.035 rad]. This is the slow
      tracking drift between anchor re-alignments.
  realign_period [20 s]: every period the drift is pulled back toward zero (an anchor
      alignment), which shows up as a small jump in map -> odom. 0 disables it.
  dropout_period [0 s] / dropout_duration [1.0 s]: periodic tracking loss; map -> odom
      then freezes at its last value and status reads "pose_stale".
  seed [0]: random seed (0 = random each run).

The headset is rigidly mounted on the robot at mount_xyz / mount_rpy in base_footprint.
Ground truth is used only to synthesize the measurement, never published as an estimate.
"""
import json
import math
import random

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


# Quaternions as (x, y, z, w).
def q_mul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w * X + x * W + y * Z - z * Y, w * Y - x * Z + y * W + z * X,
            w * Z + x * Y - y * X + z * W, w * W - x * X - y * Y - z * Z)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rot(q, v):
    return q_mul(q_mul(q, (*v, 0.0)), q_conj(q))[:3]


def q_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def q_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class FakeQuest(Node):
    def __init__(self):
        super().__init__('fake_quest')
        defaults = {
            'rate': 30.0, 'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
            'mount_xyz': [0.15, 0.0, 1.0], 'mount_rpy': [0.0, 0.0, 0.0],
            'position_noise_std': 0.004, 'yaw_noise_std': 0.004,
            'drift_xy_rate': 0.004, 'drift_yaw_rate': 0.0015,
            'drift_xy_max': 0.10, 'drift_yaw_max': 0.035,
            'realign_period': 20.0, 'dropout_period': 0.0, 'dropout_duration': 1.0,
            'transform_tolerance': 0.2, 'seed': 0, 'publish_map_to_odom': True,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.p = {k: g(k) for k in defaults}
        self.rng = random.Random(self.p['seed'] or None)
        self.mount_t = tuple(float(v) for v in self.p['mount_xyz'])
        self.mount_q = q_rpy(*(float(v) for v in self.p['mount_rpy']))

        self.gt = None                       # latest ground-truth Odometry
        self.drift = [0.0, 0.0, 0.0]         # x, y, yaw random walk
        self.last_tick = None
        self.start = None
        self.last_realign = None
        self.map_odom = None                 # (x, y, yaw), last valid
        self.status = 'waiting_pose'
        self.sent = 0

        # odom -> base_footprint comes from the EKF's own output topic rather than a TF
        # listener: a Python TF buffer subscribed to all of /tf costs a quarter of a core.
        self.odom = None
        self.create_subscription(Odometry, 'odometry/filtered', self.on_odom, 10)
        self.tf = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'quest/pose', 10)
        self.status_pub = self.create_publisher(String, 'quest/status', 1)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 10)
        self.create_timer(1.0 / self.p['rate'], self.tick)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            f"Simulated Quest at {self.mount_t} in {self.p['base_frame']}; noise "
            f"{self.p['position_noise_std']} m, drift <= {self.p['drift_xy_max']} m, "
            f"realign every {self.p['realign_period']} s")

    def on_gt(self, msg):
        self.gt = msg

    def on_odom(self, msg):
        self.odom = msg

    def in_dropout(self, t):
        period = self.p['dropout_period']
        return period > 0 and (t - self.start) % period > period - self.p['dropout_duration']

    def step_drift(self, t):
        dt = 0.0 if self.last_tick is None else max(0.0, t - self.last_tick)
        self.last_tick = t
        s = math.sqrt(dt)
        for i, (rate, cap) in enumerate(((self.p['drift_xy_rate'], self.p['drift_xy_max']),) * 2
                                        + ((self.p['drift_yaw_rate'], self.p['drift_yaw_max']),)):
            self.drift[i] = max(-cap, min(cap, self.drift[i] + self.rng.gauss(0.0, rate * s)))
        period = self.p['realign_period']
        if period > 0 and t - self.last_realign >= period:
            # Anchor re-alignment: most of the accumulated drift disappears at once.
            self.drift = [d * 0.2 for d in self.drift]
            self.last_realign = t

    def tick(self):
        if self.gt is None:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.start is None:
            self.start = self.last_realign = t
        self.step_drift(t)
        if self.in_dropout(t):
            self.status = 'pose_stale'
            self.publish_map_odom(now)
            return
        self.status = 'live'

        # True headset pose in map = robot pose (base_footprint) composed with the mount.
        gp, gq = self.gt.pose.pose.position, self.gt.pose.pose.orientation
        base_t, base_q = (gp.x, gp.y, gp.z), (gq.x, gq.y, gq.z, gq.w)
        off = q_rot(base_q, self.mount_t)
        head_t = [base_t[i] + off[i] for i in range(3)]
        head_q = q_mul(base_q, self.mount_q)

        # Measured headset pose: drift (a slowly moving frame error) plus white noise.
        n = self.p['position_noise_std']
        head_t[0] += self.drift[0] + self.rng.gauss(0.0, n)
        head_t[1] += self.drift[1] + self.rng.gauss(0.0, n)
        head_t[2] += self.rng.gauss(0.0, n)
        head_q = q_mul(q_rpy(0.0, 0.0, self.drift[2] + self.rng.gauss(0.0, self.p['yaw_noise_std'])),
                       head_q)

        stamp = self.gt.header.stamp
        pose = PoseStamped()
        pose.header.frame_id = self.p['map_frame']
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = head_t
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = head_q
        self.pose_pub.publish(pose)
        self.sent += 1

        # Same TF chain as quest_bridge: map -> quest_world (alignment) -> quest_headset.
        self.tf.sendTransform([
            self.make_tf(self.p['map_frame'], 'quest_world', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), stamp),
            self.make_tf('quest_world', 'quest_headset', head_t, head_q, stamp),
        ])

        # Robot estimate from the headset: invert the mount.
        est_q = q_mul(head_q, q_conj(self.mount_q))
        off = q_rot(est_q, self.mount_t)
        est = (head_t[0] - off[0], head_t[1] - off[1], q_yaw(est_q))

        # map -> odom = estimate * inverse(odom -> base_footprint). The EKF's latest
        # estimate is at most one filter step (20 ms) older than the ground truth.
        if self.odom is None:
            self.get_logger().warn('No odometry/filtered yet (EKF)', throttle_duration_sec=5.0)
            return
        ot, oq = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        oyaw = q_yaw((oq.x, oq.y, oq.z, oq.w))
        yaw = est[2] - oyaw
        c, s = math.cos(yaw), math.sin(yaw)
        self.map_odom = (est[0] - (c * ot.x - s * ot.y), est[1] - (s * ot.x + c * ot.y), yaw)
        self.publish_map_odom(now)

    def publish_map_odom(self, now):
        if self.map_odom is None or not self.p['publish_map_to_odom']:
            return
        x, y, yaw = self.map_odom
        # Post-date like AMCL, so lookups at the latest sensor time never extrapolate.
        stamp = (now + Duration(seconds=self.p['transform_tolerance'])).to_msg()
        self.tf.sendTransform(self.make_tf(self.p['map_frame'], self.p['odom_frame'], (x, y, 0.0),
                                           q_rpy(0.0, 0.0, yaw), stamp))

    @staticmethod
    def make_tf(parent, child, t, q, stamp):
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id, m.header.stamp = parent, child, stamp
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
        (m.transform.rotation.x, m.transform.rotation.y,
         m.transform.rotation.z, m.transform.rotation.w) = map(float, q)
        return m

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'status': self.status, 'source': 'simulated', 'received': {'pose': self.sent},
            'drift_m': [round(d, 4) for d in self.drift[:2]], 'drift_yaw_rad': round(self.drift[2], 4),
        })))


def main():
    rclpy.init()
    node = FakeQuest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
