#!/usr/bin/env python3
"""Send the robot to a route graph node and report what happened.

  route_demo --goal trunk          (default) drive from wherever the robot is to the trunk
  route_demo --goal landing        and back
  route_demo --x 4 --y -3 --yaw 0  any pose in the map frame

While driving it prints the route the Route Server computes, as the list of graph nodes
it passes. The behavior tree computes a new route at the start of every attempt (after
each recovery); this script asks the Route Server's compute_route action the same
question at the same moments and prints "REROUTE" when the node list changes. At the end it prints a
summary, measured against Gazebo's ground truth:

  result, time, number of reroutes,
  max deviation: how far the robot's true path got from the route path it was following
      (large near obstacles = it went around them, small elsewhere = it follows the route),
  localization error: map -> base_footprint (Quest + EKF) versus the true pose,
  final error: true distance from the goal.

Exit code 0 when the goal was reached.
"""
import argparse
import json
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, NavigateToPose
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener

# Goal heading at each named node (rad, map frame).
GOAL_YAW = {'trunk': 0.0, 'landing': math.pi, 'inside': math.pi, 'apron': 0.0}


def load_graph(path):
    with open(path) as f:
        data = json.load(f)
    nodes, names = {}, {}
    for feat in data['features']:
        if feat['geometry']['type'] == 'Point':
            name = feat['properties'].get('metadata', {}).get('name', str(feat['properties']['id']))
            nodes[name] = tuple(feat['geometry']['coordinates'][:2])
            names[feat['properties']['id']] = name
    return nodes, names


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


class RouteDemo(Node):
    def __init__(self, nodes, names):
        super().__init__('route_demo')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.nodes, self.names = nodes, names
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.route_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.route_future = None
        self.goal_pose = None
        self.recoveries = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 10)
        self.create_timer(0.2, self.sample)
        self.path = []
        self.route = None
        self.routes = []
        self.gt = None
        self.max_dev = (0.0, None)
        self.loc_err = []
        self.feedback = None

    def on_gt(self, msg):
        self.gt = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def ask_route(self):
        req = ComputeRoute.Goal()
        req.goal = self.goal_pose
        req.use_poses = True
        self.route_future = self.route_client.send_goal_async(req)

    def poll_route(self):
        f = self.route_future
        if f is None or not f.done():
            return
        if hasattr(f.result(), 'get_result_async'):   # goal handle: now wait for the result
            if not f.result().accepted:
                self.get_logger().warn('Route Server rejected the route request')
                self.route_future = None
                return
            self.route_future = f.result().get_result_async()
            return
        self.route_future = None
        res = f.result().result
        self.get_logger().debug(f'compute_route status {f.result().status} error {res.error_code}')
        if f.result().status != 4 or res.error_code != 0:
            self.get_logger().warn(f'Route Server: no route (status {f.result().status}, error {res.error_code})')
            return
        seq = [self.names.get(n.nodeid, str(n.nodeid)) for n in res.route.nodes]
        self.path = [(p.pose.position.x, p.pose.position.y) for p in res.path.poses]
        # A route that is just the remaining tail of the current one is progress, not a reroute.
        if self.route is not None and seq == self.route[len(self.route) - len(seq):]:
            return
        if seq != self.route:
            kind = 'route' if self.route is None else 'REROUTE'
            self.route = seq
            self.routes.append(seq)
            self.get_logger().info(f'{kind}: {" -> ".join(seq)}  (cost {res.route.route_cost:.2f})')

    def sample(self):
        if self.gt is None:
            return
        if len(self.path) >= 2:
            d = min(seg_dist(self.gt, a, b) for a, b in zip(self.path, self.path[1:]))
            if d > self.max_dev[0]:
                self.max_dev = (d, self.gt)
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            t = tf.transform.translation
            self.loc_err.append(math.hypot(t.x - self.gt[0], t.y - self.gt[1]))
        except TransformException:
            pass

    def on_feedback(self, msg):
        self.feedback = msg.feedback
        if msg.feedback.number_of_recoveries > self.recoveries:
            # The tree is about to compute a new route; ask the same question.
            self.recoveries = msg.feedback.number_of_recoveries
            self.reroute_at = self.get_clock().now()

    def run(self, x, y, yaw, timeout):
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:   # wait for /clock
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose action server not available (is the demo launched?)')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.get_logger().info(f'goal ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')
        self.goal_pose = goal.pose
        self.reroute_at = None
        if self.route_client.wait_for_server(timeout_sec=30.0):
            self.ask_route()
        else:
            self.get_logger().warn('compute_route action not found; routes will not be reported')
        start = self.get_clock().now()
        send = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        result = handle.get_result_async()
        last_print = start
        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.poll_route()
            now = self.get_clock().now()
            # The recovery waits 3 s, then the tree recomputes: ask just after that.
            if self.reroute_at is not None and (now - self.reroute_at).nanoseconds > 3.3e9 \
                    and self.route_future is None:
                self.reroute_at = None
                self.ask_route()
            elapsed = (now - start).nanoseconds * 1e-9
            if (now - last_print).nanoseconds > 3e9 and self.feedback is not None:
                last_print = now
                fb = self.feedback
                self.get_logger().info(f't={elapsed:5.1f}s  remaining {fb.distance_remaining:5.2f} m  '
                                       f'recoveries {fb.number_of_recoveries}')
            if elapsed > timeout:
                self.get_logger().error(f'timeout after {timeout:.0f} s; cancelling')
                handle.cancel_goal_async()
                break
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
        status = result.result().status if result.done() else None
        ok = status == 4   # GoalStatus.STATUS_SUCCEEDED
        res = result.result().result if result.done() else None
        final = math.hypot(self.gt[0] - x, self.gt[1] - y) if self.gt else float('nan')
        loc = self.loc_err or [float('nan')]
        dev, where = self.max_dev
        print('\n=== route demo summary ===')
        print(f'result:             {"SUCCEEDED" if ok else f"FAILED (status {status}, error {getattr(res, "error_code", "?")})"}')
        print(f'time:               {elapsed:.1f} s (sim)')
        print(f'routes:             {len(self.routes)} ({max(0, len(self.routes) - 1)} reroutes)')
        for i, r in enumerate(self.routes):
            print(f'  {i}: {" -> ".join(r)}')
        print(f'max deviation:      {dev:.2f} m from the route path'
              + (f' at ({where[0]:.2f}, {where[1]:.2f})' if where else ''))
        print(f'localization error: mean {sum(loc) / len(loc):.3f} m, max {max(loc):.3f} m '
              f'(Quest + EKF vs ground truth, {len(self.loc_err)} samples)')
        print(f'final error:        {final:.2f} m from the goal (true pose)')
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--goal', default='trunk', help='graph node name')
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--yaw', type=float)
    ap.add_argument('--graph', default=os.path.join(
        get_package_share_directory('aplc_navigation'), 'graphs', 'house_route.geojson'))
    ap.add_argument('--timeout', type=float, default=240.0, help='seconds (sim time)')
    a, ros_args = ap.parse_known_args()
    nodes, names = load_graph(a.graph)
    if a.x is not None and a.y is not None:
        x, y = a.x, a.y
    elif a.goal in nodes:
        x, y = nodes[a.goal]
    else:
        sys.exit(f'unknown goal {a.goal!r}; nodes: {", ".join(nodes)}')
    yaw = a.yaw if a.yaw is not None else GOAL_YAW.get(a.goal, 0.0)
    rclpy.init(args=ros_args)
    node = RouteDemo(nodes, names)
    try:
        ok = node.run(x, y, yaw, a.timeout)
    except KeyboardInterrupt:
        ok = False
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
