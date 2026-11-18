#!/usr/bin/env python3
"""Draw the route graph in RViz so a viewer can read it, and highlight the current route.

The Route Server's own /route_graph markers are 10 cm dots with number labels at z = 0,
so on the landing and ramp they sit under the floor. This node reads the same GeoJSON
and publishes /route_graph_viz (MarkerArray, transient local):

  route_graph_viz/nodes       spheres at floor height (landing 0.45 m, ramp sloped)
  route_graph_viz/labels      node names
  route_graph_viz/edges       grey lines, one per undirected edge
  route_graph_viz/active      thick orange lines on the edges of the route last
                              published on /plan (the Route Server's path); a reroute
                              shows as the highlight jumping to the other lane

Floor heights are specific to the house_1 / house_ramp world.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

LANDING_Z = 0.45            # ground floor and landing
RAMP_Y = (-2.65, 2.75)      # ramp runs down along -y between these, x 0 .. 1.2
LIFT = 0.05                 # draw slightly above the floor


def floor_z(x, y):
    if x < 0.0 or (x <= 1.52 and y >= RAMP_Y[1]):
        return LANDING_Z
    if 0.0 <= x <= 1.2 and RAMP_Y[0] <= y <= RAMP_Y[1]:
        return LANDING_Z * (y - RAMP_Y[0]) / (RAMP_Y[1] - RAMP_Y[0])
    return 0.0


def load(path):
    with open(path) as f:
        data = json.load(f)
    nodes, edges = {}, set()
    for feat in data['features']:
        p = feat['properties']
        if feat['geometry']['type'] == 'Point':
            x, y = feat['geometry']['coordinates'][:2]
            nodes[p['id']] = (p.get('metadata', {}).get('name', str(p['id'])), x, y)
        elif 'startid' in p:
            edges.add(tuple(sorted((p['startid'], p['endid']))))
    return nodes, sorted(edges)


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def color(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


class GraphViz(Node):
    def __init__(self):
        super().__init__('route_graph_viz')
        default = os.path.join(get_package_share_directory('aplc_navigation'), 'graphs', 'house_route.geojson')
        path = self.declare_parameter('graph_filepath', default).value
        self.frame = self.declare_parameter('frame', 'map').value
        self.nodes, self.edges = load(path)
        self.active = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'route_graph_viz', qos)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.publish()
        self.get_logger().info(f'{len(self.nodes)} nodes, {len(self.edges)} edges from {path}')

    def pt(self, nid, dz=LIFT):
        _, x, y = self.nodes[nid]
        return Point(x=x, y=y, z=floor_z(x, y) + dz)

    def marker(self, ns, mid, mtype, scale, col):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = col
        return m

    def on_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return
        segs = list(zip(pts, pts[1:]))
        active = []
        for a, b in self.edges:
            (_, ax, ay), (_, bx, by) = self.nodes[a], self.nodes[b]
            mid = ((ax + bx) / 2, (ay + by) / 2)
            if min(seg_dist(mid, s, e) for s, e in segs) < 0.2:
                active.append((a, b))
        if active != self.active:
            self.active = active
            self.publish()

    def publish(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        nodes = self.marker('route_graph_viz/nodes', 0, Marker.SPHERE_LIST, 0.22, color(0.95, 0.25, 0.2))
        nodes.points = [self.pt(n) for n in self.nodes]
        arr.markers.append(nodes)

        edges = self.marker('route_graph_viz/edges', 0, Marker.LINE_LIST, 0.06, color(0.85, 0.85, 0.85, 0.8))
        for a, b in self.edges:
            edges.points += [self.pt(a), self.pt(b)]
        arr.markers.append(edges)

        if self.active:
            act = self.marker('route_graph_viz/active', 0, Marker.LINE_LIST, 0.14, color(1.0, 0.6, 0.0, 0.9))
            for a, b in self.active:
                act.points += [self.pt(a, LIFT + 0.02), self.pt(b, LIFT + 0.02)]
            arr.markers.append(act)

        for i, nid in enumerate(self.nodes):
            lab = self.marker('route_graph_viz/labels', i, Marker.TEXT_VIEW_FACING, 0.28, color(1.0, 1.0, 1.0))
            lab.pose.position = self.pt(nid, 0.35)
            lab.text = self.nodes[nid][0]
            arr.markers.append(lab)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = GraphViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
