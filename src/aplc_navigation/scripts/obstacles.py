#!/usr/bin/env python3
"""Spawn, move and remove demo obstacles in the running Gazebo world (house_ramp).

Talks to Gazebo directly over gz-transport (no ROS needed). Obstacles are static
models named demo_*, so they never get pushed around by the robot.

  obstacles box   [--x 3.9 --y -2.6]      one 0.4 x 0.4 x 0.8 m box (lane A centerline):
                                          the controller should drive around it
  obstacles block [--lane a|b]            a barrier across one whole lane: the controller
                                          cannot pass, so the Route Server must reroute
  obstacles walker [--x 3.9 --speed 0.4]  a person-sized cylinder pacing across the
                                          driveway until Ctrl+C (then removed)
  obstacles clear                         remove every demo_* obstacle
  obstacles list

Positions are the map frame (= Gazebo world). Lane centerlines: A y = -2.6, B y = -4.4,
x 2.8 .. 5.0 (see graphs/house_route.geojson).
"""
import argparse
import math
import sys
import time

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene

WORLD = 'house_ramp'
LANE_Y = {'a': -2.6, 'b': -4.4}
DRIVEWAY_Y = (-5.15, -1.65)


def model_sdf(name, shape, color=(0.9, 0.35, 0.1)):
    r, g, b = color
    material = (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
                '</material>')
    return (f'<sdf version="1.9"><model name="{name}"><static>true</static><link name="link">'
            f'<collision name="c"><geometry>{shape}</geometry></collision>'
            f'<visual name="v"><geometry>{shape}</geometry>{material}</visual>'
            '</link></model></sdf>')


def box(sx, sy, sz):
    return f'<box><size>{sx} {sy} {sz}</size></box>'


def cylinder(r, h):
    return f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>'


class World:
    def __init__(self, world=WORLD):
        self.node = Node()
        self.world = world

    def call(self, service, req, req_t, rep_t, timeout=3000):
        ok, rep = self.node.request(f'/world/{self.world}/{service}', req, req_t, rep_t, timeout)
        if not ok:
            sys.exit(f'Gazebo service /world/{self.world}/{service} did not answer. Is the simulation running?')
        return rep

    def spawn(self, name, shape, x, y, z, yaw=0.0, color=(0.9, 0.35, 0.1)):
        self.remove(name, quiet=True)
        req = EntityFactory()
        req.sdf = model_sdf(name, shape, color)
        req.name = name
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        req.pose.orientation.z, req.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        rep = self.call('create', req, EntityFactory, Boolean)
        print(f'{"spawned" if rep.data else "FAILED to spawn"} {name} at ({x:.2f}, {y:.2f})')

    def remove(self, name, quiet=False):
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        rep = self.call('remove', req, Entity, Boolean)
        if not quiet:
            print(f'removed {name}' if rep.data else f'{name} not present')

    def set_pose(self, name, x, y, z):
        req = Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.w = 1.0
        ok, _ = self.node.request(f'/world/{self.world}/set_pose', req, Pose, Boolean, 500)
        return ok

    def demo_models(self):
        scene = self.call('scene/info', Empty(), Empty, Scene)
        return [m.name for m in scene.model if m.name.startswith('demo_')]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=WORLD)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('box', help='one box the controller can drive around')
    b.add_argument('--x', type=float, default=3.9)
    b.add_argument('--y', type=float, default=LANE_Y['a'])
    b.add_argument('--size', type=float, default=0.4, help='footprint edge (m)')
    b.add_argument('--height', type=float, default=0.8)
    b.add_argument('--name', default='demo_box')
    k = sub.add_parser('block', help='barrier across a whole lane')
    k.add_argument('--lane', choices=['a', 'b'], default='a')
    k.add_argument('--x', type=float, default=3.9)
    w = sub.add_parser('walker', help='cylinder pacing across the driveway until Ctrl+C')
    w.add_argument('--x', type=float, default=3.9)
    w.add_argument('--speed', type=float, default=0.4, help='m/s')
    w.add_argument('--y-min', type=float, default=DRIVEWAY_Y[0] + 0.3)
    w.add_argument('--y-max', type=float, default=DRIVEWAY_Y[1] - 0.3)
    w.add_argument('--pause', type=float, default=2.0, help='seconds standing still at each end')
    w.add_argument('--keep', action='store_true', help='leave it in place on exit')
    sub.add_parser('clear', help='remove all demo_* obstacles')
    sub.add_parser('list', help='list demo_* obstacles')
    a = ap.parse_args()
    world = World(a.world)

    if a.cmd == 'box':
        world.spawn(a.name, box(a.size, a.size, a.height), a.x, a.y, a.height / 2)
    elif a.cmd == 'block':
        # A 3.5 m wall from 0.9 m past the lane's centerline (the other lane's centerline
        # stays 0.9 m clear) far out over the grass, so there is no short way around it:
        # the controller stalls and the Route Server has to switch lanes.
        y_lane = LANE_Y[a.lane]
        if a.lane == 'a':
            y0, y1 = y_lane - 0.9, y_lane + 2.6
        else:
            y0, y1 = y_lane - 2.6, y_lane + 0.9
        world.spawn(f'demo_block_{a.lane}', box(0.3, abs(y1 - y0), 1.0), a.x, (y0 + y1) / 2, 0.5,
                    color=(0.85, 0.1, 0.1))
    elif a.cmd == 'walker':
        name, h = 'demo_walker', 1.7
        y, direction = a.y_max, -1.0
        world.spawn(name, cylinder(0.25, h), a.x, y, h / 2, color=(0.2, 0.4, 0.9))
        print(f'walker pacing y {a.y_min:.2f} .. {a.y_max:.2f} at {a.speed} m/s; Ctrl+C to stop')
        dt, last = 0.05, time.monotonic()
        try:
            while True:
                time.sleep(dt)
                now = time.monotonic()
                y += direction * a.speed * (now - last)
                last = now
                if not a.y_min <= y <= a.y_max:
                    y = min(a.y_max, max(a.y_min, y))
                    direction = -direction
                    world.set_pose(name, a.x, y, h / 2)
                    time.sleep(a.pause)
                    last = time.monotonic()
                world.set_pose(name, a.x, y, h / 2)
        except KeyboardInterrupt:
            pass
        finally:
            if not a.keep:
                world.remove(name)
    elif a.cmd == 'clear':
        names = world.demo_models()
        for n in names:
            world.remove(n)
        if not names:
            print('no demo obstacles')
    elif a.cmd == 'list':
        print('\n'.join(world.demo_models()) or 'no demo obstacles')


if __name__ == '__main__':
    main()
