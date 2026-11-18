#!/usr/bin/env python3
"""Generate the Nav2 Route Server graph (GeoJSON) for the house_1 / house_ramp world.

Coordinates are the Gazebo world frame, which is also the map frame in simulation
(the simulated Quest publishes map -> odom with map = Gazebo world).

Route: inside the front door -> landing -> down the ramp -> apron -> one of two
driveway lanes -> behind the car's trunk. The two lanes give the Route Server an
alternative when one lane is blocked. Lane A (nearer the house side of the
driveway) is slightly shorter, so it is the default choice.

Every edge is listed in both directions (Route Server edges are directed).
Node ids are 0..N-1; edge ids start at 100 so they never collide with node ids.

Usage (bare filename goes to ../graphs/):
  python3 gen_route_graph.py -o house_route.geojson
"""
import argparse
import json
import math
import os

# name: (x, y) in meters, map frame.
NODES = [
    ('inside', (-1.5, 3.5)),       # ground floor, 1.5 m inside the doorway
    ('landing', (0.7, 3.5)),       # outside the door (landing spans x 0..1.52, y 2.75..4.25)
    ('ramp_top', (0.6, 2.4)),      # ramp: x 0..1.2, descends along -y to y = -2.65
    ('ramp_bottom', (0.6, -3.1)),
    ('apron', (1.6, -3.4)),        # concrete apron x 0..2; driveway x 2..14, y -5.15..-1.65
    ('lane_a_start', (2.8, -2.6)),
    ('lane_a_end', (5.0, -2.6)),
    ('lane_b_start', (2.8, -4.4)),
    ('lane_b_end', (5.0, -4.4)),
    ('trunk', (6.0, -3.4)),        # default car rear bumper at x = 7.0, y = -3.4
]

EDGES = [
    ('inside', 'landing'),
    ('landing', 'ramp_top'),
    ('ramp_top', 'ramp_bottom'),
    ('ramp_bottom', 'apron'),
    ('apron', 'lane_a_start'),
    ('lane_a_start', 'lane_a_end'),
    ('lane_a_end', 'trunk'),
    ('apron', 'lane_b_start'),
    ('lane_b_start', 'lane_b_end'),
    ('lane_b_end', 'trunk'),
]


def build():
    ids = {name: i for i, (name, _) in enumerate(NODES)}
    features = []
    for name, (x, y) in NODES:
        features.append({
            'type': 'Feature',
            'properties': {'id': ids[name], 'frame': 'map', 'metadata': {'name': name}},
            'geometry': {'type': 'Point', 'coordinates': [x, y]},
        })
    pos = dict(NODES)
    edge_id = 100
    for a, b in EDGES:
        for s, e in ((a, b), (b, a)):
            features.append({
                'type': 'Feature',
                'properties': {
                    'id': edge_id, 'startid': ids[s], 'endid': ids[e],
                    'metadata': {'name': f'{s}->{e}'},
                },
                'geometry': {'type': 'MultiLineString', 'coordinates': [[list(pos[s]), list(pos[e])]]},
            })
            edge_id += 1
    return {
        'type': 'FeatureCollection',
        'name': 'aplc_house_route',
        'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
        'features': features,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-o', '--output', required=True, help='output .geojson (bare name: ../graphs/)')
    a = ap.parse_args()
    out = a.output
    if os.path.dirname(out) == '':
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'graphs', out)
    with open(out, 'w') as f:
        json.dump(build(), f, indent=1)
        f.write('\n')
    pos = dict(NODES)
    lane = lambda pts: sum(math.dist(pos[p], pos[q]) for p, q in zip(pts, pts[1:]))
    print(f'wrote {os.path.normpath(out)}: {len(NODES)} nodes, {2 * len(EDGES)} edges; '
          f'lane A {lane(["apron", "lane_a_start", "lane_a_end", "trunk"]):.2f} m, '
          f'lane B {lane(["apron", "lane_b_start", "lane_b_end", "trunk"]):.2f} m')


if __name__ == '__main__':
    main()
