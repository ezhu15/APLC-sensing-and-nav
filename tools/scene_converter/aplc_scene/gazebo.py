#!/usr/bin/env python3
"""Export a portable APLC scene map as a Gazebo model.

Standard library only. Input must be an aplc.scene map produced by import-quest.
One snapshot per export: separate sessions must be aligned before merging.
"""
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

SOLID = {'FLOOR', 'WALL_FACE', 'INNER_WALL_FACE', 'OTHER_ROOM_FACE',
         'CEILING', 'TABLE', 'COUCH', 'BED', 'STORAGE', 'SCREEN', 'LAMP', 'PLANT', 'OTHER'}
COLORS = {'FLOOR': '0.55 0.55 0.55 1', 'WALL_FACE': '0.8 0.8 0.85 1',
          'WALL_ART': '1 0.35 0.05 1', 'DOOR_FRAME': '0.1 0.7 0.2 1',
          'CEILING': '0.6 0.6 0.7 1'}


def finite(values):
    if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
        raise ValueError('Non-finite or nonnumeric geometry')
    return values


def cross2(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def polygon(points):
    """Ear clipping for a simple polygon without holes; rejects degeneracy."""
    p = [tuple(finite(v)) for v in points]
    if p and p[0] == p[-1]:
        p.pop()
    if len(p) < 3 or any(len(v) != 2 for v in p):
        raise ValueError('Plane needs at least three 2D vertices')
    area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(p, p[1:]+p[:1]))
    if abs(area) < 1e-8:
        raise ValueError('Degenerate plane polygon')
    if area < 0:
        p.reverse()
    # Remove adjacent duplicates / collinear points before clipping.
    changed = True
    while changed and len(p) > 3:
        changed = False
        for i in range(len(p)):
            if abs(cross2(p[i-1], p[i], p[(i+1) % len(p)])) < 1e-8:
                p.pop(i)
                changed = True
                break
    order, triangles = list(range(len(p))), []
    while len(order) > 3:
        for j, b in enumerate(order):
            a, c = order[j-1], order[(j+1) % len(order)]
            if cross2(p[a], p[b], p[c]) <= 1e-8:
                continue
            if any(all(cross2(p[x], p[y], p[k]) >= -1e-8
                       for x, y in ((a, b), (b, c), (c, a)))
                   for k in order if k not in (a, b, c)):
                continue
            triangles.append((a, b, c))
            order.pop(j)
            break
        else:
            raise ValueError('Cannot triangulate polygon; check duplicate/self-intersecting edges')
    if cross2(*(p[i] for i in order)) <= 1e-8:
        raise ValueError('Degenerate final plane triangle')
    triangles.append(tuple(order))
    return p, triangles


def prism(points, thickness):
    p, caps = polygon(points)
    n = len(p)
    vertices = [(x, u, v) for x in (-thickness/2, thickness/2) for u, v in p]
    faces = [(c, b, a) for a, b, c in caps] + [tuple(i+n for i in t) for t in caps]
    for i in range(n):
        j = (i+1) % n
        faces += [(i, j, j+n), (i, j+n, i+n)]
    return vertices, faces


def box(lo, hi):
    finite(lo); finite(hi)
    if len(lo) != 3 or len(hi) != 3 or any(b <= a for a, b in zip(lo, hi)):
        raise ValueError('Invalid volume bounds')
    vertices = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
             (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
             (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return vertices, faces


def world_point(p, transform, units):
    t = finite(transform['translation']); q = finite(transform['quaternion_xyzw'])
    s = finite(transform['scale'])
    if len(t) != 3 or len(q) != 4 or len(s) != 3 or any(abs(v) < 1e-12 for v in s):
        raise ValueError('Invalid actor transform')
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-12:
        raise ValueError('Zero quaternion')
    x, y, z, w = (v/norm for v in q)
    a, b, c = (p[i]*s[i] for i in range(3))
    # Same numerical rotation as Unreal FQuat::RotateVector, then basis reflection.
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    v = [a+w*tx+y*tz-z*ty+t[0], b+w*ty+z*tx-x*tz+t[1], c+w*tz+x*ty-y*tx+t[2]]
    return [v[0]/units, -v[1]/units, v[2]/units]


def signed_volume(v, faces):
    total = 0.0
    for i, j, k in faces:
        a, b, c = v[i], v[j], v[k]
        total += a[0]*(b[1]*c[2]-b[2]*c[1]) + a[1]*(b[2]*c[0]-b[0]*c[2]) + a[2]*(b[0]*c[1]-b[1]*c[0])
    return total/6


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def write_obj(path, vertices, faces):
    # DART's mesh importer requires normals; flat normals preserve sharp edges.
    normals = []
    for i, j, k in faces:
        a = [vertices[j][n]-vertices[i][n] for n in range(3)]
        b = [vertices[k][n]-vertices[i][n] for n in range(3)]
        normal = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
        length = math.sqrt(sum(n*n for n in normal))
        if length <= 1e-15:
            raise ValueError('Degenerate mesh triangle')
        normals.append([n/length for n in normal])
    with Path(path).open('w') as f:
        f.write('# Baked Gazebo-frame vertices in meters; original shared frame retained\n')
        for p in vertices:
            f.write('v ' + ' '.join(f'{n:.12g}' for n in p) + '\n')
        for normal in normals:
            f.write('vn ' + ' '.join(f'{n:.12g}' for n in normal) + '\n')
        for index, face in enumerate(faces, 1):
            f.write('f ' + ' '.join(f'{i+1}//{index}' for i in face) + '\n')


def export(snapshot, output, name='aplc_rooms', thickness=0.02):
    from .map import load_map
    data = load_map(snapshot)
    if not math.isfinite(thickness) or thickness <= 0:
        raise ValueError('Thickness must be positive meters')
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', name):
        raise ValueError('Model name must be an identifier')
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('Output already exists; use a new directory')
    items, warnings = [], []
    for anchor in data['anchors']:
        geometry = anchor.get('geometry')
        if geometry is None:
            warnings.append(f"{anchor['uuid']}: no usable geometry; retained in map")
            continue
        if geometry['type'] == 'box':
            vertices = geometry['corners_m']
            _, faces = box([0, 0, 0], [1, 1, 1])
            kind = 'volume_box'
        else:
            # Triangulate in the plane's orthonormal basis; preserve world vertices.
            boundary = geometry['boundary_m']
            origin = boundary[0]
            def sub(a,b): return [x-y for x,y in zip(a,b)]
            def dot(a,b): return sum(x*y for x,y in zip(a,b))
            def unit(a):
                n = math.sqrt(dot(a,a))
                if n < 1e-12: raise ValueError('Degenerate plane')
                return [x/n for x in a]
            u = unit(sub(boundary[1], origin))
            normal = geometry['normal']
            v = [normal[1]*u[2]-normal[2]*u[1], normal[2]*u[0]-normal[0]*u[2], normal[0]*u[1]-normal[1]*u[0]]
            uv = [[dot(sub(p,origin),u), dot(sub(p,origin),v)] for p in boundary]
            local, faces = prism(uv, thickness)
            vertices = [[origin[i]+x*normal[i]+a*u[i]+b*v[i] for i in range(3)] for x,a,b in local]
            kind = 'extruded_plane'
        if signed_volume(vertices, faces) < 0:
            faces = [(c,b,a) for a,b,c in faces]
        labels = set(anchor['labels'])
        collision = bool(labels & SOLID) and not bool(labels & {'WALL_ART','DOOR_FRAME','WINDOW_FRAME','OPENING'})
        items.append(dict(name=f'anchor_{len(items):04d}', uuid=anchor['uuid'],
                          room_uuid=anchor['room_uuids'][0], room_uuids=anchor['room_uuids'], labels=sorted(labels),
                          geometry=kind, collision=collision, vertices=vertices, faces=faces))
    if not items:
        raise ValueError('No usable geometry in map')
    warnings += ['Openings are not cut into walls/floors; no stair geometry is inferred.',
                 'Plane thickness is assumed, not measured. Review collision geometry before navigation.']
    model_dir = output/name
    (model_dir/'meshes').mkdir(parents=True)
    sdf = ET.Element('sdf', version='1.9')
    model = element(sdf, 'model', name=name)
    element(model, 'static', 'true')
    for item in items:
        mesh_name = item['name'] + '.obj'
        mesh_path = model_dir/'meshes'/mesh_name
        write_obj(mesh_path, item['vertices'], item['faces'])
        link = element(model, 'link', name=item['name'])
        for kind in (['visual', 'collision'] if item['collision'] else ['visual']):
            obj = element(link, kind, name=item['name']+'_'+kind)
            geometry = element(obj, 'geometry')
            mesh = element(geometry, 'mesh')
            element(mesh, 'uri', f'model://{name}/meshes/{mesh_name}')
            if kind == 'visual':
                material = element(obj, 'material')
                color = next((COLORS[x] for x in item['labels'] if x in COLORS), '0.4 0.6 0.8 1')
                element(material, 'ambient', color)
                element(material, 'diffuse', color)
    write_xml(sdf, model_dir/'model.sdf')
    config = ET.Element('model')
    element(config, 'name', name); element(config, 'version', '1.0')
    element(config, 'sdf', 'model.sdf', version='1.9')
    element(config, 'description', 'Quest room layout; approximate solid geometry with separate semantic landmarks')
    write_xml(config, model_dir/'model.config')
    world_root = ET.Element('sdf', version='1.9')
    world = element(world_root, 'world', name='aplc_scene')
    physics = element(world, 'physics', name='1ms', type='ignored')
    element(physics, 'max_step_size', '0.001'); element(physics, 'real_time_factor', '1')
    for filename, system in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
        element(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{system}')
    light = element(world, 'light', name='sun', type='directional')
    element(light, 'diffuse', '0.8 0.8 0.8 1'); element(light, 'direction', '-0.5 0.2 -1')
    include = element(world, 'include'); element(include, 'uri', f'model://{name}')
    write_xml(world_root, output/'world.sdf')
    shutil.copyfile(snapshot, model_dir/'source_map.json')
    manifest = dict(source=str(Path(snapshot).resolve()), source_room_count=len(data['rooms']), map_id=data['map_id'], frame_id=data['frame_id'],
                    units='meters', conversion='Portable map geometry in meters; no additional frame conversion',
                    plane_thickness_m=thickness, warnings=warnings,
                    anchors=[{k:v for k,v in x.items() if k not in ('vertices', 'faces')} for x in items])
    (model_dir/'anchors.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--name', default='aplc_rooms')
    parser.add_argument('--plane-thickness', type=float, default=0.02, help='assumed centered plane thickness in meters')
    args = parser.parse_args()
    try:
        result = export(args.snapshot, args.output, args.name, args.plane_thickness)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(1, f'Export failed: {exc}\n')
    print(f"Exported {len(result['anchors'])} anchors from {result['source_room_count']} rooms to {args.output}")
    for warning in result['warnings']:
        print('Review:', warning)


if __name__ == '__main__':
    main()
