"""Portable scene map and the isolated APLC Unreal snapshot adapter.

All geometry is baked into one right-handed, meter-valued capture frame.
This frame is not a persistent or surveyed global localization frame.
"""
import hashlib
import json
import math
from pathlib import Path
from .gazebo import box, finite, polygon, world_point

FORMAT = 'aplc.scene'


def vector(value, size):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'Expected {size}-component vector')
    if any(isinstance(x, bool) for x in value):
        raise ValueError('Boolean is not a coordinate')
    return finite(value)


def identity(value, description):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{description} must be a nonempty string')
    return value


def pose(transform, units):
    q = vector(transform['quaternion_xyzw'], 4)
    n = math.sqrt(sum(x*x for x in q))
    if n < 1e-12:
        raise ValueError('Zero quaternion')
    return {'position_m': world_point([0,0,0], transform, units),
            'orientation_xyzw': [-q[0]/n, q[1]/n, -q[2]/n, q[3]/n],
            'scale': list(vector(transform['scale'], 3))}


def convert_snapshot(data, frame_id='map'):
    if data.get('schema_version') != 1 or not isinstance(data.get('rooms'), list):
        raise ValueError('Expected APLC schema_version=1 room snapshot, not native MRUK JSON')
    units = data.get('world_to_meters')
    if isinstance(units, bool) or not isinstance(units, (int,float)) or not math.isfinite(units) or units <= 0:
        raise ValueError('world_to_meters must be positive')
    identity(frame_id, 'frame_id')
    digest = hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()
    result = dict(format=FORMAT, schema_version=1, map_id='sha256:'+digest,
                  frame_id=frame_id, units='meters', handedness='right', axes='x-forward y-left z-up',
                  alignment={'status':'unresolved', 'description':'Single capture world frame. Cross-session/device alignment requires calibration or localized shared anchors.'},
                  provenance={'adapter':'aplc-unreal-schema-1', 'source_sha256':digest,
                              'capture_metadata':{k:v for k,v in data.items() if k != 'rooms'},
                              'conversion':'(x,-y,z)/world_to_meters; quaternion (-qx,qy,-qz,qw); anchor world transforms applied once'},
                  rooms=[], anchors=[], annotations={'floors':[], 'connections':[]})
    seen, room_ids = {}, set()
    for room in data['rooms']:
        rid = identity(room['uuid'], 'room uuid')
        if rid in room_ids: raise ValueError(f'Duplicate room {rid}')
        room_ids.add(rid)
        room_entry = dict(uuid=rid, name=None, floor_id=None, anchor_uuids=[])
        if 'world_transform' in room: room_entry['pose'] = pose(room['world_transform'], units)
        result['rooms'].append(room_entry)
        for anchor in room['anchors']:
            uid = identity(anchor['uuid'], 'anchor uuid')
            if uid in room_entry['anchor_uuids']: raise ValueError(f'Duplicate anchor {uid} within room')
            room_entry['anchor_uuids'].append(uid)
            if uid in seen:
                old, entry = seen[uid]
                if old != anchor: raise ValueError(f'Conflicting duplicate anchor {uid}')
                entry['room_uuids'].append(rid)
                continue
            labels = anchor['labels']
            if not isinstance(labels, list) or any(not isinstance(x,str) for x in labels):
                raise ValueError('Anchor labels must be strings')
            transform = anchor['world_transform']
            entry = dict(uuid=uid, room_uuids=[rid], labels=sorted(set(labels)),
                         pose=pose(transform, units), geometry=None, plane=None)
            # Preserve plane data even when a volume is chosen for rendering.
            boundary = anchor.get('plane_boundary_local_uv', [])
            if len(boundary) < 3 and anchor.get('has_plane'):
                lo, hi = vector(anchor['plane_min_uv'],2), vector(anchor['plane_max_uv'],2)
                boundary = [lo,[hi[0],lo[1]],hi,[lo[0],hi[1]]]
            if len(boundary) >= 3:
                points, _ = polygon(boundary)
                points = [world_point([0,u,v], transform, units) for u,v in points]
                a,b,c = points[:3]
                u,v = [b[i]-a[i] for i in range(3)], [c[i]-b[i] for i in range(3)]
                normal = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
                length = math.sqrt(sum(x*x for x in normal))
                if length < 1e-12: raise ValueError('Degenerate transformed plane')
                entry['plane'] = dict(type='plane',boundary_m=points,normal=[x/length for x in normal])
                entry['geometry'] = entry['plane']
            if anchor.get('has_volume'):
                corners,_ = box(anchor['volume_min_local'],anchor['volume_max_local'])
                entry['geometry'] = dict(type='box',corners_m=[world_point(p,transform,units) for p in corners])
            seen[uid] = (anchor,entry)
            result['anchors'].append(entry)
    validate_map(result)
    return result


def validate_map(data):
    if data.get('format') != FORMAT or data.get('schema_version') != 1:
        raise ValueError('Expected portable aplc.scene schema_version=1 map')
    if data.get('units') != 'meters' or data.get('handedness') != 'right':
        raise ValueError('Map must use right-handed meters')
    identity(data['map_id'],'map_id'); identity(data['frame_id'],'frame_id')
    rooms = {identity(r['uuid'],'room uuid'):r for r in data['rooms']}
    anchors = {identity(a['uuid'],'anchor uuid'):a for a in data['anchors']}
    if len(rooms) != len(data['rooms']) or len(anchors) != len(data['anchors']):
        raise ValueError('Duplicate room/anchor ID')
    for uid,a in anchors.items():
        if not a['room_uuids'] or any(r not in rooms for r in a['room_uuids']):
            raise ValueError('Unknown room reference')
        if not isinstance(a['labels'],list) or any(not isinstance(x,str) for x in a['labels']): raise ValueError('Invalid labels')
        p = a['pose']; vector(p['position_m'],3); vector(p['scale'],3)
        q = vector(p['orientation_xyzw'],4)
        if abs(sum(x*x for x in q)-1) > 1e-6: raise ValueError('Quaternion must be normalized')
        for g in [a.get('geometry'), a.get('plane')]:
            if g is None: continue
            if g['type'] == 'plane':
                points = g['boundary_m']
                if len(points)<3: raise ValueError('Plane needs three vertices')
                for v in points: vector(v,3)
                n = vector(g['normal'],3)
                if abs(sum(x*x for x in n)-1)>1e-6: raise ValueError('Invalid plane normal')
                if any(abs(sum((p[i]-points[0][i])*n[i] for i in range(3)))>1e-6 for p in points): raise ValueError('Nonplanar boundary')
            elif g['type'] == 'box':
                if len(g['corners_m']) != 8: raise ValueError('Box needs eight corners')
                for v in g['corners_m']: vector(v,3)
            else: raise ValueError('Unknown geometry type')
        if any(uid not in rooms[r]['anchor_uuids'] for r in a['room_uuids']): raise ValueError('Inconsistent room membership')
    for rid,r in rooms.items():
        if len(set(r['anchor_uuids'])) != len(r['anchor_uuids']): raise ValueError('Duplicate room membership')
        if any(uid not in anchors or rid not in anchors[uid]['room_uuids'] for uid in r['anchor_uuids']): raise ValueError('Inconsistent anchor membership')
    return data


def load_map(path):
    return validate_map(json.loads(Path(path).read_text()))


def write_map(data, path):
    validate_map(data)
    # Exclusive creation prevents silently replacing an annotated map.
    content = json.dumps(data, indent=2, allow_nan=False)+'\n'
    with Path(path).open('x') as f: f.write(content)
