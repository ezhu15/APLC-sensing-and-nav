"""ROS-independent visualization geometry in the map's existing meter frame."""
from aplc_scene.gazebo import box, polygon


def triangles(geometry):
    if geometry is None: return []
    if geometry['type'] == 'box':
        _, faces = box([0,0,0],[1,1,1])
        vertices = geometry['corners_m']
    else:
        vertices = geometry['boundary_m']
        # Project onto the most stable coordinate plane for triangulation.
        drop = max(range(3), key=lambda i: abs(geometry['normal'][i]))
        projected = [tuple(x for i,x in enumerate(v) if i != drop) for v in vertices]
        points, faces = polygon(projected)
        lookup = dict(zip(projected,vertices))
        vertices = [lookup[p] for p in points]
    return [vertices[i] for face in faces for i in face]


def room_center(room, anchors):
    floor = [anchors[u]['pose']['position_m'] for u in room['anchor_uuids'] if 'FLOOR' in anchors[u]['labels']]
    points = floor or [anchors[u]['pose']['position_m'] for u in room['anchor_uuids']]
    if not points: return None
    return [sum(p[i] for p in points)/len(points) for i in range(3)]
