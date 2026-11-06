import copy
import json
from pathlib import Path
import tempfile
import unittest
from aplc_scene.map import convert_snapshot, validate_map, load_map, write_map
from aplc_scene.gazebo import export


def snapshot():
    anchor = dict(uuid='floor',labels=['FLOOR'],has_plane=True,has_volume=False,
                  plane_boundary_local_uv=[[0,0],[200,0],[200,100],[0,100]],
                  world_transform=dict(translation=[100,200,300],quaternion_xyzw=[0,0,0,1],scale=[1,1,1]))
    return dict(schema_version=1,world_to_meters=100,tracking_origin='LocalFloor',
                rooms=[dict(uuid='room',anchors=[anchor])])


class MapTests(unittest.TestCase):
    def test_boundary_frame_metadata_and_roundtrip(self):
        m = convert_snapshot(snapshot())
        a = m['anchors'][0]
        self.assertEqual(a['pose']['position_m'],[1,-2,3])
        self.assertEqual(a['plane']['boundary_m'],[[1,-2,3],[1,-4,3],[1,-4,4],[1,-2,4]])
        self.assertEqual(m['provenance']['capture_metadata']['tracking_origin'],'LocalFloor')
        self.assertEqual(m['alignment']['status'],'unresolved')
        self.assertIsNone(m['rooms'][0]['floor_id'])
        self.assertEqual(m,convert_snapshot(snapshot()))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.json'; write_map(m,path)
            self.assertEqual(m,load_map(path))
            with self.assertRaises(FileExistsError): write_map(m,path)

    def test_shared_membership_and_conflict(self):
        s=snapshot(); s['rooms'].append(dict(uuid='room2',anchors=copy.deepcopy(s['rooms'][0]['anchors'])))
        m=convert_snapshot(s)
        self.assertEqual(len(m['anchors']),1)
        self.assertEqual(m['anchors'][0]['room_uuids'],['room','room2'])
        s['rooms'][1]['anchors'][0]['labels']=['WALL_FACE']
        with self.assertRaises(ValueError): convert_snapshot(s)

    def test_volume_and_no_geometry_retained(self):
        s=snapshot(); a=s['rooms'][0]['anchors'][0]
        a.update(has_volume=True,volume_min_local=[0,0,0],volume_max_local=[100,200,300])
        m=convert_snapshot(s)
        self.assertEqual(m['anchors'][0]['geometry']['corners_m'][-1],[2,-4,6])
        self.assertIsNotNone(m['anchors'][0]['plane'])
        a.update(has_volume=False,has_plane=False,plane_boundary_local_uv=[])
        self.assertIsNone(convert_snapshot(s)['anchors'][0]['geometry'])

    def test_invalid_input(self):
        for value in [0,-1,float('inf'),True]:
            s=snapshot();s['world_to_meters']=value
            with self.assertRaises(ValueError): convert_snapshot(s)
        m=convert_snapshot(snapshot());m['anchors'][0]['plane']['boundary_m'][0][0]=float('nan')
        with self.assertRaises(ValueError): validate_map(m)
        m=convert_snapshot(snapshot());m['rooms'][0]['anchor_uuids']=[]
        with self.assertRaises(ValueError): validate_map(m)

    def test_landmarks_and_floor_no_double_conversion(self):
        s=snapshot();a=copy.deepcopy(s['rooms'][0]['anchors'][0]);a.update(uuid='stairs',labels=['WALL_ART'])
        s['rooms'][0]['anchors'].append(a)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'map.json';write_map(convert_snapshot(s),path)
            result=export(path,Path(tmp)/'gazebo')
            self.assertEqual([a['collision'] for a in result['anchors']],[True,False])
            vertices=[list(map(float,l.split()[1:])) for l in (Path(tmp)/'gazebo/aplc_rooms/meshes/anchor_0000.obj').read_text().splitlines() if l.startswith('v ')]
            self.assertAlmostEqual(min(v[0] for v in vertices),0.99)
            self.assertAlmostEqual(max(v[0] for v in vertices),1.01)
            self.assertEqual(min(v[1] for v in vertices),-4)
