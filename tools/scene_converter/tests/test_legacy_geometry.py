import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from aplc_scene.gazebo import export, polygon, prism, signed_volume, world_point


class ExportTests(unittest.TestCase):
    def test_concave_floor(self):
        p, triangles = polygon([[0,0],[3,0],[3,1],[1,1],[1,3],[0,3]])
        area = sum(abs((p[b][0]-p[a][0])*(p[c][1]-p[a][1])-(p[b][1]-p[a][1])*(p[c][0]-p[a][0]))/2 for a,b,c in triangles)
        self.assertAlmostEqual(area, 5)
        v, f = prism(p, 0.02)
        self.assertAlmostEqual(abs(signed_volume(v,f)), 0.1)

    def test_frame_conversion(self):
        t = dict(translation=[100,200,300], quaternion_xyzw=[0,0,0,1], scale=[1,1,1])
        self.assertEqual(world_point([0,0,0], t, 100), [1,-2,3])
        t['quaternion_xyzw']=[0,0,math.sqrt(0.5),math.sqrt(0.5)]
        p=world_point([100,0,0],t,100)
        for a,b in zip(p,[1,-3,3]): self.assertAlmostEqual(a,b)

    def test_multilevel_landmarks_and_package(self):
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'scene.json'
            rooms=[]
            for level in range(2):
                anchors=[]
                for label in ['FLOOR','WALL_ART']:
                    anchors.append(dict(uuid=f'{label}_{level}',labels=[label],has_plane=True,has_volume=False,
                        plane_boundary_local_uv=[[0,0],[100,0],[100,100],[0,100]],
                        world_transform=dict(translation=[level*500,0,level*300],quaternion_xyzw=[0,0,0,1],scale=[1,1,1])))
                rooms.append(dict(uuid=f'room_{level}',anchors=anchors))
            src.write_text(json.dumps(dict(schema_version=1,world_to_meters=100,rooms=rooms)))
            from aplc_scene.map import convert_snapshot, write_map
            canonical=Path(d)/'map.json'
            write_map(convert_snapshot(json.loads(src.read_text())), canonical)
            manifest=export(canonical,Path(d)/'export')
            self.assertEqual(manifest['source_room_count'],2)
            self.assertEqual(len(manifest['anchors']),4)
            self.assertEqual(sum(a['collision'] for a in manifest['anchors']),2)
            root=ET.parse(Path(d)/'export/aplc_rooms/model.sdf').getroot()
            self.assertEqual(len(root.findall('.//link')),4)
            self.assertEqual(len(root.findall('.//collision')),2)
            vertices=[]
            for line in (Path(d)/'export/aplc_rooms/meshes/anchor_0002.obj').read_text().splitlines():
                if line.startswith('v '): vertices.append([float(x) for x in line.split()[1:]])
            obj_lines=(Path(d)/'export/aplc_rooms/meshes/anchor_0002.obj').read_text().splitlines()
            normals=[line for line in obj_lines if line.startswith('vn ')]
            faces=[line for line in obj_lines if line.startswith('f ')]
            self.assertEqual(len(normals),len(faces))
            for face in faces:
                self.assertTrue(all('//' in token for token in face.split()[1:]))
            self.assertTrue(all(4.98<v[0]<5.02 for v in vertices))
            self.assertAlmostEqual(min(v[2] for v in vertices),3)
            self.assertAlmostEqual(max(v[2] for v in vertices),4)
            ET.parse(Path(d)/'export/world.sdf')
            with self.assertRaises(ValueError): export(src,Path(d)/'export')

    def test_reject_native_json(self):
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'native.json'; src.write_text('{"spatialEntities":[]}')
            with self.assertRaises(ValueError): export(src,Path(d)/'out')
            self.assertFalse((Path(d)/'out').exists())


if __name__=='__main__': unittest.main()
