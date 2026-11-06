import unittest
from aplc_scene_ros.geometry import triangles, room_center


class GeometryTests(unittest.TestCase):
    def test_concave_plane_preserves_area(self):
        points=[[0,0,3],[3,0,3],[3,1,3],[1,1,3],[1,3,3],[0,3,3]]
        vertices=triangles(dict(type='plane',boundary_m=points,normal=[0,0,1]))
        area=0
        for i in range(0,len(vertices),3):
            a,b,c=vertices[i:i+3]
            area+=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
        self.assertAlmostEqual(area,5)
        self.assertTrue(all(v[2]==3 for v in vertices))

    def test_volume_and_empty_room(self):
        corners=[[x,y,z] for x in (1,2) for y in (3,4) for z in (5,6)]
        vertices=triangles(dict(type='box',corners_m=corners))
        self.assertEqual(len(vertices),36)
        self.assertEqual({tuple(v) for v in vertices},{tuple(v) for v in corners})
        self.assertEqual(triangles(None),[])
        self.assertIsNone(room_center({'anchor_uuids':[]},{}))


if __name__=='__main__': unittest.main()
