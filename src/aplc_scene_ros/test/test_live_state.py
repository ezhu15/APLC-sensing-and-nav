import copy
import math
import unittest
from aplc_scene_ros.live_state import LiveState,compose


def pose(seq=1,t=1e6):
    return dict(version=1,sequence=seq,timestamp_us=t,frame_id='quest_world',position=dict(x=1,y=0,z=2),orientation=dict(x=0,y=0,z=0,w=1))


def align(seq=1,t=1e6,session='a'):
    return dict(type='anchor_alignment',version=1,sequence=seq,timestamp_us=t,session_id=session,
                ttl_seconds=2,map_id='test-map',parent_frame_id='map',child_frame_id='quest_world',
                valid=True,status='aligned',transform=dict(position_m=[10,20,0],orientation_xyzw=[0,0,math.sqrt(.5),math.sqrt(.5)]),inlier_uuids=['a','b','c'])


class StateTests(unittest.TestCase):
    def setup_live(self):
        s=LiveState('test-map','map');s.ingest('alignment',align(),10);s.ingest('pose',pose(),10);return s

    def test_transform_direction(self):
        s=self.setup_live();self.assertEqual(s.status(10),'live')
        p,q=compose(s.alignment[0],s.pose[0]);self.assertAlmostEqual(p[0],10);self.assertAlmostEqual(p[1],21);self.assertAlmostEqual(p[2],2)
        self.assertAlmostEqual(q[2],math.sqrt(.5))

    def test_staleness_disconnect_and_invalid_alignment(self):
        s=self.setup_live();self.assertEqual(s.status(10.6),'pose_stale')
        s.ingest('pose',pose(2,3.1e6),12.1);self.assertEqual(s.status(12.1),'alignment_stale')
        s=self.setup_live();s.disconnect('pose');self.assertEqual(s.status(10),'waiting_pose')
        s=self.setup_live();s.disconnect('alignment');self.assertEqual(s.status(10),'alignment_disconnected')
        s=self.setup_live();m=align(2);m.update(valid=False,status='stabilizing');s.ingest('alignment',m,10)
        self.assertEqual(s.status(10),'alignment_stabilizing')

    def test_wrong_map_frame_peer(self):
        for key,value in [('map_id','other'),('parent_frame_id','other'),('child_frame_id','other')]:
            s=self.setup_live();m=align(2);m[key]=value;s.ingest('alignment',m,10)
            self.assertEqual(s.status(10),'map_or_frame_mismatch')
        s=self.setup_live();s.pose_peer='other';self.assertEqual(s.status(10),'peer_mismatch')

    def test_buffered_capture_detected(self):
        s=self.setup_live();s.ingest('pose',pose(2,1.1e6),10.1)
        # Arrives now, but capture time is a second behind the learned clock offset.
        s.ingest('pose',pose(3,1.2e6),11.2);self.assertEqual(s.status(11.2),'pose_stale')

    def test_replay_and_new_session(self):
        s=self.setup_live()
        with self.assertRaises(ValueError):s.ingest('pose',pose(),10.1)
        self.assertNotEqual(s.status(10.1),'live')
        s.ingest('alignment',align(0,1.2e6,'b'),10.2)
        self.assertIsNone(s.pose)
        s.ingest('pose',pose(0,1.2e6),10.2);self.assertEqual(s.status(10.2),'live')
        with self.assertRaises(ValueError):s.ingest('alignment',align(0,1.2e6,'b'),10.2)

    def test_bad_numeric_values(self):
        for value in [float('nan'),float('inf'),True,'1']:
            s=self.setup_live();m=pose(2);m['position']['x']=value
            with self.assertRaises(ValueError):s.ingest('pose',m,10)
        s=self.setup_live();m=pose(2);m['orientation']['w']=0
        with self.assertRaises(ValueError):s.ingest('pose',m,10)

    def test_old_first_alignment_after_fresh_pose(self):
        s=LiveState('test-map','map')
        s.ingest('pose',pose(1,5e6),10)
        s.ingest('alignment',align(1,1e6),10)
        self.assertEqual(s.status(10),'alignment_stale')

    def test_stabilizing_and_mismatch_also_expire(self):
        for mismatch in (False,True):
            s=self.setup_live();m=align(2)
            if mismatch:m['map_id']='wrong-map'
            else:m.update(valid=False,status='stabilizing')
            s.ingest('alignment',m,10)
            self.assertNotEqual(s.status(10),'alignment_stale')
            s.ingest('pose',pose(2,3.1e6),12.1)
            self.assertEqual(s.status(12.1),'alignment_stale')
            s.disconnect('alignment')
            self.assertEqual(s.status(12.1),'alignment_disconnected')
