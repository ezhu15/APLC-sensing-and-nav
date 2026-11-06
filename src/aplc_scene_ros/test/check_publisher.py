"""Run inside sourced ROS environment; validates real DDS late-subscription."""
import json
import os
import subprocess
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from aplc_scene.map import load_map
from aplc_scene_ros.geometry import triangles


def main():
    path = os.environ['APLC_TEST_MAP']
    data = load_map(path)
    proc = subprocess.Popen(['ros2','run','aplc_scene_ros','scene_publisher','--ros-args','-p',f'map_file:={path}','-p','republish_seconds:=60.0'])
    rclpy.init(); node = rclpy.create_node('scene_integration_check')
    try:
        time.sleep(3)  # Subscribe after the initial publication.
        received=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub=node.create_subscription(MarkerArray,'scene/markers',received.append,qos)
        deadline=time.monotonic()+12
        while not received and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=0.2)
        assert received, 'Late subscriber did not receive retained scene'
        markers=received[0].markers
        assert markers[0].action==Marker.DELETEALL
        anchors=[a for a in data['anchors'] if a['geometry'] and 'CEILING' not in a['labels']]
        mesh=[m for m in markers if m.ns=='anchors']
        assert len(mesh)==len(anchors),(len(mesh),len(anchors))
        assert all(m.header.frame_id==data['frame_id'] for m in markers)
        assert len({(m.ns,m.id) for m in markers})==len(markers)
        expected={a['uuid']:triangles(a['geometry']) for a in anchors}
        assert sum(len(m.points) for m in mesh)==sum(len(p) for p in expected.values())
        assert all(m.type==Marker.TRIANGLE_LIST and len(m.points)%3==0 for m in mesh)
        assert len([m for m in markers if m.ns=='rooms'])==len(data['rooms'])
        # Compare published coordinates against pure geometry output, not just counts.
        for index,a in enumerate(sorted(data['anchors'],key=lambda a:a['uuid'])):
            if a['uuid'] not in expected: continue
            marker=next(m for m in mesh if m.id==index)
            for p,q in zip(marker.points,expected[a['uuid']]):
                assert max(abs(p.x-q[0]),abs(p.y-q[1]),abs(p.z-q[2]))<1e-9
        print(f'PASS: late subscriber received {len(mesh)} geometry markers, {len(data["rooms"])} room labels; frame and vertices verified')
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
