"""Actual TCP and ROS integration: composition, stale stream and map rejection."""
import json
import math
import os
import socket
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile, DurabilityPolicy
from aplc_scene.map import load_map


def main():
    path=os.environ['APLC_TEST_MAP'];data=load_map(path)
    proc=subprocess.Popen(['ros2','run','aplc_scene_ros','quest_bridge','--ros-args','-p',f'map_file:={path}','-p','pose_port:=15005','-p','alignment_port:=15006','-p','path_limit:=5'])
    rclpy.init();node=rclpy.create_node('live_bridge_integration');sockets=[]
    poses=[];statuses=[];paths=[];transforms=[];trails=[];retained=[]
    subs=[node.create_subscription(PoseStamped,'quest/pose',poses.append,10),node.create_subscription(String,'quest/status',lambda m:statuses.append(json.loads(m.data)),10),node.create_subscription(Path,'quest/path',paths.append,10),node.create_subscription(TFMessage,'/tf',transforms.append,10)]
    subs.append(node.create_subscription(MarkerArray,'quest/trail',trails.append,10))
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def send(sock,msg):sock.sendall((json.dumps(msg)+'\n').encode())
    try:
        for port in (15005,15006):
            deadline=time.monotonic()+10
            while True:
                try:sock=socket.create_connection(('127.0.0.1',port),timeout=.2);sock.settimeout(2);sockets.append(sock);break
                except OSError:
                    if time.monotonic()>deadline:raise
                    spin(.1)
        spin(1)
        start=time.monotonic()
        def timestamp():return (time.monotonic()-start+1000)*1e6
        alignment=dict(type='anchor_alignment',version=1,session_id='integration',sequence=0,timestamp_us=timestamp(),ttl_seconds=2,map_id=data['map_id'],parent_frame_id=data['frame_id'],child_frame_id='quest_world',valid=True,status='aligned',transform=dict(position_m=[10,20,0],orientation_xyzw=[0,0,math.sqrt(.5),math.sqrt(.5)]),inlier_uuids=['a','b','c'])
        send(sockets[1],alignment)
        for seq in range(15):
            send(sockets[0],dict(version=1,sequence=seq,timestamp_us=timestamp(),frame_id='quest_world',position=dict(x=1,y=0,z=2),orientation=dict(x=0,y=0,z=0,w=1)))
            spin(.06)
        assert poses,'No ROS poses'
        p=poses[-1].pose.position
        assert abs(p.x-10)<1e-6 and abs(p.y-21)<1e-6 and abs(p.z-2)<1e-6,(p.x,p.y,p.z)
        assert any(t.child_frame_id=='quest_headset' for m in transforms for t in m.transforms),'No headset TF'
        assert any(p.poses for p in paths),'No trail'
        spin(.8)
        assert statuses[-1]['status']=='pose_stale',statuses[-1]
        assert paths[-1].poses,'Historical path was cleared on stale pose'
        assert sum(len(m.points) for m in trails[-1].markers)==5,'Trail point bound not enforced'
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subs.append(node.create_subscription(MarkerArray,'quest/trail',retained.append,qos))
        spin(.2)
        assert retained and sum(len(m.points) for m in retained[-1].markers)==5,'Late RViz subscriber lost retained history'
        alignment.update(sequence=1,timestamp_us=timestamp())
        send(sockets[1],alignment)
        for seq in (15,16):
            send(sockets[0],dict(version=1,sequence=seq,timestamp_us=timestamp(),frame_id='quest_world',position=dict(x=2,y=0,z=2),orientation=dict(x=0,y=0,z=0,w=1)))
            spin(.15)
        segments=[m for m in trails[-1].markers if m.action==Marker.ADD]
        assert len(segments)==2,'Tracking gap was joined into a single trail segment'
        assert sum(len(m.points) for m in segments)==5,'History exceeded point limit'
        assert len(paths[-1].poses)==2,'Path should contain only the current continuous segment'
        count=len(poses)
        alignment.update(sequence=2,timestamp_us=timestamp(),map_id='wrong-map')
        send(sockets[1],alignment);spin(.3)
        assert statuses[-1]['status']=='map_or_frame_mismatch',statuses[-1]
        assert len(poses)==count,'Published with wrong map'
        spin(2.1)
        assert statuses[-1]['status']=='alignment_stale',statuses[-1]
        print(f'PASS: {count} mapped poses, TF/trail, bounded segmented history, retained late delivery, map-ID rejection and invalid-status expiry')
    finally:
        for sock in sockets:sock.close()
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
