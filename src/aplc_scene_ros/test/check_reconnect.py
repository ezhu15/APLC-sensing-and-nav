"""Real TCP/ROS regression for abrupt device loss and rebooted monotonic clock."""
import json
import os
import socket
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from aplc_scene.map import load_map


def main():
    path=os.environ['APLC_TEST_MAP'];data=load_map(path)
    proc=subprocess.Popen(['ros2','run','aplc_scene_ros','quest_bridge','--ros-args','-p',f'map_file:={path}',
                           '-p','pose_port:=16005','-p','alignment_port:=16006','-p','connection_timeout:=1.0'])
    rclpy.init();node=rclpy.create_node('quest_reboot_check');poses=[];states=[];sockets=[]
    subs=[node.create_subscription(PoseStamped,'quest/pose',poses.append,10),
          node.create_subscription(String,'quest/status',lambda m:states.append(json.loads(m.data)),10)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def connect(port):
        end=time.monotonic()+8
        while True:
            try:
                s=socket.create_connection(('127.0.0.1',port),timeout=.2);s.settimeout(1);sockets.append(s);return s
            except OSError:
                if time.monotonic()>end:raise
                spin(.05)
    def send(s,m):s.sendall((json.dumps(m)+'\n').encode())
    def pose(seq,t):return dict(version=1,sequence=seq,timestamp_us=t,frame_id='quest_world',position=dict(x=1,y=2,z=3),orientation=dict(x=0,y=0,z=0,w=1))
    def alignment(seq,t,session):return dict(type='anchor_alignment',version=1,sequence=seq,timestamp_us=t,session_id=session,map_id=data['map_id'],parent_frame_id=data['frame_id'],child_frame_id='quest_world',ttl_seconds=2,valid=True,status='aligned',inlier_uuids=['a','b','c'],transform=dict(position_m=[0,0,0],orientation_xyzw=[0,0,0,1]))
    try:
        old_pose=connect(16005);old_alignment=connect(16006);spin(.5)
        send(old_alignment,alignment(100,1e9,'before-reboot'));spin(.1)
        send(old_pose,pose(100,1e9+.1e6));spin(.2)
        assert poses and states[-1]['status']=='live',states
        before=len(poses)
        # No FIN/RST: leave the original sockets open and silent, as can happen
        # when the headset loses power or leaves Wi-Fi without closing TCP.
        spin(1.3)
        new_pose=connect(16005);new_alignment=connect(16006)
        send(new_pose,pose(0,1e6));spin(.05)
        send(new_alignment,alignment(0,1.05e6,'after-reboot'));spin(.05)
        send(new_pose,pose(1,1.1e6));spin(.2)
        assert len(poses)>before,'Pose did not recover after ungraceful loss'
        assert states[-1]['status']=='live',states[-1]
        assert states[-1]['session_id']=='after-reboot'
        assert states[-1]['rejected']==0,states[-1]
        assert states[-1]['pose_age_seconds']<.5,states[-1]
        print('PASS: idle socket replacement, new session, reset sequence and reset device clock recovered without restarting bridge/RViz')
    finally:
        for s in sockets:s.close()
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
