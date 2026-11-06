"""Nonblocking TCP -> ROS bridge for the unchanged Quest streams."""
from collections import deque
import json
import math
import selectors
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from aplc_scene.map import load_map
from .live_state import LiveState, compose


class LiveBridge(Node):
    def __init__(self):
        super().__init__('quest_bridge')
        for name,value in [('map_file',''),('bind_address','0.0.0.0'),('pose_port',5005),('alignment_port',5006),
                           ('pose_timeout',0.5),('path_limit',15000),('connection_timeout',5.0)]: self.declare_parameter(name,value)
        data=load_map(self.get_parameter('map_file').value)
        self.frame=data['frame_id'];timeout=self.get_parameter('pose_timeout').value
        limit=self.get_parameter('path_limit').value
        if not math.isfinite(timeout) or timeout<=0 or limit<1 or limit>15000: raise ValueError('Invalid timeout/path limit')
        self.connection_timeout=self.get_parameter('connection_timeout').value
        if not math.isfinite(self.connection_timeout) or self.connection_timeout<=0:raise ValueError('Invalid connection timeout')
        self.state=LiveState(data['map_id'],self.frame,timeout)
        self.selector=selectors.DefaultSelector();self.connections={}
        self.pose_pub=self.create_publisher(PoseStamped,'quest/pose',10)
        history_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,'quest/path',history_qos)
        self.trail_pub=self.create_publisher(MarkerArray,'quest/trail',history_qos)
        self.marker_pub=self.create_publisher(MarkerArray,'quest/markers',1)
        self.status_pub=self.create_publisher(String,'quest/status',1)
        self.tf=TransformBroadcaster(self)
        self.path=deque(maxlen=limit);self.segment_ids=deque(maxlen=limit)
        self.path_session=None;self.segment=0;self.break_pending=True
        self.last_key=None;self.last_status=None;self.last_status_time=0
        self.last_path_time=0;self.generation=0;self.rejected=0;self.received={'pose':0,'alignment':0}
        try:
            for kind in ('pose','alignment'):
                port=self.get_parameter(kind+'_port').value
                if not 1<=port<=65535: raise ValueError('Invalid TCP port')
                server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                try:
                    server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                    server.bind((self.get_parameter('bind_address').value,port));server.listen(2);server.setblocking(False)
                    self.selector.register(server,selectors.EVENT_READ,('listener',kind,None))
                except Exception:server.close();raise
                self.get_logger().info(f'Listening for {kind} on {server.getsockname()}')
        except Exception:
            self.close_network();raise
        self.timer=self.create_timer(0.02,self.tick)

    def close_network(self):
        for key in list(self.selector.get_map().values()):key.fileobj.close()
        self.selector.close()

    def drop(self,sock,kind):
        self.selector.unregister(sock);sock.close();self.connections.pop(kind,None)
        self.state.disconnect(kind)

    def network(self,now):
        for kind,(sock,buffer,last_message) in list(self.connections.items()):
            if now-last_message>self.connection_timeout:
                self.get_logger().info(f'Closing inactive {kind} TCP connection; allowing reconnect')
                self.drop(sock,kind)
        for key,_ in self.selector.select(timeout=0):
            mode,kind,peer=key.data;sock=key.fileobj
            if mode=='listener':
                try:conn,address=sock.accept()
                except OSError:continue
                conn.setblocking(False)
                if kind in self.connections:conn.close();continue
                self.connections[kind]=[conn,b'',now]
                self.get_logger().info(f'Accepted {kind} TCP client {address[0]}')
                self.selector.register(conn,selectors.EVENT_READ,('stream',kind,address[0]))
                continue
            try:data=sock.recv(65536)
            except BlockingIOError:continue
            except (ConnectionError,OSError):data=b''
            if not data:self.drop(sock,kind);continue
            buffer=self.connections[kind][1]+data
            if len(buffer)>65536:self.rejected+=1;self.drop(sock,kind);continue
            lines=buffer.split(b'\n');self.connections[kind][1]=lines.pop()
            for line in lines:
                try:
                    if len(line)>16384:raise ValueError('Oversize JSON message')
                    self.state.ingest(kind,json.loads(line),now,peer)
                    self.received[kind]+=1
                    self.connections[kind][2]=now
                except (ValueError,KeyError,TypeError,IndexError,UnicodeError):
                    self.rejected+=1;self.state.disconnect(kind)

    def fill_pose(self,target,pose):
        p,q=pose
        target.position.x,target.position.y,target.position.z=map(float,p)
        target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w=map(float,q)

    def transform(self,parent,child,pose,stamp):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
        p,q=pose
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,p)
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
        return t

    def publish_path(self,stamp):
        path=Path();path.header.frame_id=self.frame;path.header.stamp=stamp
        # Path has no discontinuity representation: expose only the latest segment.
        path.poses=[p for segment,p in zip(self.segment_ids,self.path) if segment==self.segment]
        self.path_pub.publish(path)
        clear=Marker();clear.action=Marker.DELETEALL
        markers=[clear];current=None;last_segment=None
        for segment,p in zip(self.segment_ids,self.path):
            if segment!=last_segment:
                current=Marker();current.header.frame_id=self.frame;current.ns='quest_trail';current.id=segment
                current.type=Marker.LINE_STRIP;current.action=Marker.ADD;current.pose.orientation.w=1.0
                current.scale.x=0.03;current.color.r=0.2;current.color.g=0.8;current.color.b=1.0;current.color.a=1.0
                markers.append(current);last_segment=segment
            position=p.pose.position
            current.points.append(Point(x=position.x,y=position.y,z=position.z))
        self.trail_pub.publish(MarkerArray(markers=[m for m in markers if m.action==Marker.DELETEALL or len(m.points)>=2]))

    def tick(self):
        now=time.monotonic();self.network(now)
        status=self.state.status(now);stamp=self.get_clock().now().to_msg()
        if self.state.session is not None and self.path_session!=self.state.session:
            self.path.clear();self.segment_ids.clear();self.last_key=None
            self.segment=0;self.break_pending=True;self.path_session=self.state.session
            self.publish_path(stamp)
        if status!='live':self.break_pending=True
        changed=status!=self.last_status
        if changed or now-self.last_status_time>=1:
            info=dict(status=status,map_id=self.state.map_id,session_id=self.state.session,
                      received=self.received,rejected=self.rejected,
                      pose_age_seconds=self.state.age(self.state.pose[1],self.state.pose[2],now) if self.state.pose else None,
                      history_points=len(self.path),history_limit=self.path.maxlen,
                      connected_streams=sorted(self.connections),connection_idle_seconds={kind:now-values[2] for kind,values in self.connections.items()},
                      alignment_age_seconds=self.state.age(self.state.alignment_message[0],self.state.alignment_message[1],now) if self.state.alignment_message else None)
            self.status_pub.publish(String(data=json.dumps(info)))
            if changed:self.get_logger().info('Quest status: '+status)
            self.last_status_time=now
            if status!='live':
                clear=Marker();clear.action=Marker.DELETEALL
                label=Marker();label.header.frame_id=self.frame;label.ns='quest_status';label.id=0
                label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.0
                label.pose.position.z=1.0;label.scale.z=0.3;label.color.r=1.0;label.color.g=0.4;label.color.a=1.0
                label.text='Quest: '+status
                self.marker_pub.publish(MarkerArray(markers=[clear,label]))
        self.last_status=status
        if status!='live':return
        raw,t,received,seq=self.state.pose
        key=(self.state.session,seq)
        if key==self.last_key:
            if now-self.last_path_time>=0.2:self.publish_path(stamp);self.last_path_time=now
            return
        self.last_key=key
        mapped=compose(self.state.alignment[0],raw)
        # Estimated capture stamp: subtract monotonic age from ROS receipt time.
        age=self.state.age(t,received,now)
        capture=self.get_clock().now()-Duration(seconds=age)
        stamp=capture.to_msg()
        pose=PoseStamped();pose.header.frame_id=self.frame;pose.header.stamp=stamp
        self.fill_pose(pose.pose,mapped);self.pose_pub.publish(pose)
        self.tf.sendTransform([self.transform(self.frame,'quest_world',self.state.alignment[0],stamp),
                               self.transform('quest_world','quest_headset',raw,stamp)])
        if self.break_pending:self.segment+=1;self.break_pending=False
        self.path.append(pose);self.segment_ids.append(self.segment)
        if now-self.last_path_time>=0.2:self.publish_path(stamp);self.last_path_time=now
        clear=Marker();clear.action=Marker.DELETEALL
        m=Marker();m.header=pose.header;m.ns='quest_headset';m.id=0;m.type=Marker.ARROW;m.action=Marker.ADD
        m.pose=pose.pose;m.scale.x=0.4;m.scale.y=0.10;m.scale.z=0.10
        m.color.r=0.1;m.color.g=0.8;m.color.b=1.0;m.color.a=1.0
        m.lifetime=Duration(seconds=self.state.pose_timeout).to_msg()
        self.marker_pub.publish(MarkerArray(markers=[clear,m]))


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=LiveBridge();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.close_network();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
