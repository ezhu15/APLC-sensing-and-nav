"""ROS-independent protocol, alignment and freshness checks."""
import math


def number(v):
    if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v): raise ValueError('Expected finite number')
    return float(v)


def vector(v,n):
    if not isinstance(v,(list,tuple)) or len(v)!=n: raise ValueError('Invalid vector size')
    return [number(x) for x in v]


def quaternion(v):
    q=vector(v,4);norm=sum(x*x for x in q)
    if abs(norm-1)>0.01: raise ValueError('Quaternion must be normalized')
    return [x/math.sqrt(norm) for x in q]


def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]


def rotate(q,p):
    return multiply(multiply(q,[*p,0]),[-q[0],-q[1],-q[2],q[3]])[:3]


def compose(transform,pose):
    p=rotate(transform[1],pose[0])
    return ([x+y for x,y in zip(transform[0],p)],multiply(transform[1],pose[1]))


class LiveState:
    def __init__(self,map_id,frame_id,pose_timeout=0.5):
        self.map_id=map_id;self.frame_id=frame_id;self.pose_timeout=pose_timeout
        self.pose=None;self.alignment=None;self.session=None;self.offset=None
        self.max_time=None;self.max_received=None;self.last_pose_seq=None;self.last_align_seq=None
        self.align_problem='waiting_alignment';self.generation=0;self.retired_sessions=set()
        self.pose_peer=None;self.align_peer=None;self.alignment_message=None

    def disconnect(self,kind):
        if kind=='pose':
            self.pose=None;self.last_pose_seq=None;self.pose_peer=None
        else:
            self.alignment=None;self.align_peer=None;self.alignment_message=None;self.align_problem='alignment_disconnected'
        self.generation+=1

    def clock(self,t,now):
        offset=now-t/1e6
        self.offset=offset if self.offset is None else min(self.offset,offset)
        if self.max_time is None or t>=self.max_time:self.max_time=t;self.max_received=now

    def ingest(self,kind,msg,now,peer='127.0.0.1'):
        if not isinstance(msg,dict) or type(msg.get('version')) is not int or msg['version']!=1: raise ValueError('Unsupported protocol version')
        seq=number(msg['sequence']);t=number(msg['timestamp_us'])
        if seq<0 or not seq.is_integer() or t<0: raise ValueError('Invalid sequence/timestamp')
        if kind=='pose':
            if msg['frame_id']!='quest_world': raise ValueError('Wrong pose frame')
            pose=(vector([msg['position'][k] for k in 'xyz'],3),quaternion([msg['orientation'][k] for k in 'xyzw']))
            if self.last_pose_seq is not None and seq<=self.last_pose_seq:
                self.pose=None;self.alignment=None;self.align_problem='pose_sequence_reset_or_replay';self.generation+=1
                raise ValueError('Pose sequence reset/replay: waiting for a new alignment session')
            self.last_pose_seq=seq
            self.pose=(pose,t,now,seq);self.pose_peer=peer
        else:
            if msg['type']!='anchor_alignment' or not isinstance(msg['session_id'],str) or not msg['session_id']: raise ValueError('Invalid alignment session')
            if msg['session_id'] in self.retired_sessions: raise ValueError('Retired alignment session')
            new_session=self.session!=msg['session_id']
            if new_session:
                if self.session is not None:
                    self.retired_sessions.add(self.session);self.pose=None;self.last_pose_seq=None
                self.session=msg['session_id'];self.last_align_seq=None;self.offset=None;self.max_time=None;self.generation+=1
                if self.pose is not None:self.clock(self.pose[1],self.pose[2])
            if self.last_align_seq is not None and seq<=self.last_align_seq: raise ValueError('Alignment replay/out of order')
            ttl=number(msg['ttl_seconds'])
            if not 0<ttl<=2: raise ValueError('Invalid alignment TTL')
            self.last_align_seq=seq;self.alignment=None;self.align_peer=peer
            self.alignment_message=(t,now,ttl)
            self.clock(t,now)
            if msg['map_id']!=self.map_id or msg['parent_frame_id']!=self.frame_id or msg['child_frame_id']!='quest_world':
                self.align_problem='map_or_frame_mismatch';self.generation+=1;return
            if msg['valid'] is not True or msg['status']!='aligned':
                self.align_problem='alignment_'+str(msg['status']);self.generation+=1;return
            transform=(vector(msg['transform']['position_m'],3),quaternion(msg['transform']['orientation_xyzw']))
            ids=msg['inlier_uuids']
            if not isinstance(ids,list) or len(set(ids))<3: raise ValueError('Insufficient inlier anchors')
            self.alignment=(transform,t,now,ttl);self.align_problem=''
        self.clock(t,now)

    def age(self,t,received,now):
        # Lower-bound clock offset learned from both streams. Detects buffering
        # relative to the best observed transit delay, without mixing clock epochs.
        return max(0,now-received,now-(t/1e6+self.offset))

    def status(self,now):
        if self.alignment_message is not None:
            t,received,ttl=self.alignment_message
            if self.age(t,received,now)>ttl:return 'alignment_stale'
        if self.alignment is None:return self.align_problem
        if self.age(self.alignment[1],self.alignment[2],now)>self.alignment[3]:return 'alignment_stale'
        if self.pose is None:return 'waiting_pose'
        if self.pose_peer!=self.align_peer:return 'peer_mismatch'
        if self.age(self.pose[1],self.pose[2],now)>self.pose_timeout:return 'pose_stale'
        return 'live'
