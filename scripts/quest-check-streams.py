#!/usr/bin/env python3
"""Temporary local TCP diagnostic. Does not publish ROS or modify Quest data."""
import argparse
import json
import math
import selectors
import socket
import time
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--bind',default='127.0.0.1',help='Use 0.0.0.0 for LAN or container access')
p.add_argument('--seconds',type=float,default=60)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
sel=selectors.DefaultSelector(); buffers={}; counts={5005:0,5006:0}; errors=[]; last={}; statuses=set()
a.output.mkdir(parents=True,exist_ok=True)
for port in counts:
    s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    try: s.bind((a.bind,port));s.listen();s.setblocking(False)
    except OSError as e: print(f'Port {port} unavailable; leaving existing receiver alone: {e}',flush=True);s.close();continue
    sel.register(s,selectors.EVENT_READ,('listener',port));print(f'Listening {port}',flush=True)
deadline=time.monotonic()+a.seconds
try:
    while time.monotonic()<deadline:
        for key,_ in sel.select(timeout=0.5):
            kind,port=key.data;s=key.fileobj
            if kind=='listener':
                c,_=s.accept();c.setblocking(False);sel.register(c,selectors.EVENT_READ,('stream',port));buffers[c]=b'';continue
            data=s.recv(65536)
            if not data: sel.unregister(s);s.close();buffers.pop(s,None);continue
            buffers[s]+=data
            if len(buffers[s])>1000000: raise ValueError('Oversize TCP frame')
            while b'\n' in buffers[s]:
                line,buffers[s]=buffers[s].split(b'\n',1)
                try:
                    msg=json.loads(line)
                    if port==5005:
                        assert msg['version']==1 and msg['frame_id']=='quest_world'
                        assert all(math.isfinite(x) for field in ('position','orientation') for x in msg[field].values())
                        q=msg['orientation'];assert abs(sum(v*v for v in q.values())-1)<1e-4
                    else:
                        assert msg['type']=='anchor_alignment' and msg['version']==1
                        if msg['valid']:
                            assert msg['status']=='aligned' and len(msg['inlier_uuids'])>=3
                            assert all(math.isfinite(x) for v in msg['transform'].values() for x in v)
                        statuses.add(msg['status'])
                        if msg['status']!=last.get(port,{}).get('status'):print(json.dumps(msg),flush=True)
                    counts[port]+=1;last[port]=msg
                except (ValueError,KeyError,AssertionError,TypeError) as e: errors.append(str(e))
finally:
    for key in list(sel.get_map().values()):key.fileobj.close()
    summary=dict(counts=counts,errors=errors,statuses=sorted(statuses),last=last)
    (a.output/'streams.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
