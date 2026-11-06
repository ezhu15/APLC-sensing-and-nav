#!/usr/bin/env python3
"""Select Quest USB/Wi-Fi TCP destination without rebuilding. Uses ADB for setup."""
import argparse
import ipaddress
import subprocess
import tempfile
from pathlib import Path

SAVED='/sdcard/Android/data/com.JHU.APLC/files/UnrealGame/APLC_Test/APLC_Test/Saved'


def make_config(mode,host,port):
    if not 1<=port<=65534: raise ValueError('Pose port must be 1..65534; alignment uses port+1')
    if mode=='wifi':
        if not host: raise ValueError('--host is required for wifi')
        addr=ipaddress.ip_address(host)
        if addr.is_unspecified or addr.is_multicast or addr.is_loopback or addr.is_link_local:
            raise ValueError('Use the receiver computer\'s numeric LAN address')
        host=str(addr)
    else: host='127.0.0.1'
    return f'[APLC.Network]\nTransport={"WiFi" if mode=="wifi" else "USB"}\nHost={host}\nPosePort={port}\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['usb','wifi'])
    parser.add_argument('--host',help='Receiver computer LAN IP (wifi only)')
    parser.add_argument('--port',type=int,default=5005,help='Pose port; alignment uses port+1')
    parser.add_argument('--serial',help='ADB device serial')
    parser.add_argument('--restart',action='store_true',help='Restart APLC to apply settings')
    parser.add_argument('--dry-run',action='store_true',help='Print settings without using ADB')
    args=parser.parse_args()
    try: content=make_config(args.mode,args.host,args.port)
    except ValueError as exc: parser.error(str(exc))
    print(content,end='')
    if args.dry_run:return
    adb=['adb']+(['-s',args.serial] if args.serial else [])
    def run(*command):subprocess.run(adb+list(command),check=True)
    run('shell','mkdir','-p',SAVED)
    with tempfile.TemporaryDirectory(prefix='aplc-network-') as tmp:
        path=Path(tmp)/'QuestNetwork.ini';path.write_text(content)
        run('push',str(path),SAVED+'/QuestNetwork.ini')
    if args.mode=='usb':
        for port in (args.port,args.port+1):run('reverse',f'tcp:{port}',f'tcp:{port}')
    if args.restart:run('shell','am','start','-S','-n','com.JHU.APLC/com.epicgames.unreal.GameActivity')
    else:print('Settings saved. Restart APLC to apply them.')


if __name__=='__main__':main()
