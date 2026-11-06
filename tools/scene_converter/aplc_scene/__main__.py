import argparse
import json
from pathlib import Path
from .map import convert_snapshot, load_map, write_map
from .gazebo import export


def main():
    parser = argparse.ArgumentParser(description='APLC portable scene converter')
    commands = parser.add_subparsers(dest='command', required=True)
    imp = commands.add_parser('import-quest', help='Import an APLC schema-1 snapshot')
    imp.add_argument('snapshot',type=Path); imp.add_argument('--output',type=Path,required=True)
    imp.add_argument('--frame-id',default='map')
    val = commands.add_parser('validate'); val.add_argument('map',type=Path)
    gz = commands.add_parser('gazebo'); gz.add_argument('map',type=Path)
    gz.add_argument('--output',type=Path,required=True); gz.add_argument('--name',default='aplc_rooms')
    gz.add_argument('--plane-thickness',type=float,default=0.02)
    args = parser.parse_args()
    try:
        if args.command == 'import-quest':
            data = convert_snapshot(json.loads(args.snapshot.read_text()),args.frame_id)
            write_map(data,args.output)
        elif args.command == 'validate': data = load_map(args.map)
        else:
            result = export(args.map,args.output,args.name,args.plane_thickness)
            print(f"Exported {len(result['anchors'])} meshes from {result['source_room_count']} rooms")
            for w in result['warnings']: print('Review:',w)
            return
        print(f"Map {data['map_id']}: {len(data['rooms'])} rooms, {len(data['anchors'])} anchors; frame={data['frame_id']}; meters")
    except (ValueError,KeyError,TypeError,OSError,IndexError) as exc:
        parser.exit(1,f'Conversion failed: {exc}\n')


if __name__ == '__main__': main()
