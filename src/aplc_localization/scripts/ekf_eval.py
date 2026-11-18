#!/usr/bin/env python3
"""Evaluate the EKF against Gazebo ground truth.

  ekf_eval run      run the test matrix (fresh simulation per run), then analyze
  ekf_eval analyze  analyze bags recorded earlier

Run from the repository root (defaults are relative to it):
  bags    eval_bags/                      (gitignored)
  results docs/localization/results/      (summary.md + PNG plots)

Each run: launch the simulation headless with the case's gyro bias, EKF config and
Gazebo seed; record wheel odometry, /odometry/filtered, /imu and /ground_truth/odom;
drive the test with ekf_eval_drive; shut down.

Analysis: t0 = the first instant all three pose sources publish (robot at rest). Each
source is re-expressed relative to its own pose at t0 (position and yaw), because ground
truth is world-referenced and the EKF's heading drifts from launch. Estimates are
interpolated to the ground-truth timestamps and compared in x, y and yaw.
Ground truth is only ever read here, never by the EKF.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

TOPICS = {
    'wheel': '/diff_drive_controller/odom',
    'ekf': '/odometry/filtered',
    'truth': '/ground_truth/odom',
}
RECORD = list(TOPICS.values()) + ['/imu']
SPAWN = {'door': {'x': '-1.2', 'y': '3.5', 'z': '0.50', 'yaw': '0.0'}}   # inside, facing the door
BIAS_DEG = {'0': 0.0, '0.1': 0.1, '1': 1.0}   # gyro bias magnitude per axis, deg/s


# ----------------------------------------------------------------------------- running

def cases(tests, repeats, seed0, door_bag):
    out = []
    for test in tests:
        if test == 'door' and not door_bag:
            print(f'skipping door: no --door-bag (record one first, see docs/localization/ekf.md)')
            continue
        variants = [(b, False) for b in BIAS_DEG] + ([('0.1', True)] if test == 'door' else [])
        for bias, ax in variants:
            for r in range(repeats):
                seed = seed0 + r
                name = f'{test}_gyro{bias}_ax{"on" if ax else "off"}_s{seed}'
                out.append(dict(name=name, test=test, gyro_bias_deg=BIAS_DEG[bias], bias_label=bias,
                                ax=ax, seed=seed))
    return out


def ax_on_config(bags_dir):
    """Copy of the installed ekf.yaml with forward acceleration (ax) fused from the IMU."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    src = os.path.join(get_package_share_directory('aplc_localization'), 'config', 'ekf.yaml')
    cfg = yaml.safe_load(open(src))
    cfg['ekf_filter_node']['ros__parameters']['imu0_config'][12] = True
    path = os.path.join(bags_dir, 'ekf_ax_on.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


def stop(proc, sig=signal.SIGINT, timeout=20):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def wait_for_line(log_path, proc, text, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if proc.poll() is not None:
            raise RuntimeError(f'simulation exited early, see {log_path}')
        with open(log_path, errors='replace') as f:
            if text in f.read():
                return
        time.sleep(0.5)
    raise RuntimeError(f'timed out waiting for "{text}", see {log_path}')


def run_case(case, args, ax_config):
    bag = os.path.join(args.bags, case['name'])
    if os.path.exists(bag):
        if not args.overwrite:
            print(f'  {case["name"]}: bag exists, skipping (use --overwrite)')
            return
        subprocess.run(['rm', '-rf', bag], check=True)
    env = dict(os.environ, GZ_PARTITION=f'ekf_eval_{os.getpid()}_{case["name"]}')
    launch = ['ros2', 'launch', 'aplc_gazebo', 'world.launch.py', 'gui:=false', 'camera:=false',
              'rviz:=false', f'seed:={case["seed"]}',
              f'imu_gyro_bias:={math.radians(case["gyro_bias_deg"]):.6f}']
    if case['ax']:
        launch.append(f'ekf_config:={ax_config}')
    for k, v in SPAWN.get(case['test'], {}).items():
        launch.append(f'{k}:={v}')
    drive = ['ros2', 'run', 'aplc_localization', 'ekf_eval_drive',
             {'stationary': 'stationary', 'square': 'square', 'door': 'replay'}[case['test']]]
    if case['test'] == 'door':
        drive += ['--bag', os.path.abspath(args.door_bag)]

    log = os.path.join(args.bags, f'{case["name"]}.log')
    print(f'  {case["name"]}: launching')
    with open(log, 'w') as logf:
        sim = subprocess.Popen(launch, stdout=logf, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        rec = None
        try:
            wait_for_line(log, sim, 'Configured and activated diff_drive_controller', 240)
            rec = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag] + RECORD,
                                   stdout=logf, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            time.sleep(3.0)   # let the recorder subscribe before the robot moves
            print(f'  {case["name"]}: driving ({case["test"]})')
            subprocess.run(drive, env=env, check=True, stdout=logf, stderr=subprocess.STDOUT,
                           timeout=args.run_timeout)
        finally:
            if rec:
                stop(rec)
            stop(sim)
    with open(os.path.join(bag, 'case.json'), 'w') as f:
        json.dump(case, f, indent=1)
    print(f'  {case["name"]}: done')


# ----------------------------------------------------------------------------- analysis

def read_bag(path):
    import numpy as np
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS.values())))
    by_topic = {t: [] for t in TOPICS.values()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        by_topic[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y,
                                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    out = {}
    for key, topic in TOPICS.items():
        a = np.array(sorted(by_topic[topic]))
        if len(a) < 2:
            raise RuntimeError(f'{path}: no data on {topic}')
        a[:, 3] = np.unwrap(a[:, 3])
        out[key] = a
    return out


def rezero(a, t0):
    """Pose relative to the pose at t0: rotate by -yaw0 about the t0 position."""
    import numpy as np
    x0, y0, yaw0 = (np.interp(t0, a[:, 0], a[:, i]) for i in (1, 2, 3))
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    dx, dy = a[:, 1] - x0, a[:, 2] - y0
    return np.column_stack([a[:, 0] - t0, c * dx - s * dy, s * dx + c * dy, a[:, 3] - yaw0])


def load_run(path):
    """Bag -> poses relative to t0, plus how long all sources overlap."""
    case = json.load(open(os.path.join(path, 'case.json')))
    raw = read_bag(path)
    t0 = max(a[0, 0] for a in raw.values())
    t1 = min(a[-1, 0] for a in raw.values())
    rel = {k: rezero(a[a[:, 0] >= t0 - 0.1], t0) for k, a in raw.items()}
    return case, rel, t1 - t0


def analyze_run(case, rel, duration):
    """Errors over [0, duration] s. Runs of one test share a duration so they compare."""
    import numpy as np
    truth = rel['truth'][(rel['truth'][:, 0] >= 0) & (rel['truth'][:, 0] <= duration)]
    t = truth[:, 0]
    res = {'case': case, 't': t, 'truth': truth, 'est': {}, 'metrics': {}}
    for k in ('wheel', 'ekf'):
        e = np.column_stack([t] + [np.interp(t, rel[k][:, 0], rel[k][:, i]) for i in (1, 2, 3)])
        pos_err = np.hypot(e[:, 1] - truth[:, 1], e[:, 2] - truth[:, 2])
        # Both headings are unwrapped, so this is the accumulated error (it can exceed 180 deg).
        yaw_err = np.degrees(e[:, 3] - truth[:, 3])
        res['est'][k] = e
        res['metrics'][k] = {
            'final_pos': float(pos_err[-1]), 'rms_pos': float(np.sqrt(np.mean(pos_err ** 2))),
            'max_pos': float(pos_err.max()), 'final_yaw': float(yaw_err[-1]),
            'max_yaw': float(np.abs(yaw_err).max()), 'yaw_err': yaw_err,
        }
    res['duration'] = float(t[-1])
    res['path_len'] = float(np.sum(np.hypot(np.diff(truth[:, 1]), np.diff(truth[:, 2]))))
    return res


def label(case):
    return f'EKF, gyro bias {case["bias_label"]} deg/s' + (', ax on' if case['ax'] else '')


def plots(runs, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'0': 'tab:green', '0.1': 'tab:blue', '1': 'tab:red'}
    for test in sorted({r['case']['test'] for r in runs}):
        rs = sorted([r for r in runs if r['case']['test'] == test],
                    key=lambda r: (r['case']['ax'], BIAS_DEG[r['case']['bias_label']], r['case']['seed']))
        first_seed = min(r['case']['seed'] for r in rs)
        fig, ax = plt.subplots(figsize=(7, 3.4), dpi=80)
        wheel_shown = False
        for r in rs:
            c = r['case']
            style = '--' if c['ax'] else '-'
            lw = 1.4 if c['seed'] == first_seed else 0.6
            lab = label(c) if c['seed'] == first_seed else None
            ax.plot(r['t'], r['metrics']['ekf']['yaw_err'], style, color=colors[c['bias_label']], lw=lw, label=lab)
            if not wheel_shown and c['seed'] == first_seed:
                ax.plot(r['t'], r['metrics']['wheel']['yaw_err'], color='0.35', lw=1.4, label='wheel odometry only')
                wheel_shown = True
        ax.axhline(0, color='k', lw=0.5)
        ax.set_xlabel('time since start (s)')
        ax.set_ylabel('heading error (deg)')
        ax.set_title(f'{test}: heading error vs ground truth')
        ax.legend(fontsize=7, loc='best')
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'{test}_heading_error.png'))
        plt.close(fig)

        if test == 'stationary':
            continue
        fig, ax = plt.subplots(figsize=(5.2, 5.0), dpi=80)
        ref = next(r for r in rs if r['case']['seed'] == first_seed)
        ax.plot(ref['truth'][:, 1], ref['truth'][:, 2], 'k', lw=2.2, label='ground truth')
        ax.plot(ref['est']['wheel'][:, 1], ref['est']['wheel'][:, 2], color='0.45', lw=1.2, label='wheel odometry only')
        for r in rs:
            if r['case']['seed'] != first_seed:
                continue
            c = r['case']
            ax.plot(r['est']['ekf'][:, 1], r['est']['ekf'][:, 2], '--' if c['ax'] else '-',
                    color=colors[c['bias_label']], lw=1.2, label=label(c))
        ax.plot(0, 0, 'ko', ms=5)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('x from start (m)')
        ax.set_ylabel('y from start (m)')
        ax.set_title(f'{test}: path, start pose = origin, heading +x')
        ax.legend(fontsize=7, loc='best')
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'{test}_path.png'))
        plt.close(fig)


def summary(runs, out_dir):
    lines = [
        '# EKF evaluation results',
        '',
        'Generated by `ros2 run aplc_localization ekf_eval`. Errors are against Gazebo ground',
        'truth, with every source re-expressed relative to its own pose at the start. Position',
        'errors in metres, yaw errors in degrees. "wheel" = wheel odometry only; "EKF" =',
        '/odometry/filtered.',
        '',
        '| Test | Gyro bias (deg/s) | ax | Seed | Duration (s) | Path (m) | Estimator | Final pos | RMS pos | Max pos | Final yaw | Max yaw |',
        '|---|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    order = {'stationary': 0, 'square': 1, 'door': 2}
    for r in sorted(runs, key=lambda r: (order.get(r['case']['test'], 9), r['case']['ax'],
                                         BIAS_DEG[r['case']['bias_label']], r['case']['seed'])):
        c = r['case']
        for k, name in (('wheel', 'wheel'), ('ekf', '**EKF**')):
            m = r['metrics'][k]
            lines.append(f'| {c["test"]} | {c["bias_label"]} | {"on" if c["ax"] else "off"} | {c["seed"]} | '
                         f'{r["duration"]:.1f} | {r["path_len"]:.2f} | {name} | {m["final_pos"]:.3f} | '
                         f'{m["rms_pos"]:.3f} | {m["max_pos"]:.3f} | {m["final_yaw"]:+.2f} | {m["max_yaw"]:.2f} |')
    lines += ['', 'Plots: `*_heading_error.png` (heading error over time) and `*_path.png` (x-y path).', '']
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines))
    return '\n'.join(lines)


def analyze(args):
    loaded = []
    for name in sorted(os.listdir(args.bags)):
        path = os.path.join(args.bags, name)
        if os.path.isfile(os.path.join(path, 'case.json')):
            loaded.append(load_run(path))
    if not loaded:
        sys.exit(f'no evaluated runs in {args.bags}')
    # Recording length varies by a second or two with startup timing; cut every run of a
    # test to the shortest one so "final" errors are taken at the same elapsed time.
    common = {}
    for case, _rel, dur in loaded:
        common[case['test']] = min(common.get(case['test'], dur), dur)
    runs = [analyze_run(case, rel, common[case['test']]) for case, rel, _dur in loaded]
    os.makedirs(args.results, exist_ok=True)
    plots(runs, args.results)
    print(summary(runs, args.results))
    print(f'\nwrote {args.results}/summary.md and plots')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('run', 'analyze'):
        p = sub.add_parser(name)
        p.add_argument('--bags', default='eval_bags')
        p.add_argument('--results', default='docs/localization/results')
    r = sub.choices['run']
    r.add_argument('--tests', default='stationary,square,door', help='comma list: stationary,square,door')
    r.add_argument('--door-bag', default='eval_bags/door_to_car_cmd', help='cmd_vel bag for the door test')
    r.add_argument('--repeats', type=int, default=1, help='runs per case with seeds seed..seed+N-1')
    r.add_argument('--seed', type=int, default=1)
    r.add_argument('--run-timeout', type=float, default=900)
    r.add_argument('--overwrite', action='store_true', help='re-run cases whose bag already exists')
    args = ap.parse_args()

    if args.cmd == 'run':
        os.makedirs(args.bags, exist_ok=True)
        door_bag = args.door_bag if os.path.exists(args.door_bag) else None
        todo = cases([t.strip() for t in args.tests.split(',') if t.strip()], args.repeats, args.seed, door_bag)
        ax_config = ax_on_config(args.bags) if any(c['ax'] for c in todo) else None
        print(f'{len(todo)} runs')
        for i, case in enumerate(todo, 1):
            print(f'[{i}/{len(todo)}]')
            run_case(case, args, ax_config)
    analyze(args)


if __name__ == '__main__':
    main()
