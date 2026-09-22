#!/usr/bin/env python3
"""Run a desk benchmark with the real controller TF; requires an idle Unitree domain 0."""
import argparse
import json
import os
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '93'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--checkpoint', required=True)
    args = parser.parse_args()
    import numpy as np
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    observer = rclpy.create_node('benchmark_smoke')
    results = []
    subscription = observer.create_subscription(String, '/sam3/benchmark',
                                               lambda msg: results.append(json.loads(msg.data)), 10)
    commands = [
        ['ros2', 'run', 'g1_controller', 'g1_mujoco_camera', '-r', 'g1',
         '-s', args.scene, '-i', '0', '-n', 'lo'],
        ['ros2', 'launch', 'g1_controller', 'g1_controller_robogpc_steering.launch.py',
         'simulation:=true', 'camera:=true', 'rviz:=false', 'sam3:=true',
         'sam3_python:=/opt/g1-sam3-venv/bin/python', f'sam3_checkpoint:={args.checkpoint}',
         'sam3_prompt:=monitor', 'sam3_benchmark:=true',
         'sam3_target_body:=desk_Furniture_Monitor'],
    ]
    children, logs = [], []
    try:
        for i, command in enumerate(commands):
            log = open(f'/tmp/g1-benchmark-smoke-{i}.log', 'w')
            logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True))
        deadline = time.monotonic() + 90
        while len(results) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            assert all(p.poll() is None for p in children), 'Child exited; inspect smoke logs'
        assert len(results) >= 3, 'No benchmark results; inspect /tmp/g1-benchmark-smoke-*.log'
        for result in results:
            assert result['frame_id'] == 'pelvis'
            assert result['target_body'] == 'desk_Furniture_Monitor'
            delta = np.array(result['estimated']) - result['ground_truth']
            np.testing.assert_allclose(delta, result['delta'])
            np.testing.assert_allclose(np.linalg.norm(delta), result['distance_m'])
        assert len({r['stamp_ns'] for r in results}) == len(results)
        print('PASS: live controller pelvis TF + SAM3 + exact-stamp MuJoCo benchmark', flush=True)
        print(json.dumps(results[-1], indent=2), flush=True)
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
