"""Host-side lifecycle test; requires Docker and the prepared ROS container."""
import json
import os
from pathlib import Path
import select
import subprocess
import time
import unittest

WRAPPER = Path(__file__).resolve().parents[1] / 'scripts/codex_ros_docker.sh'


class LauncherTests(unittest.TestCase):
    def launch(self):
        process = subprocess.Popen([str(WRAPPER), '--execute'], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   env=dict(os.environ, ROS_DOMAIN_ID='173'))
        self.addCleanup(self.close, process)
        self.buffer = b''
        self.send(process, {'id': 1, 'method': 'initialize', 'params': {
            'protocolVersion': '2025-06-18', 'capabilities': {},
            'clientInfo': {'name': 'launcher-test', 'version': '1'}}})
        self.assertIn('result', self.receive(process, 1))
        self.send(process, {'method': 'notifications/initialized'})
        self.send(process, {'id': 2, 'method': 'tools/call',
                            'params': {'name': 'observe_scene', 'arguments': {}}})
        result = self.receive(process, 2)['result']
        scene = json.loads(result['content'][0]['text'])
        self.assertFalse(scene['dry_run'])  # --execute must survive the wrapper.
        self.assertEqual(scene['limits']['speed_mps'], .5)
        self.assertEqual(scene['limits']['yaw_rate_radps'], 1.)
        self.assertGreater(scene['bridge_pid'], 0)
        return process

    @staticmethod
    def send(process, value):
        process.stdin.write((json.dumps(dict(jsonrpc='2.0', **value))+'\n').encode())
        process.stdin.flush()

    def receive(self, process, request_id):
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                item = json.loads(line)
                if item.get('id') == request_id:
                    return item
            if select.select([process.stdout], [], [], .1)[0]:
                chunk = os.read(process.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError('Bridge exited: '+process.stderr.read().decode())
                self.buffer += chunk
        raise TimeoutError(request_id)

    @staticmethod
    def close(process):
        if not process.stdin.closed:
            process.stdin.close()
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=8)
        process.stdout.close()
        process.stderr.close()

    def test_signal_and_eof_release_remote_lock(self):
        first = self.launch()
        first.terminate()  # Host client shutdown must stop the remote bridge.
        self.assertEqual(first.wait(timeout=8), 143)
        second = self.launch()  # Fails if the old remote process retains its lock.
        second.stdin.close()
        self.assertEqual(second.wait(timeout=8), 0)
        third = self.launch()  # Normal MCP EOF must release the lock too.
        third.stdin.close()
        self.assertEqual(third.wait(timeout=8), 0)


if __name__ == '__main__':
    unittest.main()
