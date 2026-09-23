#!/usr/bin/env bash
# Host-side MCP stdio launcher for the existing ROS simulator container.
set -euo pipefail
bridge_container="${G1_ROS_CONTAINER:-g1-robogpc-sim}"
bridge_session="${BASHPID}-${RANDOM}-${RANDOM}"
bridge_pid_file="/tmp/g1-codex-mcp-${bridge_session}.pid"
bridge_client_pid=''

cleanup() {
  local result=$?
  trap - EXIT INT TERM
  # Docker may leave an exec process alive after its client disconnects.
  # Match both PID and our unique session token before terminating anything.
  docker exec "$bridge_container" python3 -c '
import os, signal, sys, time
from pathlib import Path
pid_file, session = Path(sys.argv[1]), sys.argv[2]
try:
    pid = int(pid_file.read_text())
    process = Path(f"/proc/{pid}")
    def matches():
        try:
            return (f"G1_CODEX_MCP_SESSION={session}".encode() in process.joinpath("environ").read_bytes().split(b"\0")
                    and b"codex_ros_bridge.py" in process.joinpath("cmdline").read_bytes())
        except FileNotFoundError:
            return False
    if matches():
        os.kill(pid, signal.SIGTERM)
        for _ in range(40):
            if not matches():
                break
            time.sleep(.05)
        if matches():
            os.kill(pid, signal.SIGKILL)
except (FileNotFoundError, ProcessLookupError):
    pass
finally:
    pid_file.unlink(missing_ok=True)
' "$bridge_pid_file" "$bridge_session" >/dev/null 2>&1 || true
  if [[ -n "$bridge_client_pid" ]]; then
    kill "$bridge_client_pid" 2>/dev/null || true
    wait "$bridge_client_pid" 2>/dev/null || true
  fi
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

docker exec -i \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RCUTILS_LOGGING_USE_STDOUT=0 \
  -e G1_CODEX_MCP_SESSION="$bridge_session" \
  "$bridge_container" bash -c '
    source /opt/ros/jazzy/setup.bash
    source /root/Workspaces/g1_ros_ws/install/setup.bash
    printf "%s\n" "$$" > "$1"
    shift
    exec /opt/g1-codex-venv/bin/python \
      /root/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/scripts/codex_ros_bridge.py "$@"
  ' bash "$bridge_pid_file" "$@" <&0 &
bridge_client_pid=$!
wait "$bridge_client_pid"
