#!/usr/bin/env bash
# Launch only this test's processes; retain logs and propagate the test exit code.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
player="${CI_PLAYER:-$repo_dir/Builds/CI/Linux/CiRover.x86_64}"
rover_ws="${ROVER_WS:-$HOME/rugged_rover_ws}"
output="${CI_LOG_DIR:-$repo_dir/Logs/ci-dropout}"
if [[ ! -f "$player" ]]; then
  echo "Linux player missing: $player. Build with Tools > CI > Build Linux Player." >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$rover_ws/install/setup.bash"
source "${FAULT_INJECTION_WS:-$HOME/fault_injection_ws}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-190}"
mkdir -p "$output"
# The player's ROS connection currently uses localhost:10000.
if ss -ltn | grep -q ':10000 '; then
  echo "Port 10000 is already occupied; stop the other Unity ROS endpoint first." >&2
  exit 2
fi
chmod +x "$player"
children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch rugged_rover_bringup unity_sim.launch.py > "$output/ros.log" 2>&1 &
children+=("$!")
setsid "$player" -batchmode -nographics -logFile "$output/unity.log" > "$output/player-console.log" 2>&1 &
children+=("$!")
setsid ros2 launch ros2_fault_injection fault_injector.launch.py scenario_file:="$repo_dir/tools/ci/scan_dropout.yaml" > "$output/injector.log" 2>&1 &
children+=("$!")
python3 "$repo_dir/tools/ci/test_ci_scan_dropout.py" 2>&1 | tee "$output/dropout-test.log"
