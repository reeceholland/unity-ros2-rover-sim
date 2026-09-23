#!/usr/bin/env bash
# One isolated simulation per invocation. All deadlines use wall time.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:-navigation}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in motion|dropout|navigation) ;; *) echo "Usage: $0 {motion|dropout|navigation} [test arguments]" >&2; exit 2;; esac
player="${CI_PLAYER:-$repo_dir/Builds/CI/Linux/CiRover.x86_64}"
output="${CI_LOG_DIR:-$repo_dir/Logs/ci-$mode}"
[[ -f "$player" ]] || { echo "Missing Linux player: $player" >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "${ROVER_WS:-$HOME/rugged_rover_ws}/install/setup.bash"
source "${FAULT_INJECTION_WS:-$HOME/fault_injection_ws}/install/setup.bash"
export ROS_DOMAIN_ID="${CI_ROS_DOMAIN_ID:-190}"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID >= 1 && ROS_DOMAIN_ID <= 232 )) || { echo "CI_ROS_DOMAIN_ID must be 1..232" >&2; exit 2; }
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p "$output"
output="$(cd "$output" && pwd)"
# Prevent two runners racing for the same endpoint.
exec 9>"/tmp/ci-rover-port-10000.lock"
flock -n 9 || { echo "Another rover CI run owns port 10000" >&2; exit 2; }
if ss -ltn | grep -q ':10000 '; then
  echo "Port 10000 is occupied; stop the existing endpoint first." >&2; exit 2
fi
chmod +x "$player"
children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for ((i=0; i<50; i++)); do
    alive=false
    for pid in "${children[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=true; done
    "$alive" || break
    sleep 0.1
  done
  for pid in "${children[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
navigation=false
[[ "$mode" != navigation ]] || navigation=true
scenario="$repo_dir/tools/ci/navigation_faults.yaml"
[[ "$mode" != dropout ]] || scenario="$repo_dir/tools/ci/scan_dropout.yaml"
setsid ros2 launch rugged_rover_bringup unity_sim.launch.py use_slam:="$navigation" use_nav2:="$navigation" use_motor_fault_injection:=true ros_tcp_ip:=127.0.0.1 > "$output/ros.log" 2>&1 &
children+=("$!")
setsid ros2 launch ros2_fault_injection fault_injector.launch.py scenario_file:="$scenario" > "$output/injector.log" 2>&1 &
children+=("$!")
setsid "$player" -batchmode -nographics -logFile "$output/unity.log" > "$output/player-console.log" 2>&1 &
children+=("$!")
case "$mode" in
 motion) test_args=(test_ci_rover_motion.py --ready-timeout 60 --travel-time 2);;
 dropout) test_args=(test_ci_scan_dropout.py);;
 navigation) test_args=(test_ci_navigation.py --scenario "$repo_dir/tools/ci/waypoints.json" --output "$output");;
esac
# Clear the previous run before starting the live log follower.
: > "$output/test.log"
# A hard outer deadline also bounds startup and unexpected client hangs.
setsid timeout --signal=INT --kill-after=10s "${CI_TIMEOUT:-900s}" python3 -u "$repo_dir/tools/ci/${test_args[0]}" "${test_args[@]:1}" "$@" > "$output/test.log" 2>&1 &
children+=("$!")
test_pid="${children[-1]}"
# Follow output live without hiding the test exit status behind a pipeline.
setsid tail --pid="$test_pid" --sleep-interval=0.1 -n +1 -f "$output/test.log" &
children+=("$!")
log_pid="${children[-1]}"
result=0
wait "$test_pid" || result=$?
wait "$log_pid" || true
echo "Logs: $output"
exit "$result"
