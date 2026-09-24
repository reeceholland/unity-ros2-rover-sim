#!/usr/bin/env bash
# One isolated simulation per invocation. All deadlines use wall time.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:-navigation}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in motion|dropout|navigation|collision) ;; *) echo "Usage: $0 {motion|dropout|navigation|collision} [test arguments]" >&2; exit 2;; esac
player="${CI_PLAYER:-$repo_dir/Builds/CI/Linux/CiRover.x86_64}"
output="${CI_LOG_DIR:-$repo_dir/Logs/ci-$mode}"
[[ -f "$player" ]] || { echo "Missing Linux player: $player" >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "${ROVER_WS:-$HOME/rugged_rover_ws}/install/setup.bash"
# Underlays were sourced explicitly above. Avoid replaying a captured CI temp path.
source "${FAULT_INJECTION_WS:-$HOME/fault_injection_ws}/install/local_setup.bash"
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
export CI_PROCESS_FAILURE_FILE="$output/process-failure.txt"
rm -f -- "$CI_PROCESS_FAILURE_FILE"
required_pids=()
required_names=(ros_launch fault_injector unity_player)
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
required_pids+=("$!")
setsid ros2 launch ros2_fault_injection fault_injector.launch.py scenario_file:="$scenario" > "$output/injector.log" 2>&1 &
children+=("$!")
required_pids+=("$!")
player_args=()
case "${CI_COLLISION_FIXTURE:-baseline}" in
 baseline) ;;
 wall) player_args+=(--ci-wall-contact);;
 missing) player_args+=(--ci-missing-collision);;
 *) echo 'CI_COLLISION_FIXTURE must be baseline, wall or missing' >&2; exit 2;;
esac
setsid "$player" -batchmode -nographics "${player_args[@]}" -logFile "$output/unity.log" > "$output/player-console.log" 2>&1 &
children+=("$!")
required_pids+=("$!")
case "$mode" in
 motion) test_args=(test_ci_rover_motion.py --ready-timeout 60 --travel-time 2);;
 dropout) test_args=(test_ci_scan_dropout.py);;
 collision) test_args=(test_ci_collision.py --output "$output");;
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
required_exit_pattern='\[ERROR\].*process has died|\[SLAM\].*(failed|error processing)|\[(async_slam_toolbox_node|ros2_control_node|default_server_endpoint|ekf_node|controller_server|planner_server|bt_navigator|collision_monitor|robot_state_publisher|fault_injector)[^]]*\]: process has finished'
while kill -0 "$test_pid" 2>/dev/null; do
  reason=""
  for i in "${!required_pids[@]}"; do
    pid="${required_pids[$i]}"
    if ! kill -0 "$pid" 2>/dev/null; then
      child_status=0
      wait "$pid" || child_status=$?
      reason="Required process ${required_names[$i]} exited (status=$child_status); inspect ros.log, injector.log, unity.log and player-console.log"
      break
    fi
  done
  # ros2 launch may stay alive after one of its required nodes has died.
  if [[ -z "$reason" ]] && grep -Eq "$required_exit_pattern" "$output/ros.log" "$output/injector.log"; then
    reason="Required ROS node failed; $(grep -Em1 "$required_exit_pattern" "$output/ros.log" "$output/injector.log")"
  fi
  if [[ -n "$reason" ]]; then
    printf '%s\n' "$reason" > "$CI_PROCESS_FAILURE_FILE.tmp"
    mv -- "$CI_PROCESS_FAILURE_FILE.tmp" "$CI_PROCESS_FAILURE_FILE"
    echo "::error title=Required process exited::$reason"
    result=1
    # Navigation reads the failure file and writes its reports. Other modes
    # receive SIGINT after a short reporting grace period.
    for ((j=0; j<30; j++)); do
      kill -0 "$test_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -INT -- "-$test_pid" 2>/dev/null || true
    break
  fi
  sleep 0.1
done
test_status=0
wait "$test_pid" || test_status=$?
[[ "$result" != 0 ]] || result="$test_status"
wait "$log_pid" || true
echo "Logs: $output"
exit "$result"
