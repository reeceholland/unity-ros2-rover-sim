#!/usr/bin/env bash
# Run on the build server; creates a separate persistent overlay, never deletes CI files.
set -eo pipefail
rover_ws="${ROVER_WS:-$HOME/rover-debug/rover_ws}"
fault_source="${FAULT_SOURCE:-$HOME/actions-runner/_work/ros2_fault_injection/ros2_fault_injection}"
fault_ws="${PERSISTENT_FAULT_WS:-$HOME/rover-debug/fault_ws}"
[[ -f "$rover_ws/install/local_setup.bash" ]] || { echo "Missing persistent rover install: $rover_ws"; exit 2; }
[[ -d "$fault_source/.git" ]] || { echo "FAULT_SOURCE must name the fault-injection repository"; exit 2; }
mkdir -p "$fault_ws/src"
if [[ ! -d "$fault_ws/src/ros2_fault_injection" ]]; then
  git clone --no-hardlinks "$fault_source" "$fault_ws/src/ros2_fault_injection"
fi
# Start with no AMENT/CMAKE/COLCON environment inherited from a deleted CI underlay.
env -i HOME="$HOME" USER="$(id -un)" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  ROVER_WS="$rover_ws" FAULT_WS="$fault_ws" bash --noprofile --norc <<'BUILD'
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$ROVER_WS/install/local_setup.bash"
cd "$FAULT_WS"
colcon build --symlink-install --packages-up-to ros2_fault_injection
# setup.bash is deliberately tested, not just local_setup.bash: its saved
# parent chain must now refer to the persistent rover install.
source "$FAULT_WS/install/setup.bash"
python3 -c 'from ros2_fault_injection.srv import SetFaultState; print("Fault overlay import OK")'
if grep -R -n --include='setup.*' '_temp/rover_ws' "$FAULT_WS/install"; then
  echo 'ERROR: stale CI underlay remains' >&2
  exit 1
fi
BUILD
printf '\nUse for subsequent runs:\nexport ROVER_WS=%q\nexport FAULT_INJECTION_WS=%q\n' "$rover_ws" "$fault_ws"
