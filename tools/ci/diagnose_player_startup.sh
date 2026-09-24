#!/usr/bin/env bash
# Controlled comparison; does not assume that Xvfb explains a native crash.
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
logs="${CI_LOG_DIR:-$HOME/rover-debug/startup-comparison-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$logs"
{
  uname -a
  sha256sum "${CI_PLAYER:?Set CI_PLAYER to the Linux player executable}"
  ldd "$CI_PLAYER"
  printf 'Core pattern: '; cat /proc/sys/kernel/core_pattern
} > "$logs/environment.txt" 2>&1
ulimit -c unlimited || true
for mode in no-display xvfb; do
  status=0
  if [[ "$mode" == xvfb ]]; then
    CI_LOG_DIR="$logs/$mode" CI_COLLISION_FIXTURE=baseline xvfb-run -a bash "$here/run_headless.sh" collision || status=$?
  else
    env -u DISPLAY -u WAYLAND_DISPLAY CI_LOG_DIR="$logs/$mode" CI_COLLISION_FIXTURE=baseline \
      bash "$here/run_headless.sh" collision || status=$?
  fi
  printf '%s\n' "$status" > "$logs/$mode/exit-code.txt"
done
echo "Comparison logs: $logs"
echo 'If a native crash occurs, use coredumpctl info CiRover.x86_64 and coredumpctl debug CiRover.x86_64, then thread apply all bt.'
