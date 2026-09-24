# Headless resilience validation

The runner requires fresh clock, raw/injected scans, map TF, the Nav2 action,
fault service, wheel commands, ground truth and collision heartbeat. Each is
logged separately. Required process exits fail the run and produce a GitHub
annotation. Navigation writes `results.json`, `junit.xml`, `summary.md` and
observer reports; GitHub Actions also receives the Markdown summary directly.

## Stopping contract

- Collision Monitor rejects scans older than 0.5 simulation seconds.
- Wheel commands must reach zero within another 0.2 seconds (one 5 Hz controller
  cycle, including downstream delivery).
- Physical speed must be within 0.02 m/s and 0.02 rad/s after another 0.5 seconds.
- Both states must remain stopped for at least 0.2 seconds and until healthy
  scans return. A transient zero during braking is not a confirmed stop.
- Maximum integrated ground-truth travel during the stop is 0.30 m. At the
  configured 0.20 m/s maximum speed, 0.7 s detection/command latency plus 0.5 s
  braking at full speed would travel 0.24 m, leaving 0.06 m measurement margin.

The runner reads Collision Monitor's live `source_timeout` and refuses to run
if it differs from the observer configuration. These are simulation acceptance
limits; changes to speed or braking require reviewing the distance budget too.

## Fault and recovery policy

Injection requires one continuous simulation second of fresh commanded and
measured motion after the new goal is accepted. An early Nav2 abort does not
truncate an activated fault window. After restoring scans, the runner waits for
three healthy scans and fresh map TF, then retries an aborted goal once within
the original overall goal deadline. Both attempts are retained in JSON and the
GitHub summary. A rejected/cancelled goal, failed retry, missing recovery or
observer failure still fails CI. This tests an explicit test-runner retry policy;
it does not claim the rover autonomously resumes an aborted goal in production.

## Build-server workspace repair

First copy the source changes into the persistent server checkouts and rebuild
`rugged_rover_bringup`. Then, on the server:

```bash
cd ~/rover-debug/unity-ros2-rover-sim
export ROVER_WS="$HOME/rover-debug/rover_ws"
bash tools/ci/rebuild_persistent_overlay.sh
export FAULT_INJECTION_WS="$HOME/rover-debug/fault_ws"
```

The repair builds a separate persistent fault overlay in a clean environment.
It copies committed source from the existing runner checkout with a local Git
clone; it does not delete the Actions workspace. It sources the resulting full
`setup.bash` and checks for the obsolete `_temp/rover_ws` reference.
Override `FAULT_SOURCE` if the old checkout is elsewhere. Install any missing
dependencies reported by colcon before retrying.

## Run navigation and collision validation

Set `CI_PLAYER` to a rebuilt Linux player containing `CiCollisionFixture`.

```bash
export CI_PLAYER="$HOME/rover-debug/ci-player/CiRover.x86_64"
export CI_LOG_DIR="$HOME/rover-debug/navigation-$(date +%Y%m%d-%H%M%S)"
xvfb-run -a bash tools/ci/run_headless_navigation.sh --dropout-duration 5 --feedback-interval 5

export CI_LOG_DIR="$HOME/rover-debug/collision-validation-$(date +%Y%m%d-%H%M%S)"
bash tools/ci/validate_collision.sh
```

The collision suite starts three separate simulations: stationary on the floor
(must pass), an opt-in static wall overlapping the chassis after 15 simulation
seconds (must fail for that named wall), and deliberately suppressed telemetry
(must fail for missing telemetry). The wall exercises real Unity collision
callbacks, not a synthetic ROS collision message. Individual negative runs
return 1; the validation harness returns 0 only if all three expected outcomes
are observed. A setup error cannot satisfy a negative test.

## Investigate the native startup crash

Keep Xvfb for normal server runs. Collect a controlled comparison separately:

```bash
export CI_LOG_DIR="$HOME/rover-debug/startup-comparison-$(date +%Y%m%d-%H%M%S)"
bash tools/ci/diagnose_player_startup.sh
```

This uses the same binary and baseline with no display and with Xvfb, saves
exit codes, library/kernel details and Unity/ROS logs, and permits core dumps.
If a crash recurs, inspect `coredumpctl info CiRover.x86_64`, then
`coredumpctl debug CiRover.x86_64` and `thread apply all bt` in the debugger.
If the host does not use systemd-coredump, use the recorded kernel core pattern
to locate the core instead. A successful Xvfb run alone does not identify the
native crash's cause.
