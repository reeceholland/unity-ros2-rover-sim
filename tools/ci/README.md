# Headless rover CI

Run a fresh Linux Unity player, ROS bringup and the fault injector for each test.
The runner uses localhost discovery and ROS domain **190** (override with
`CI_ROS_DOMAIN_ID`), reserves TCP port 10000, and terminates only its own process
groups. Do not run a second endpoint or player on that port.

## Prerequisites

- Ubuntu/WSL with ROS Jazzy, `python3`, `flock`, `timeout`, `ss`.
- Built and sourced-compatible `~/rugged_rover_ws` and `~/fault_injection_ws`.
  Rover bringup must support `use_slam`, `use_nav2`, `use_motor_fault_injection`, `ros_tcp_ip` and simulation time.
- Unity **6000.3.14f1** with **Linux Build Support (Mono)**.
- The CI scene publishes `/clock`, `/scan_raw`, `/platform/motors/feedback`,
  and `/imu/data`, and subscribes to `/platform/motors/cmd` at localhost:10000.

Build using **Tools > CI > Build Linux Player** in Unity. Output:
`Builds/CI/Linux/CiRover.x86_64`. Distribute the entire Linux directory, including
its data folder and shared libraries. Rebuild after changing the scene/scripts.
The scripts can also test an already released player via `CI_PLAYER`; record
its release tag/checksum alongside the ROS repository versions in CI artifacts.

With the editor closed, a Windows batch build is:

```powershell
& 'C:/Program Files/Unity/Hub/Editor/6000.3.14f1/Editor/Unity.exe' -batchmode -nographics -quit -projectPath (Get-Location).Path -executeMethod BuildCiPlayer.BuildLinux -logFile ci-player-build.log
```

## Run on Linux or WSL

From the repository root:

```bash
# Short baseline: move forward for two simulated seconds and stop.
bash tools/ci/run_headless_motion.sh

# Verify raw scans continue during complete injected scan dropout and recover.
bash tools/ci/run_headless_dropout.sh

# Navigate configured goals with a timed scan-dropout recovery case.
bash tools/ci/run_headless_navigation.sh

# Supply a different route (JSON schema demonstrated by waypoints.json).
bash tools/ci/run_headless_navigation.sh --scenario /absolute/path/route.json
```

The runner explicitly enables `use_motor_fault_injection:=true`.
Motion and dropout runs disable Nav2 and SLAM. Navigation enables both.
The navigation/motion injector always forwards motor commands from
`/platform/motors/cmd_raw` to `/platform/motors/cmd` and scans from `/scan_raw`
to `/scan`. Its scan fault starts disabled. The stationary dropout test uses
`/ci/scan` so it can test forwarding in isolation.

## Waypoints and fault timing

`waypoints.json` contains **ROS map-frame** metres and yaw in degrees.
The route visits Point 1 through Point 4 in the supplied order, with a one-second
complete scan dropout at the start of Point 2. Each goal allows 180 wall-clock
seconds. Orientations are converted from the supplied planar quaternions to yaw
and normalized when sent. Confirm
the start pose, clear route and lidar transform in your built CI scene.

Unity-to-ROS axis conversion is `(x, y, z) -> (z, -x, y)` and heading changes
sign, but Unity world and SLAM map origins/orientations need not coincide.
Do not paste Unity Inspector coordinates directly into this file. Use positions
measured in the ROS map, or apply the calibrated world-to-map transform first.
For stable larger test suites, pin the scene/start pose and use a saved map with
localization; this first runner uses the existing live SLAM bringup.

Each goal has a wall-clock deadline and independent final position/yaw tolerance.
Fault `start` and `duration` are **wall-clock seconds after goal acceptance**.
Currently the supported scheduled fault is `ci_scan_dropout` (100% drop).
All fault windows must complete before success, so a trivial/too-close goal fails
instead of silently skipping the fault. Fault toggles and their simulation times
are recorded. Fixed injector seeds improve repeatability, but physics, SLAM and
asynchronous message timing are not bit-for-bit deterministic.

A pass requires action status SUCCEEDED, zero Nav2 error code, a fresh map pose
within tolerance, and completed fault windows. Aborted actions fail even if their
error code is zero. Clock stalls/resets, rejected goals, unavailable services,
and deadlines fail with nonzero exit status. Active goals are cancelled and
activated faults are disabled on normal error/interrupt cleanup; the runner has
a final bounded process-group shutdown as a fallback.

Pose checks use the navigation TF estimate, **not independent Unity ground truth**.
This tests navigation completion/recovery, not physical accuracy or proof of safe
stopping during dropout. Add a ground-truth pose oracle and explicit stop-distance
assertions before using these results to assess safety behavior.

## Results and configuration

Logs are retained in `Logs/ci-motion`, `Logs/ci-dropout` or `Logs/ci-navigation`.
Navigation also writes `results.json` (goal outcomes, errors, fault events) and
`junit.xml` for CI report ingestion. Archive the entire log directory even on
failure. A forced kill/startup shell failure may prevent Python reports; the
runner exit status must always be checked. Give each CI job a unique `CI_LOG_DIR`.

Environment overrides:

| Variable | Default |
| --- | --- |
| `CI_PLAYER` | `Builds/CI/Linux/CiRover.x86_64` |
| `ROVER_WS` | `~/rugged_rover_ws` |
| `FAULT_INJECTION_WS` | `~/fault_injection_ws` |
| `CI_LOG_DIR` | `Logs/ci-<mode>` |
| `CI_ROS_DOMAIN_ID` | `190` |
| `CI_TIMEOUT` | `900s` (whole test wall-clock limit) |

The existing motion/dropout Python clients remain usable against a manually
started simulation; run their `--help` for overrides. Motion smoke results are
wheel-odometry based. The wrapper explicitly uses a short two-second movement;
the standalone motion client retains its existing 30-second default.

Run ROS-independent scenario validation tests:

```bash
python3 -m unittest discover -s tools/ci -p test_navigation_scenario.py
```

Release archives and checksums are local artifacts; keep them outside tracked
source (for example under ignored `Builds/releases`) and upload them as GitHub
release assets. Headless testing does not require the Unity editor on the server.

Verbose navigation output streams to the terminal and `test.log`: goal coordinates,
acceptance, current pose/quaternion, distance remaining, navigation time, ETA,
recoveries, fault changes and final pose errors. Feedback defaults to once per
second; to print every Nav2 feedback message, run:

```bash
bash tools/ci/run_headless_navigation.sh --feedback-interval 0
```
