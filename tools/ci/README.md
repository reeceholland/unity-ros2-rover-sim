# CI rover motion smoke test

Open `Assets/Scenes/CiLidarWorld.unity`. In Ubuntu WSL, source ROS and start
`ros2 launch rugged_rover_bringup unity_sim.launch.py`, then press Play.
Do not run Nav2, teleop, or another command publisher during this test.
The test commands real motion in the simulator; start the rover in the clear
centre of the CI world.

In another WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rugged_rover_ws/install/setup.bash
python3 /mnt/c/Users/reece_pu0ewge/Desktop/Unity/unity-ros2-rover-sim/tools/ci/test_ci_rover_motion.py
```

The test waits up to 30 wall-clock seconds for advancing simulation time,
valid scans, odometry, named wheel feedback and a velocity-command subscriber.
It commands 0.15 m/s for two simulated seconds, checks 0.10–0.60 m forward
odometry displacement, verifies scans continue and checks wheel speeds settle
below 0.1 rad/s. It publishes stop commands on success, failure and Ctrl+C.
Exit status is 0 for pass and nonzero for failure. A paused clock or missing
sensor cannot leave the test waiting indefinitely.

Topic names and wall-clock timeouts can be overridden; run with `--help`.
This checks ROS odometry, not independent Unity ground truth. Headless player
building and automatic process startup are not included yet.

## Linux headless player

Install **Linux Build Support (Mono)** for Unity **6000.3.14f1** through Unity
Hub. In the editor select **Tools > CI > Build Linux Player**. The output is
`Builds/CI/Linux/CiRover.x86_64`; distribute the entire Linux folder, including
its data directory and shared libraries.

With the editor closed, the equivalent batch build is:

```powershell
& 'C:/Program Files/Unity/Hub/Editor/6000.3.14f1/Editor/Unity.exe' -batchmode -nographics -quit -projectPath 'C:/Users/reece_pu0ewge/Desktop/Unity/unity-ros2-rover-sim' -executeMethod BuildCiPlayer.BuildLinux -logFile ci-player-build.log
```

Stop Play Mode and any existing ROS endpoint, then in WSL:

```bash
bash /mnt/c/Users/reece_pu0ewge/Desktop/Unity/unity-ros2-rover-sim/tools/ci/run_headless_motion.sh
```

This uses ROS domain 190 by default, starts ROS and the player, runs the motion
test and shuts down its process groups. Logs are saved under `Logs/ci-headless`.
Override `CI_PLAYER`, `ROVER_WS`, `CI_LOG_DIR` or `ROS_DOMAIN_ID` if needed.
The rover ROS workspace must already be built. Fault-injection integration is
separate from this baseline motion test.

## Headless fault-injection test

With the Linux player built and both ROS workspaces installed, run in WSL:

```bash
bash /mnt/c/Users/reece_pu0ewge/Desktop/Unity/unity-ros2-rover-sim/tools/ci/run_headless_dropout.sh
```

The script starts the same headless world plus the fault injector, checks normal
forwarding from `/scan_raw` to `/ci/scan`, enables 100% dropout while confirming
raw scans continue, then disables the fault and checks matching message stamps
resume. It does not move the rover. Logs are under `Logs/ci-dropout` and failures
return a nonzero exit code. `FAULT_INJECTION_WS` overrides the default
`~/fault_injection_ws`. Stop other ROS endpoints before running.
