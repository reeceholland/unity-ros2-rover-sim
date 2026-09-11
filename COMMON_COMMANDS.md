# Rugged Rover Common Commands

## Workspace

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Build everything:

```bash
colcon build --symlink-install
```

Build one package:

```bash
colcon build --symlink-install --packages-select rugged_rover_bringup
```

Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Bringup

Real rover:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py
```

Real rover with EKF:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py use_ekf:=true
```

Real rover without SLAM:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py use_slam:=false
```

Unity simulation:

```bash
~/rugged_rover_ws/launch_unity_sim.sh
```

SLAM only:

```bash
ros2 launch rugged_rover_bringup slam.launch.py
```

Nav2 only:

```bash
ros2 launch rugged_rover_bringup nav2.launch.py
```

RViz:

```bash
ros2 launch rugged_rover_robot_description sim_rviz.launch.py
```

## Manual Driving

Forward using `TwistStamped`:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: 0.1
  angular:
    z: 0.0
"
```

Turn on the spot:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: 0.0
  angular:
    z: 0.5
"
```

Stop:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: base_link
twist:
  linear:
    x: 0.0
  angular:
    z: 0.0
"
```

Direct motor command:

```bash
ros2 topic pub --once /platform/motors/cmd sensor_msgs/msg/JointState "
name:
- front_left_joint
- front_right_joint
- rear_left_joint
- rear_right_joint
velocity:
- 2.0
- 2.0
- 2.0
- 2.0
"
```

## Diagnostics

List topics:

```bash
ros2 topic list
```

List nodes:

```bash
ros2 node list
```

Controller states:

```bash
ros2 control list_controllers
```

Topic details:

```bash
ros2 topic info /diff_drive_controller/cmd_vel -v
```

Check odom rate:

```bash
ros2 topic hz /odom
```

Check scan rate:

```bash
ros2 topic hz /scan
```

Check IMU rate:

```bash
ros2 topic hz /imu/data
```

Echo odom position:

```bash
ros2 topic echo /diff_drive_controller/odom --field pose.pose.position
```

Echo platform debug:

```bash
ros2 topic echo /platform/debug
```

Battery voltage:

```bash
ros2 topic echo /battery/voltage
```

Diagnostics:

```bash
ros2 topic echo /diagnostics
```

## TF

View TF tree:

```bash
ros2 run tf2_tools view_frames
```

Echo transform:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Monitor transform:

```bash
ros2 run tf2_ros tf2_monitor odom base_link
```

Laser transform:

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

## micro-ROS / Teensy

Run micro-ROS agent on Pi UART:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 115200
```

Fix UART permission temporarily:

```bash
sudo chmod 666 /dev/ttyAMA0
```

Check serial devices:

```bash
ls -l /dev/ttyAMA* /dev/ttyS* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

## RPLIDAR

Launch RPLIDAR:

```bash
ros2 launch rugged_rover_bringup rplidar_s2.launch.py
```

Launch RPLIDAR with explicit port:

```bash
ros2 launch rugged_rover_bringup rplidar_s2.launch.py serial_port:=/dev/rplidar
```

Check scan:

```bash
ros2 topic echo /scan --once
```

## IMU

Read IMU serial with minicom:

```bash
minicom -D /dev/ttyACM0 -b 115200
```

Find process using IMU port:

```bash
sudo lsof /dev/ttyACM0
```

Kill serial monitor processes if they are holding the port:

```bash
pkill screen
pkill minicom
```

## Stop ROS Processes

Soft stop common ROS processes:

```bash
pkill -f ros_tcp_endpoint
pkill -f slam_toolbox
pkill -f rviz2
pkill -f ros2_control_node
pkill -f robot_state_publisher
pkill -f ekf_node
pkill -f micro_ros_agent
```

Restart ROS daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

## SSH

SSH to ROS laptop:

```powershell
ssh ros-laptop
```

SSH with explicit key:

```powershell
ssh -i "$env:USERPROFILE\.ssh\id_ed25519_unity_ros" reece@192.168.68.73
```

Check SSH resolved host:

```powershell
ssh -G ros-laptop | findstr /i "hostname user identityfile"
```

Remove old host key:

```powershell
ssh-keygen -R ros-laptop
ssh-keygen -R 192.168.68.73
```

## Network

Show IP addresses:

```bash
hostname -I
```

Check Wi-Fi link:

```bash
iw dev wlan0 link
```

Ping via Wi-Fi:

```bash
ping -I wlan0 -c 3 github.com
```

Check routes:

```bash
ip route
```

## Raspberry Pi Health

Check throttling:

```bash
vcgencmd get_throttled
```

CPU and memory load:

```bash
htop
```

Time sync:

```bash
timedatectl
chronyc tracking
```
