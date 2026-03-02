# ROS2 Graph Test (New Architecture)

> Note: The ROS-free AI pipeline is now located at `/home/shaon/mini-turtlebot/ai_pipeline`.

This guide describes how to bring up the **ROS pipeline** using the **new architecture** where:

- **`roslinker`** is the only process that should own the robot TCP connection during ROS testing.
- `roslinker` starts the ATRBridge core **in-process** (fast path).
- **Do NOT run** `atrbridge_core_daemon` at the same time as `roslinker` (both will try to connect to the robot over TCP).

---

## Architecture Summary (ROS Mode)

```
ROS2 Graph  <-->  ROSLinker (ROS2 node)
                    |
                    | in-process API (libatrbridge_core)
                    v
              ATRBridge Core (in-process)
                    |
                    | TCP (binary frames): control + telemetry
                    | UDP (binary): LiDAR stream
                    v
                ESP32 TurtleBot
```

---

## Prerequisites

- Robot powered on, ESP32 firmware running
- Robot reachable over Wi-Fi (current IP: **192.168.0.78**)
- TCP port: **9000**
- LiDAR UDP port: **5601** (if LiDAR enabled)
- ROS 2 Jazzy installed: `/opt/ros/jazzy`

> If you previously ran an older bridge (e.g., `tb_bridge_cpp`), stop it first.

---

## 1) Build + Source Workspace

```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws

# Build only what you need for the ROS pipeline
colcon build --packages-select tb_interfaces roslinker

# Source overlays (IMPORTANT for custom msgs like IRState)
source install/setup.bash
```

---

## 2) Launch ROSLinker (keeps ATRBridge core in-process)

**Terminal A (keep running):**

```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws
source install/setup.bash

ros2 launch roslinker bringup.launch.py \
  robot_id:=tb_01 robot_ip:=192.168.0.78 port:=9000 lidar_udp_port:=5601
```

### What you should see
- A log line similar to:

```
[tb_01.roslinker]: roslinker started robot_id=tb_01 ip=192.168.0.78 port=9000 lidar_udp=5601
```

- After the recent fix, you should also see a line like:
```
SET_ID sent: tb_01
```

✅ The robot OLED should switch to:
```
ID: tb_01
```

If OLED still shows `ID: ---`, see Troubleshooting below.

---

## 3) Verify ROS Graph + Topics

**Terminal B:**

```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws
source install/setup.bash

ros2 node list | rg tb_01
ros2 topic list | rg /tb_01
```

Expected topics:
- `/tb_01/cmd_vel`
- `/tb_01/imu/data_raw`
- `/tb_01/ir/state`
- `/tb_01/tof/range`
- `/tb_01/lidar/scan` (if LiDAR enabled)

---

## 4) Check Connection State (Capability Service)

This is the fastest way to confirm ATRBridge core is actually connected to the robot.

```bash
ros2 service call /tb_01/bridge/get_capabilities tb_interfaces/srv/GetCapabilities "{}"
```

Look for something like:
- `tcp_connected: true`
- capability flags for sensors/modules

If `tcp_connected` is false, the ROS graph may still exist, but no live sensor updates will stream.

---

## 5) Configure Telemetry (Recommended)

Depending on your default firmware behavior, you may need to enable/adjust telemetry rates after startup.

```bash
ros2 service call /tb_01/bridge/set_telemetry tb_interfaces/srv/SetTelemetry \
"{enable_imu: true, imu_rate_hz: 50.0,
  enable_ir: true, ir_rate_hz: 10.0,
  enable_tof: true, tof_rate_hz: 10.0,
  enable_lidar: true, lidar_rate_hz: 5.0, lidar_udp_port: 5601,
  enable_motor_state: false, motor_state_rate_hz: 0.0,
  enable_battery: false, battery_rate_hz: 0.0,
  profile_name: 'test'}"
```

---

## 6) Verify Live Sensor Streams

⚠️ Keep `roslinker` running in Terminal A. If you Ctrl+C the launch, sensor updates stop.

**Terminal B:**

```bash
ros2 topic echo /tb_01/imu/data_raw
ros2 topic echo /tb_01/ir/state
ros2 topic echo /tb_01/tof/range
ros2 topic echo /tb_01/lidar/scan
```

### Important: IRState message type error
If you see:
```
The message type 'tb_interfaces/msg/IRState' is invalid
```
you did not source your workspace overlay in that terminal.

Fix:
```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws
source install/setup.bash
```

Then retry `ros2 topic echo /tb_01/ir/state`.

---

## 7) Quick Command Test (`/cmd_vel`)

```bash
ros2 topic pub /tb_01/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}" -r 5
```

To stop:
```bash
ros2 topic pub /tb_01/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}" -r 5
```

---

# Troubleshooting

## A) Topics exist but values are not updating
Most common causes:

1) **You stopped roslinker.**  
   If Terminal A was closed or you pressed Ctrl+C, updates stop. Relaunch and keep it running.

2) **Not actually connected to the robot (TCP not connected).**  
   Check:
   ```bash
   ros2 service call /tb_01/bridge/get_capabilities tb_interfaces/srv/GetCapabilities "{}"
   ```
   Ensure it indicates `tcp_connected: true` (or equivalent).

3) **Telemetry not enabled or too low rate.**  
   Call `set_telemetry` (Section 5).

---

## B) OLED shows `ID: ---`
This usually indicates the robot did not receive the SET_ID message.

- This was previously caused by SET_ID being sent only at startup before TCP connect.
- Fix: `roslinker` now retries SET_ID on connect/reconnect.

What to do:
1) Relaunch `roslinker`
2) Wait for log line `SET_ID sent: tb_01`
3) OLED should update to `ID: tb_01`

If it does not:
- Verify the robot IP is correct (192.168.0.78)
- Verify no other bridge is holding the TCP connection (see next item)

---

## C) “Connection refused” / “No response” / intermittent connect
Make sure **only one** process is connecting to the robot TCP port.

Stop any old processes:
- `tb_bridge_cpp`
- `atrbridge_core_daemon`
- older bridge scripts/services

You can check using:
```bash
ps aux | rg tb_bridge_cpp
ps aux | rg atrbridge
ps aux | rg roslinker
```

---

## D) LiDAR topic exists but empty / not publishing
- Ensure LiDAR module is physically connected.
- Enable LiDAR via telemetry config:
  ```bash
  ros2 service call /tb_01/bridge/set_telemetry tb_interfaces/srv/SetTelemetry \
  "{enable_lidar: true, lidar_rate_hz: 5.0, lidar_udp_port: 5601}"
  ```
- Confirm UDP port is not blocked and matches firmware configuration.

---

## E) Fast retest (known-good sequence)

**Terminal A**
```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws
source install/setup.bash

ros2 launch roslinker bringup.launch.py \
  robot_id:=tb_01 robot_ip:=192.168.0.78 port:=9000 lidar_udp_port:=5601
```

**Terminal B**
```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws
source install/setup.bash

ros2 service call /tb_01/bridge/get_capabilities tb_interfaces/srv/GetCapabilities "{}"

ros2 service call /tb_01/bridge/set_telemetry tb_interfaces/srv/SetTelemetry \
"{enable_imu: true, imu_rate_hz: 50.0,
  enable_ir: true, ir_rate_hz: 10.0,
  enable_tof: true, tof_rate_hz: 10.0,
  enable_lidar: true, lidar_rate_hz: 5.0, lidar_udp_port: 5601}"

ros2 topic echo /tb_01/tof/range
ros2 topic echo /tb_01/imu/data_raw
ros2 topic echo /tb_01/ir/state
```

---

## Notes
- ROS pipeline testing should run **only `roslinker`**.
- When you move to Scratch/AI testing later, you will run the WS/JSON gateway path instead of ROSLinker.
