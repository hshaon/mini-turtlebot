# Mini-TurtleBot
This repository contains the code and firmware used in the IROS submission
**“Mini-TurtleBot: A Modular, Table-Top Robotic Platform for Multi-Level Robotics Education with AI-Assisted Behavior Generation”** (title anonymized in the paper if required by the review process).

The goal of this artifact is to make it easy for reviewers to:
- Build and run the **embedded firmware** on the table-top robot base.
- Bring up the robot in **ROS 2 mode** (research stack: SLAM, Nav2, multi-robot, etc.).
- Run the **ROS-free Scratch/AI control path** used for natural-language behaviors.
- Inspect the layered architecture that separates transport, safety, and behavior generation.

> This README is anonymized: no author names, institution names, or external repository URLs are included.
> All paths below are relative to the repository root (referred to as `<REPO_ROOT>`).

---

## 1. Repository Layout

```text
<REPO_ROOT>/
├── firmware/                 # ESP32-S3 firmware for the robot base and sensor head
│   ├── base/                 # Base controller (motors, IMU, IR, ToF, battery, LiDAR UDP)
│   └── head/                 # Optional “turtle head” (camera, mic, indicators)
├── atrbridge_core/           # C++ transport core (TCP/UDP) + optional WS/JSON gateway
├── ros2_ws/                  # ROS 2 workspace (research-mode stack)
│   ├── src/
│   │   ├── tb_interfaces/    # Custom messages & services (IR state, telemetry, etc.)
│   │   ├── roslinker/        # ROS 2 node that hosts the transport core in-process
│   │   ├── tb_localization/  # EKF-based state estimation (odometry + IMU + range)
│   │   ├── tb_bringup/       # Launch files and bringup utilities
│   │   └── ...               # Additional packages as described in the paper
├── ai_pipeline/              # ROS-free AI control pipeline (Scratch/AI path)
│   ├── ailinker_standalone.py
│   ├── ai_behavior_gen.py
│   ├── ai_control_cli.sh
│   ├── start_rosfree_stack.sh
│   └── ...
└── docs/                     # Schematics, CAD, and architecture diagrams (if included)
```

---

## 2. Hardware & Software Prerequisites

**Robot hardware (table-top platform)**
- ESP32-S3-based controller board (robot base)
- Differential drive motors with encoders
- IMU + IR + ToF range sensors
- Optional 360° LiDAR (UDP) and detachable camera/mic “head”
- 1S Li-Po battery and charger
- Onboard OLED (for robot ID and status) – optional but recommended

**Host machine**
- Ubuntu 22.04 (or similar Linux distribution)
- Wi‑Fi network shared with the robot
- C++17 toolchain (`gcc`/`clang`), `cmake`, `git`, `python3`

**ROS 2 stack (research mode)**
- ROS 2 **Jazzy** (or a compatible recent ROS 2 distribution)
- `colcon` and standard ROS 2 build tools

**AI stack (ROS-free mode, optional)**
- Python 3.10+
- Ability to create a virtual environment (`python3 -m venv`)
- API key for your chosen LLM provider (e.g., set via `GEMINI_API_KEY`); the paper uses a cloud LLM, but any compatible backend can be wired into `ai_behavior_gen.py`.

---

## 3. Firmware Overview (ESP32-S3)

The `firmware/` directory contains the microcontroller code for the robot.

- `firmware/base/` handles:
  - Wheel encoder integration and closed-loop velocity control
  - IMU, IR, ToF sensor polling
  - LiDAR UDP forwarding (if LiDAR is attached)
  - Battery monitoring and motor safety
  - Binary TCP command/telemetry protocol used by the transport core

- `firmware/head/` (optional) handles:
  - Camera capture (e.g., MJPEG over HTTP)
  - Microphone audio streaming
  - Indicator LEDs / buzzer

For IROS review, pre-flashed hardware is sufficient. If you wish to rebuild the firmware,
open the corresponding project in your ESP32 development environment (e.g., Arduino
or ESP-IDF) and flash it to the board connected to the robot. The protocol exposed
by the firmware matches the transport core used in both ROS 2 and ROS-free modes.

---

## 4. Building the Transport Core (`atrbridge_core/`)

The transport core is a small C++ layer that speaks the robot’s binary protocol over TCP
(control + telemetry) and UDP (LiDAR). It can be used in two ways:

1. Linked **in-process** with a ROS 2 node (`roslinker`)
2. As a standalone **WebSocket/JSON gateway** for Scratch/AI clients

From `<REPO_ROOT>`:

```bash
cd atrbridge_core

# Configure (enable the WS/JSON gateway target)
cmake -S . -B build -DATRBRIDGE_BUILD_WS_GATEWAY=ON

# Build
cmake --build build -j
```

This produces at least the following binaries:

- `build/atrbridge_core`
- `build/atrbridge_core_daemon`
- `build/atrbridge_ws_gateway`

---

## 5. ROS 2 Mode (Research Stack)

In ROS 2 mode, the robot appears as a standard ROS 2 node graph with topics/services for
`cmd_vel`, IMU, range sensors, and LiDAR. The `roslinker` node owns the single TCP
connection to the robot and integrates the transport core in-process.

### 5.1 Build the ROS 2 workspace

From `<REPO_ROOT>`:

```bash
# Source your ROS 2 installation (adjust distro/path as needed)
source /opt/ros/jazzy/setup.bash

# Build the workspace
cd ros2_ws
colcon build

# Source the overlay so custom messages/services are visible
source install/setup.bash
```

If you only wish to build the minimal subset required for the core demo:

```bash
colcon build --packages-select tb_interfaces roslinker tb_bringup
source install/setup.bash
```

### 5.2 Launch the robot (ROS 2 bringup)

With the robot powered on and connected to the same Wi‑Fi network as the host the OLED of the robot will show <ROBOT_IP>, then run:

```bash
# Still in <REPO_ROOT>/ros2_ws with your overlay sourced
ros2 launch roslinker bringup.launch.py robot_id:=tb_01 robot_ip:=<ROBOT_IP> port:=9000       lidar_udp_port:=5601
```

Expected behavior:
- The robot OLED updates to show the assigned `robot_id` (e.g., `tb_01`).
- The ROS 2 graph now exposes topics in the `/tb_01` namespace.

You can verify:

```bash
ros2 node list | grep tb_01
ros2 topic list | grep /tb_01
```

Typical topics include:

- `/tb_01/cmd_vel`
- `/tb_01/imu/data_raw`
- `/tb_01/ir/state`
- `/tb_01/tof/range`
- `/tb_01/lidar/scan` (if LiDAR is enabled in the firmware)

### 5.3 Quick command test

To confirm the control path:

```bash
ros2 topic pub /tb_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 5
```

To stop the robot:

```bash
ros2 topic pub /tb_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 5
```

Higher-level stacks (SLAM, localization EKF, Nav2, multi-robot coordination) can be
launched on top of this ROS 2 interface as described in the paper. The exact launch
configurations for those stacks are left to the user’s preferred SLAM/Nav2 setup.

---

## 6. Non-ROS Mode: WS/JSON Gateway (Scratch / AI Clients)

In non-ROS mode, the transport core runs as a standalone gateway and exposes a simple
WebSocket + JSON API. Both Scratch-based GUIs and the AI pipeline use this mode.

### 6.1 Run the WS/JSON gateway

From `<REPO_ROOT>/atrbridge_core`:

```bash
./build/atrbridge_ws_gateway <ROBOT_IP> 9000 5601 8080 tb_01
```

Arguments:
1. `robot_ip` – ESP32 IP of the robot
2. `tcp_port` – robot TCP server port (control + telemetry)
3. `lidar_udp_port` – UDP port to receive LiDAR packets
4. `ws_port` – WebSocket server port for clients
5. `robot_id` – logical ID (default `tb_01`)

On startup you should see logs similar to:

```text
atrbridge_ws_gateway listening on ws://0.0.0.0:8080/ws
[core] tcp_connected=true
[core] SET_ID sent: tb_01
```

WebSocket clients then connect to:

- `ws://localhost:8080/ws` (same machine)
- `ws://<HOST_IP>:8080/ws` (over the network)

The JSON protocol supports messages such as `get_capabilities`, `cmd_vel`, `estop`, and `poll`.
The exact message formats correspond to the examples used in the paper.

---

## 7. ROS-Free AI Pipeline

The `ai_pipeline/` directory implements the ROS-free AI control path described in the paper.

At a high level:

```text
Robot  <->  atrbridge_ws_gateway  <->  ailinker_standalone  <->  ai_behavior_gen
                                        (safety + tools)        (LLM + tools)
```

### 7.1 One-time setup

From `<REPO_ROOT>`:

```bash
cd ai_pipeline

# Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install Python dependencies
pip install -U pip setuptools
pip install -r requirements.txt

# Set your LLM API key (example environment variable)
export GEMINI_API_KEY="YOUR_API_KEY_HERE"
```

### 7.2 Start the ROS-free stack

With the firmware running and the robot reachable on the network:

```bash
cd <REPO_ROOT>/ai_pipeline
source .venv/bin/activate

./start_rosfree_stack.sh --robot-ip <ROBOT_IP> --robot-id tb_01
```

This script starts:

- `atrbridge_ws_gateway` (WebSocket/JSON gateway)
- `ailinker_standalone` (safety layer + tool server)
- `ai_behavior_gen` (LLM-based behavior generator)

You can stop the stack via:

```bash
./stop_rosfree_stack.sh
```

### 7.3 Interactive AI control (CLI)

To issue natural-language commands from a terminal:

```bash
cd <REPO_ROOT>/ai_pipeline
source .venv/bin/activate
export GEMINI_API_KEY="YOUR_API_KEY_HERE"

./ai_control_cli.sh
```

Example tasks:

- `move forward for 2 seconds, then stop`
- `turn right in place for 1 second, then stop`
- `move forward for 0.5 second, turn right 90 degrees, repeat this to make a square`
- `get current capabilities`

The AI layer internally uses a set of tools to ensure safety (bounded velocity/acceleration,
timeouts, and emergency stops), as described in the paper’s architecture section.

### 7.4 One-shot task execution

For scripted tests of a single task:

```bash
cd <REPO_ROOT>/ai_pipeline
source .venv/bin/activate
export GEMINI_API_KEY="YOUR_API_KEY_HERE"

./run_task.sh --task "move forward for 2 seconds, then stop"
```

This sends a single natural-language task through the same toolchain used in the interactive CLI.

> Note: This README intentionally omits experimental metric collection and benchmarking
> procedures. Any auxiliary scripts related to metrics in the repository are provided only
> as internal utilities and are not required to reproduce the qualitative behaviors shown
> in the paper.

---

## 8. Reproducing the Main Qualitative Demos

The paper describes several qualitative behaviors (e.g., point-to-point motions, simple
sequences, and basic navigation-like tasks). At a high level, reviewers can reproduce
similar behaviors by following these steps:

1. **Bring up the robot in ROS 2 mode** (Section 5) and confirm `/cmd_vel` control.
2. **Optionally run SLAM/Nav2** on top of the published topics, using your preferred
   ROS 2 navigation configuration on a small table-top map.
3. **Bring up the ROS-free AI pipeline** (Section 7) with the WS/JSON gateway.
4. Use `ai_control_cli.sh` or `run_task.sh` to issue the example natural-language tasks
   and observe the robot motion in a safe, open table-top area.

Exact numeric performance metrics (latency distributions, etc.) are **not** required for
understanding or validating the system design and are therefore excluded from this README
to keep the artifact focused on architecture and functionality.

---

## 9. Common Issues and Debugging Tips

- **No topics or no sensor updates in ROS 2 mode**
  - Confirm `roslinker` is still running.
  - Check that `robot_ip` and ports match the firmware configuration.
  - Ensure only one process is connected to the robot TCP port (don’t run ROS and WS gateway simultaneously).

- **WebSocket gateway reports `tcp_connected=false`**
  - Verify the robot is powered and reachable (`ping <ROBOT_IP>`).
  - Check that no other bridge (`roslinker` or another gateway) is running.

- **No motion when sending `/cmd_vel` or AI commands**
  - Confirm motors are enabled in firmware (no emergency stop active).
  - Verify power and battery voltage.
  - Ensure wheel directions and encoder signs are correct in firmware (see comments in the `firmware/base` code).

For any additional, low-level debugging, please refer to the inline comments in the source
files under `atrbridge_core/`, `ros2_ws/src/`, and `firmware/`.
