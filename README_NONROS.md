# Non-ROS Mode (Scratch / AI): WS/JSON Gateway Bringup

This guide explains how to run Mini-TurtleBot in **non-ROS mode** for Scratch/AI clients using the
**WS/JSON gateway**. In this mode, you do **not** need ROS 2 installed.

> **Important:** Do not run `roslinker` and the WS gateway against the same robot at the same time.
> Both will attempt to open the single TCP connection to the ESP32.

---

## Architecture Summary (Non-ROS Mode)

```
Scratch GUI  -> ScratchLinker  --WS/JSON-->  atrbridge_ws_gateway
AI Client    ->   AILinker     --WS/JSON-->  atrbridge_ws_gateway
                                            |
                                            | in-process ATRBridgeCore
                                            v
                                      TCP (binary frames) + UDP (LiDAR)
                                            v
                                         ESP32 TurtleBot
```

- **Southbound (robot <-> core):** TCP binary frames (control + telemetry), UDP binary (LiDAR)
- **Northbound (clients <-> gateway):** WebSocket + JSON

---

## Prerequisites

- Robot powered on, ESP32 firmware running
- Robot reachable over Wi-Fi
  - Current Robot IP: **192.168.0.78**
  - TCP port: **9000**
  - LiDAR UDP port: **5601**
- Build tools available: `cmake`, `make`/`ninja`, C++ toolchain

---

## Build the Core + Gateway Binaries

```bash
cd /home/shaon/mini-turtlebot/atrbridge_core

# Configure (enable WS gateway)
cmake -S . -B build -DATRBRIDGE_BUILD_WS_GATEWAY=ON

# Build
cmake --build build -j
```

Expected targets include:
- `atrbridge_core`
- `atrbridge_core_daemon`
- `atrbridge_ws_gateway`

---

## Run the WS/JSON Gateway (Non-ROS Entrypoint)

```bash
./build/atrbridge_ws_gateway 192.168.0.78 9000 5601 8080 tb_01
```

Arguments:
1) `robot_ip` (ESP32 IP)  
2) `tcp_port` (robot TCP server port)  
3) `lidar_udp_port` (UDP port to receive LiDAR packets)  
4) `ws_port` (WebSocket server port)  
5) `robot_id` (optional; defaults to `tb_01`)

### What you should see

Startup:
```
atrbridge_ws_gateway listening on ws://0.0.0.0:8080/ws
```

On successful robot link:
```
[core] tcp_connected=true
[core] SET_ID sent: tb_01
```

✅ The robot OLED should switch from:
```
ID: ---
```
to:
```
ID: tb_01
```

If you see:
```
[core] tcp_connected=false
```
then the gateway is not connected to the robot (check IP, Wi-Fi, and that no other bridge is running).

---

## Connect a Client (WebSocket URL)

Clients connect to:

- On the same machine:
  - `ws://localhost:8080/ws`
- From another machine:
  - `ws://<host-ip>:8080/ws`

---

## Supported WS/JSON Messages (Current)

Send JSON frames with a `type` field.

### 1) Get capabilities
Request:
```json
{"type":"get_capabilities"}
```

Response (example shape):
```json
{"type":"capabilities","modules":{...},"limits":{...},"runtime":{...}}
```

### 2) Command velocity
```json
{"type":"cmd_vel","vx":0.1,"wz":0.2}
```

### 3) Emergency stop
```json
{"type":"estop"}
```

### 4) Poll (request latest state snapshot)
```json
{"type":"poll"}
```

> Notes:
> - Full-rate LiDAR scans are not intended to be streamed as JSON by default.
> - If you need LiDAR for Scratch/AI, prefer downsampled features (e.g., sector minima).

---

## Core-only Mode (No WebSocket)

If you want to run only the core (no WS server), use:

```bash
./build/atrbridge_core_daemon 192.168.0.78 9000 5601
```

This runs the core gateway and prints capability JSON periodically (useful for debugging robot connectivity and module detection).

---

## Don’t Run These Together

Only ONE TCP client should connect to the robot at a time:

✅ Use **`roslinker`** for ROS mode  
✅ Use **`atrbridge_ws_gateway`** for non-ROS Scratch/AI mode  
❌ Do NOT run both concurrently against the same robot

---

## Quick Debug Checklist

1) **Is the robot reachable?**
   - Can you ping `192.168.0.78`?

2) **Is the TCP connection free?**
   - Stop `roslinker`, `tb_bridge_cpp`, or any older bridge process.

3) **Does the gateway report connected?**
   - Look for `[core] tcp_connected=true`

4) **Did SET_ID succeed?**
   - Look for `[core] SET_ID sent: tb_01`
   - Confirm OLED shows `ID: tb_01`

If you still can’t connect, capture the gateway console output (including tcp_connected transitions) and the robot OLED status.
