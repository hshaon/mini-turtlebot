# atrbridge_core

Standalone CMake-only ATRBridge core library with no ROS dependency.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Components

- `libatrbridge_core`: TCP/UDP transport + binary frame decode + state/cache + capabilities API
- `atrbridge_core_daemon`: optional executable that runs core and prints capabilities JSON
- `atrbridge_ws_gateway`: optional WS/JSON gateway for non-ROS clients

## Daemon usage

```bash
./build/atrbridge_core_daemon <robot_ip> <tcp_port> <lidar_udp_port>
```

## WS/JSON gateway usage

```bash
./build/atrbridge_ws_gateway <robot_ip> <tcp_port> <lidar_udp_port> <ws_port>
```

Messages:

- `{"type":"get_capabilities"}`
- `{"type":"cmd_vel","vx":0.1,"wz":0.2}`
- `{"type":"estop"}`
- `{"type":"poll"}`
