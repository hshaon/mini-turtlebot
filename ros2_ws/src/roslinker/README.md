# roslinker

ROS 2 linker package that connects ROS graph to `atrbridge_core`.

## Parameters

- `robot_id` (default: `tb_01`)
- `robot_ip` (default: `192.168.0.78`)
- `port` (default: `9000`)
- `lidar_udp_port` (default: `5601`)

## Interfaces

- Subscribes: `cmd_vel`
- Publishes: `imu/data_raw`, `ir/state`, `tof/range`, `lidar/scan`
- Services:
  - `bridge/set_telemetry` (`tb_interfaces/srv/SetTelemetry`)
  - `bridge/get_capabilities` (`tb_interfaces/srv/GetCapabilities`)
