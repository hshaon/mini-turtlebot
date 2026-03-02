# ailinker

Layer 3 + Layer 4 components for MiniTB AI pipeline:

- `ailinker_server`: local safe tool server (HTTP) backed by `atrbridge_ws_gateway`.
- `ai_behavior_orchestrator`: Gemini client that uses AILinker tools in a function-call loop.

## Layer 3: AILinker Server

Default runtime:

- gateway WS: `ws://127.0.0.1:8080/ws`
- HTTP tool server: `http://127.0.0.1:7070`

HTTP endpoints:

- `GET /capabilities`
- `GET /state`
- `GET /context` (MCP-style context: capabilities + constraints + tools)
- `POST /cmd_vel` with JSON `{ "vx": 0.05, "wz": 0.0, "duration_ms": 400 }`
- `POST /stop`
- `POST /estop`

Safety:

- velocity clamp (`max_vx_mps`, `max_wz_rps`) from robot capabilities or CLI overrides
- command duration clamp (`--max-duration-ms`)
- deadman timeout (`--deadman-timeout-ms`)
- rejects command if robot link is not connected

## Layer 4: AI Behavior Orchestrator

The orchestrator:

1. Pulls `/context` from AILinker.
2. Sends tool schema + state to Gemini.
3. Executes returned function calls via local AILinker HTTP.
4. Feeds tool responses back to Gemini.
5. Repeats until done/max steps, then sends `stop_robot`.

## Build

From `ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/shaon/mini-turtlebot/ros2_ws
colcon build --packages-select ailinker
source install/setup.bash
```

## Run

1) Start ATRBridge WS gateway:

```bash
cd /home/shaon/mini-turtlebot/atrbridge_core
./build/atrbridge_ws_gateway 192.168.0.78 9000 5601 8080 tb_01
```

2) Start AILinker tool server:

```bash
source /home/shaon/mini-turtlebot/ros2_ws/install/setup.bash
ros2 run ailinker ailinker_server \
  --gateway-ws-url ws://127.0.0.1:8080/ws \
  --bind-host 127.0.0.1 --bind-port 7070
```

3) Optional: quick health check:

```bash
curl -s http://127.0.0.1:7070/context | jq .
curl -s http://127.0.0.1:7070/state | jq .
```

4) Run AI behavior orchestrator:

```bash
export GEMINI_API_KEY=YOUR_API_KEY
ros2 run ailinker ai_behavior_orchestrator \
  --task "Drive forward 0.3m slowly, then stop." \
  --ailinker-url http://127.0.0.1:7070 \
  --gemini-model gemini-2.5-flash-lite
```

## One-Command Launcher

From `ros2_ws`:

```bash
cd /home/shaon/mini-turtlebot/ros2_ws
export GEMINI_API_KEY=YOUR_API_KEY
./launch_ai_stack.sh --robot-ip 192.168.0.78 --robot-id tb_01
```

## Interactive AI CLI

Keep gateway + AILinker running, then use:

```bash
cd /home/shaon/mini-turtlebot/ros2_ws
./ai_control_cli.sh
```

Type natural-language tasks at `task>`.

Built-in commands:

- `:state`
- `:caps`
- `:stop`
- `:estop`
- `:quit`
