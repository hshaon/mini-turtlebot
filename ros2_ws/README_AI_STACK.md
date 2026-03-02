## AI Stack Launch

This project provides two scripts:

- `launch_ai_stack.sh`: starts ATRBridge gateway + AILinker tool server (+ optional orchestrator)
- `ai_control_cli.sh`: interactive natural-language control CLI

> Scope note: this README is for the **ROS-based AI stack inside `ros2_ws`**.
> The **fully ROS-free pipeline** lives at:
> `/home/shaon/mini-turtlebot/ai_pipeline/README.md`

### Prerequisites

- ROS 2 Jazzy installed
- `atrbridge_ws_gateway` built in `atrbridge_core`
- `ailinker` package built in `ros2_ws`
- Gemini API key in environment

```bash
cd /home/shaon/mini-turtlebot/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
```

### Start AI Stack (recommended split mode)

Run gateway + AILinker only:

```bash
cd /home/shaon/mini-turtlebot/ros2_ws
./launch_ai_stack.sh --robot-ip 192.168.0.78 --robot-id tb_01 --no-orchestrator
```

### Start Interactive AI Control CLI

In a second terminal:

```bash
cd /home/shaon/mini-turtlebot/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
./ai_control_cli.sh
```

#### CLI built-in commands

- `:state` show current state cache  
- `:caps` show robot capabilities  
- `:stop` send stop command  
- `:estop` emergency stop  
- `:quit` exit CLI  

### Example Natural-Language Commands

Use these at `task>` prompt:

- `move forward for 2 seconds, then stop`
- `turn right in place for 1 second, then stop`
- `move forward for 0.5 second, turn right 90 deg, repeat this to make a square`
- `move backward for 1 second, then stop`
- `get me current capabilities`

### One-shot (non-interactive) command

```bash
./ai_control_cli.sh --task "move forward for 2 seconds, then stop"
```

### Useful Options

Run CLI with slightly higher completion chance for multi-step tasks:

```bash
./ai_control_cli.sh --max-steps 2
```

Use a different model:

```bash
./ai_control_cli.sh --gemini-model gemini-2.5-flash-lite
```

### Troubleshooting

#### 429 Too Many Requests

You are hitting Gemini API quota/rate limits.

- Wait for quota reset window
- Reduce calls per task (`--max-steps 1` or `2`)
- Keep prompts short and action-focused
- Verify API billing/tier for the active project

#### Safety stop

At any time:

```bash
curl -s -X POST http://127.0.0.1:7070/estop -H 'Content-Type: application/json' -d '{}'
```
