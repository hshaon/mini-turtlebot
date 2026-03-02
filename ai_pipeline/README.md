# ROS-Free AI Pipeline

This folder provides a complete **ROS-free** control pipeline:

```
TurtleBot <-> atrbridge_ws_gateway <-> ailinker_standalone <-> ai_behavior_gen
```

**Components**
- **`ailinker_standalone.py` (Layer 3)**: tool server + safety + execution  
- **`ai_behavior_gen.py` (Layer 4)**: Gemini orchestrator  
- **`ai_control_cli.py`**: interactive natural-language CLI  

> Note: The existing ROS package `src/ailinker` is **untouched** and can still be used later.

---

## 1) Prerequisites

### 1.1 Build ATRBridge core gateway binary
```bash
cd /home/shaon/mini-turtlebot/atrbridge_core
cmake -S . -B build
cmake --build build -j
```

### 1.2 Install Python deps (no ROS needed)
```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip setuptools
pip install -r requirements.txt
```

### 1.3 Set Gemini API key
```bash
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
```

---

## 2) Start ROS-free Stack

```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
./start_rosfree_stack.sh --robot-ip 192.168.0.78 --robot-id tb_01
```

This starts:
- `atrbridge_ws_gateway` on **`ws://127.0.0.1:8080/ws`**
- `ailinker_standalone` on **`http://127.0.0.1:7070`**

Stop stack:
```bash
./stop_rosfree_stack.sh
```

---

## 3) Interactive AI Control

```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
source .venv/bin/activate
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
./ai_control_cli.sh
```

### CLI commands
- `:state`  
- `:caps`  
- `:metrics`  
- `:stop`  
- `:estop`  
- `:quit`  

---

## 4) One-shot Task

```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
source .venv/bin/activate
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
./run_task.sh --task "move forward for 2 seconds, then stop"
```

---

## 5) Example Natural-Language Tasks

- `move forward for 2 seconds, then stop`
- `turn right in place for 1 second, then stop`
- `move forward for 0.5 second, turn right 90 deg, repeat this to make a square`
- `get current capabilities`

---

## 6) Robust Metrics

Two CSV metric files are generated (append-only by default):

- **AILinker metrics** (transport + safety + execution)  
  - default: `/tmp/minitb_ai_pipeline/ailinker_metrics.csv`

- **Behavior generator metrics** (LLM + tool orchestration)  
  - default: `/tmp/minitb_ai_pipeline/behavior_metrics.csv`

> Tip: Clear them before a new experiment batch.

### Captured metrics

**`ailinker_metrics.csv`** includes:
- gateway request RTT (`gw_req.rtt_us`) by request type  
- command execution latency (`cmd_exec.latency_us`)  
- keepalive count and duration compliance  
- telemetry receive events for achieved poll rate  
- stop/estop success/failure  

**`behavior_metrics.csv`** includes:
- LLM call latency and HTTP status (429/5xx counts)  
- per-tool call latency and success  
- task lifecycle events (`task_start`, `task_end`, `task_interrupt`)  

### Analyze metrics

```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
source .venv/bin/activate
python3 analyze_metrics.py \
  --ailinker-csv /tmp/minitb_ai_pipeline/ailinker_metrics.csv \
  --behavior-csv /tmp/minitb_ai_pipeline/behavior_metrics.csv
```

The analyzer prints **mean, p50, p95, p99, min, max** (paper-friendly summary).

---

## Practical Measurement Workflow (Step-by-step)

1) Clear old metric logs:
```bash
rm -f /tmp/minitb_ai_pipeline/ailinker_metrics.csv /tmp/minitb_ai_pipeline/behavior_metrics.csv
```

2) Start stack:
```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
source .venv/bin/activate
./start_rosfree_stack.sh --robot-ip 192.168.0.78 --robot-id tb_01
```

3) Run one controlled scenario (example):
```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
source .venv/bin/activate
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
./run_task.sh \
  --task-id s1_forward_2s \
  --task "move forward for 2 seconds, then stop" \
  --max-steps 2 \
  --llm-max-retries 2
```

4) Repeat **N runs per scenario** (recommended **N ≥ 5**), changing `--task-id` each run.

5) Snapshot/copy CSV logs after each scenario batch:
```bash
mkdir -p /tmp/minitb_ai_pipeline/archive
cp /tmp/minitb_ai_pipeline/ailinker_metrics.csv /tmp/minitb_ai_pipeline/archive/ailinker_s1.csv
cp /tmp/minitb_ai_pipeline/behavior_metrics.csv /tmp/minitb_ai_pipeline/archive/behavior_s1.csv
```

6) Analyze:
```bash
python3 analyze_metrics.py \
  --ailinker-csv /tmp/minitb_ai_pipeline/archive/ailinker_s1.csv \
  --behavior-csv /tmp/minitb_ai_pipeline/archive/behavior_s1.csv
```

7) Stop stack when done:
```bash
cd /home/shaon/mini-turtlebot/ai_pipeline
./stop_rosfree_stack.sh
```

---

## What to report from these metrics

### Control path
- `gw_req.rtt_us[cmd_vel]`
- `cmd_exec.latency_us`
- `cmd_exec.ok_rate`

### AI/tooling overhead
- `llm_call.latency_us`
- `llm_call.http_status_counts` (especially 429)
- `tool_call.latency_us[send_cmd_vel]`

### Stream/poll stability
- `telemetry_rx.achieved_hz`

---

## 7) Recommended Evaluation Procedure

1) Clear old logs:
```bash
rm -f /tmp/minitb_ai_pipeline/ailinker_metrics.csv /tmp/minitb_ai_pipeline/behavior_metrics.csv
```

2) Run each scenario **N times** (**N ≥ 5**) at fixed duration.

3) Keep Wi-Fi/AP conditions fixed per batch.

4) Use `analyze_metrics.py` for summary stats.

5) Report **p50/p95/p99** for:
- `gw_req.rtt_us[cmd_vel]`
- `cmd_exec.latency_us`
- `llm_call.latency_us`
- `tool_call.latency_us[send_cmd_vel]`

6) Include failure rates:
- `cmd_exec.ok_rate`
- HTTP 429 counts in `llm_call.http_status_counts`
