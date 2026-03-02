#!/usr/bin/env bash
set -eo pipefail

# MiniTB AI stack launcher:
# 1) atrbridge_ws_gateway
# 2) ailinker_server
# 3) ai_behavior_orchestrator (optional if GEMINI_API_KEY is set)

WS_ROOT="/home/shaon/mini-turtlebot/ros2_ws"
CORE_ROOT="/home/shaon/mini-turtlebot/atrbridge_core"

ROBOT_IP="${ROBOT_IP:-192.168.0.78}"
TCP_PORT="${TCP_PORT:-9000}"
LIDAR_UDP_PORT="${LIDAR_UDP_PORT:-5601}"
WS_PORT="${WS_PORT:-8080}"
ROBOT_ID="${ROBOT_ID:-tb_01}"

AILINKER_HOST="${AILINKER_HOST:-127.0.0.1}"
AILINKER_PORT="${AILINKER_PORT:-7070}"
POLL_HZ="${POLL_HZ:-20.0}"
CAPS_REFRESH_HZ="${CAPS_REFRESH_HZ:-1.0}"
MAX_DURATION_MS="${MAX_DURATION_MS:-2000}"
DEADMAN_TIMEOUT_MS="${DEADMAN_TIMEOUT_MS:-800}"

GEMINI_MODEL="${GEMINI_MODEL:-gemini-2.5-flash-lite}"
TASK="${TASK:-Drive forward slowly for a short distance, then stop safely.}"
MAX_STEPS="${MAX_STEPS:-1}"
LOOP_SLEEP_MS="${LOOP_SLEEP_MS:-900}"
LLM_MAX_RETRIES="${LLM_MAX_RETRIES:-2}"
LLM_BACKOFF_INITIAL_MS="${LLM_BACKOFF_INITIAL_MS:-1500}"
GEMINI_API_KEY="${GEMINI_API_KEY:-}"

LOG_DIR="${LOG_DIR:-/tmp/minitb_ai_stack}"
RUN_ORCHESTRATOR=1

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --robot-ip IP
  --tcp-port PORT
  --lidar-udp-port PORT
  --ws-port PORT
  --robot-id ID
  --ailinker-host HOST
  --ailinker-port PORT
  --task TEXT
  --gemini-model MODEL
  --gemini-api-key KEY
  --llm-max-retries N
  --llm-backoff-initial-ms N
  --no-orchestrator     Start gateway + ailinker only
  -h, --help

Environment overrides are also supported (ROBOT_IP, GEMINI_API_KEY, etc.).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip) ROBOT_IP="$2"; shift 2 ;;
    --tcp-port) TCP_PORT="$2"; shift 2 ;;
    --lidar-udp-port) LIDAR_UDP_PORT="$2"; shift 2 ;;
    --ws-port) WS_PORT="$2"; shift 2 ;;
    --robot-id) ROBOT_ID="$2"; shift 2 ;;
    --ailinker-host) AILINKER_HOST="$2"; shift 2 ;;
    --ailinker-port) AILINKER_PORT="$2"; shift 2 ;;
    --task) TASK="$2"; shift 2 ;;
    --gemini-model) GEMINI_MODEL="$2"; shift 2 ;;
    --gemini-api-key) GEMINI_API_KEY="$2"; shift 2 ;;
    --llm-max-retries) LLM_MAX_RETRIES="$2"; shift 2 ;;
    --llm-backoff-initial-ms) LLM_BACKOFF_INITIAL_MS="$2"; shift 2 ;;
    --no-orchestrator) RUN_ORCHESTRATOR=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 2 ;;
  esac
done

GATEWAY_WS_URL="ws://127.0.0.1:${WS_PORT}/ws"
AILINKER_URL="http://${AILINKER_HOST}:${AILINKER_PORT}"

mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
GW_LOG="${LOG_DIR}/gateway_${STAMP}.log"
AIL_LOG="${LOG_DIR}/ailinker_${STAMP}.log"
ORCH_LOG="${LOG_DIR}/orchestrator_${STAMP}.log"

# ROS setup scripts can reference unset vars internally; avoid nounset here.
set +u
source /opt/ros/jazzy/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

GW_BIN="${CORE_ROOT}/build/atrbridge_ws_gateway"
if [[ ! -x "$GW_BIN" ]]; then
  echo "Missing gateway binary: $GW_BIN"
  echo "Build it with: cmake -S ${CORE_ROOT} -B ${CORE_ROOT}/build && cmake --build ${CORE_ROOT}/build -j"
  exit 2
fi

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

echo "[1/3] Starting atrbridge_ws_gateway..."
"$GW_BIN" "$ROBOT_IP" "$TCP_PORT" "$LIDAR_UDP_PORT" "$WS_PORT" "$ROBOT_ID" >"$GW_LOG" 2>&1 &
pids+=("$!")
sleep 0.4

echo "[2/3] Starting ailinker_server..."
ros2 run ailinker ailinker_server \
  --gateway-ws-url "$GATEWAY_WS_URL" \
  --bind-host "$AILINKER_HOST" \
  --bind-port "$AILINKER_PORT" \
  --poll-hz "$POLL_HZ" \
  --capabilities-refresh-hz "$CAPS_REFRESH_HZ" \
  --max-duration-ms "$MAX_DURATION_MS" \
  --deadman-timeout-ms "$DEADMAN_TIMEOUT_MS" >"$AIL_LOG" 2>&1 &
pids+=("$!")
sleep 0.6

echo "Logs:"
echo "  gateway:      $GW_LOG"
echo "  ailinker:     $AIL_LOG"

if [[ "$RUN_ORCHESTRATOR" -eq 0 ]]; then
  echo "Orchestrator disabled (--no-orchestrator). Press Ctrl+C to stop."
  wait
  exit 0
fi

if [[ -z "${GEMINI_API_KEY:-}" ]]; then
  echo "GEMINI_API_KEY is not set. Running gateway + ailinker only."
  echo "Set GEMINI_API_KEY in environment (or --gemini-api-key) and rerun to launch orchestrator."
  wait
  exit 0
fi

echo "Gemini config: model=${GEMINI_MODEL}"

echo "[3/3] Starting ai_behavior_orchestrator..."
GEMINI_API_KEY="$GEMINI_API_KEY" ros2 run ailinker ai_behavior_orchestrator \
  --task "$TASK" \
  --ailinker-url "$AILINKER_URL" \
  --gemini-model "$GEMINI_MODEL" \
  --max-steps "$MAX_STEPS" \
  --loop-sleep-ms "$LOOP_SLEEP_MS" \
  --llm-max-retries "$LLM_MAX_RETRIES" \
  --llm-backoff-initial-ms "$LLM_BACKOFF_INITIAL_MS" >"$ORCH_LOG" 2>&1 &
pids+=("$!")
echo "  orchestrator: $ORCH_LOG"

echo "AI stack running. Waiting for orchestrator to finish (Ctrl+C to stop all)."
wait "${pids[-1]}"
