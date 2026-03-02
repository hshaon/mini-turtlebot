#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CORE_BIN="${CORE_BIN:-${REPO_ROOT}/atrbridge_core/build/atrbridge_ws_gateway}"

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

RUN_DIR="${RUN_DIR:-/tmp/minitb_ai_pipeline}"
mkdir -p "${RUN_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
GW_LOG="${RUN_DIR}/gateway_${STAMP}.log"
AIL_LOG="${RUN_DIR}/ailinker_${STAMP}.log"
PID_FILE="${RUN_DIR}/stack.pid"
AILINKER_METRICS_CSV="${AILINKER_METRICS_CSV:-${RUN_DIR}/ailinker_metrics.csv}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --robot-ip IP
  --robot-id ID
  --tcp-port PORT
  --lidar-udp-port PORT
  --ws-port PORT
  --ailinker-host HOST
  --ailinker-port PORT
  --poll-hz HZ
  --caps-refresh-hz HZ
  --max-duration-ms N
  --deadman-timeout-ms N
  --foreground
  -h, --help
EOF
}

FOREGROUND=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip) ROBOT_IP="$2"; shift 2 ;;
    --robot-id) ROBOT_ID="$2"; shift 2 ;;
    --tcp-port) TCP_PORT="$2"; shift 2 ;;
    --lidar-udp-port) LIDAR_UDP_PORT="$2"; shift 2 ;;
    --ws-port) WS_PORT="$2"; shift 2 ;;
    --ailinker-host) AILINKER_HOST="$2"; shift 2 ;;
    --ailinker-port) AILINKER_PORT="$2"; shift 2 ;;
    --poll-hz) POLL_HZ="$2"; shift 2 ;;
    --caps-refresh-hz) CAPS_REFRESH_HZ="$2"; shift 2 ;;
    --max-duration-ms) MAX_DURATION_MS="$2"; shift 2 ;;
    --deadman-timeout-ms) DEADMAN_TIMEOUT_MS="$2"; shift 2 ;;
    --foreground) FOREGROUND=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 2 ;;
  esac
done

if [[ ! -x "${CORE_BIN}" ]]; then
  echo "Missing gateway binary: ${CORE_BIN}"
  echo "Build with:"
  echo "  cmake -S /home/shaon/mini-turtlebot/atrbridge_core -B /home/shaon/mini-turtlebot/atrbridge_core/build"
  echo "  cmake --build /home/shaon/mini-turtlebot/atrbridge_core/build -j"
  exit 2
fi

GATEWAY_WS_URL="ws://127.0.0.1:${WS_PORT}/ws"

if [[ "${FOREGROUND}" -eq 1 ]]; then
  echo "[1/2] Starting atrbridge_ws_gateway (foreground)"
  "${CORE_BIN}" "${ROBOT_IP}" "${TCP_PORT}" "${LIDAR_UDP_PORT}" "${WS_PORT}" "${ROBOT_ID}" &
  GW_PID=$!
  trap 'kill ${GW_PID} 2>/dev/null || true' EXIT INT TERM
  echo "[2/2] Starting ailinker_standalone (foreground)"
  python3 "${SCRIPT_DIR}/ailinker_standalone.py" \
    --gateway-ws-url "${GATEWAY_WS_URL}" \
    --bind-host "${AILINKER_HOST}" \
    --bind-port "${AILINKER_PORT}" \
    --poll-hz "${POLL_HZ}" \
    --capabilities-refresh-hz "${CAPS_REFRESH_HZ}" \
    --max-duration-ms "${MAX_DURATION_MS}" \
    --deadman-timeout-ms "${DEADMAN_TIMEOUT_MS}" \
    --metrics-csv-path "${AILINKER_METRICS_CSV}"
  exit 0
fi

echo "[1/2] Starting atrbridge_ws_gateway..."
"${CORE_BIN}" "${ROBOT_IP}" "${TCP_PORT}" "${LIDAR_UDP_PORT}" "${WS_PORT}" "${ROBOT_ID}" >"${GW_LOG}" 2>&1 &
GW_PID=$!
sleep 0.3

echo "[2/2] Starting ailinker_standalone..."
python3 "${SCRIPT_DIR}/ailinker_standalone.py" \
  --gateway-ws-url "${GATEWAY_WS_URL}" \
  --bind-host "${AILINKER_HOST}" \
  --bind-port "${AILINKER_PORT}" \
  --poll-hz "${POLL_HZ}" \
  --capabilities-refresh-hz "${CAPS_REFRESH_HZ}" \
  --max-duration-ms "${MAX_DURATION_MS}" \
  --deadman-timeout-ms "${DEADMAN_TIMEOUT_MS}" \
  --metrics-csv-path "${AILINKER_METRICS_CSV}" >"${AIL_LOG}" 2>&1 &
AIL_PID=$!

cat >"${PID_FILE}" <<EOF
GW_PID=${GW_PID}
AIL_PID=${AIL_PID}
GW_LOG=${GW_LOG}
AIL_LOG=${AIL_LOG}
AILINKER_METRICS_CSV=${AILINKER_METRICS_CSV}
EOF

echo "ROS-free stack started."
echo "  gateway pid:  ${GW_PID}"
echo "  ailinker pid: ${AIL_PID}"
echo "  gateway log:  ${GW_LOG}"
echo "  ailinker log: ${AIL_LOG}"
echo "  pid file:     ${PID_FILE}"
echo "Stop with: ${SCRIPT_DIR}/stop_rosfree_stack.sh"
