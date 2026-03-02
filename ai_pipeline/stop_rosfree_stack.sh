#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${RUN_DIR:-/tmp/minitb_ai_pipeline}"
PID_FILE="${RUN_DIR}/stack.pid"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "No pid file found: ${PID_FILE}"
  exit 0
fi

# shellcheck disable=SC1090
source "${PID_FILE}"

for pid in "${AIL_PID:-}" "${GW_PID:-}"; do
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
  fi
done

# Also clean up any leftover processes from previous sessions.
pkill -f "/ai_pipeline/ailinker_standalone.py" 2>/dev/null || true
pkill -f "/atrbridge_core/build/atrbridge_ws_gateway" 2>/dev/null || true

echo "Stopped ROS-free stack (if running)."
rm -f "${PID_FILE}"
