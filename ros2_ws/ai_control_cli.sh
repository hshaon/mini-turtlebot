#!/usr/bin/env bash
set -eo pipefail

WS_ROOT="/home/shaon/mini-turtlebot/ros2_ws"
AILINKER_URL="${AILINKER_URL:-http://127.0.0.1:7070}"
GEMINI_MODEL="${GEMINI_MODEL:-gemini-2.5-flash-lite}"
MAX_STEPS="${MAX_STEPS:-1}"
LOOP_SLEEP_MS="${LOOP_SLEEP_MS:-900}"
LLM_MAX_RETRIES="${LLM_MAX_RETRIES:-2}"
LLM_BACKOFF_INITIAL_MS="${LLM_BACKOFF_INITIAL_MS:-1500}"
GEMINI_API_KEY="${GEMINI_API_KEY:-}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --ailinker-url URL
  --gemini-model MODEL
  --gemini-api-key KEY
  --max-steps N
  --loop-sleep-ms N
  --llm-max-retries N
  --llm-backoff-initial-ms N
  --task TEXT        Run one task and exit (non-interactive)
  -h, --help

Interactive commands:
  :q / :quit         Exit CLI
  :state             Show current AILinker state JSON
  :caps              Show capability JSON
  :stop              Send stop_robot
  :estop             Send estop
EOF
}

ONE_TASK=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ailinker-url) AILINKER_URL="$2"; shift 2 ;;
    --gemini-model) GEMINI_MODEL="$2"; shift 2 ;;
    --gemini-api-key) GEMINI_API_KEY="$2"; shift 2 ;;
    --max-steps) MAX_STEPS="$2"; shift 2 ;;
    --loop-sleep-ms) LOOP_SLEEP_MS="$2"; shift 2 ;;
    --llm-max-retries) LLM_MAX_RETRIES="$2"; shift 2 ;;
    --llm-backoff-initial-ms) LLM_BACKOFF_INITIAL_MS="$2"; shift 2 ;;
    --task) ONE_TASK="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 2 ;;
  esac
done

if [[ -z "$GEMINI_API_KEY" ]]; then
  echo "GEMINI_API_KEY is empty."
  echo "Set it first: export GEMINI_API_KEY=..."
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

health_check() {
  if ! curl -fsS "${AILINKER_URL}/context" >/dev/null 2>&1; then
    echo "AILinker is not reachable at ${AILINKER_URL}"
    echo "Start stack first: ./launch_ai_stack.sh --no-orchestrator ..."
    return 1
  fi
  return 0
}

run_task() {
  local task_text="$1"
  set +e
  GEMINI_API_KEY="$GEMINI_API_KEY" ros2 run ailinker ai_behavior_orchestrator \
    --task "$task_text" \
    --ailinker-url "$AILINKER_URL" \
    --gemini-model "$GEMINI_MODEL" \
    --max-steps "$MAX_STEPS" \
    --loop-sleep-ms "$LOOP_SLEEP_MS" \
    --llm-max-retries "$LLM_MAX_RETRIES" \
    --llm-backoff-initial-ms "$LLM_BACKOFF_INITIAL_MS"
  local rc=$?
  set -e
  if [[ $rc -ne 0 ]]; then
    echo "Task execution failed (exit=${rc}). You can retry or run :stop."
  fi
  return $rc
}

if [[ -n "$ONE_TASK" ]]; then
  health_check || exit 2
  run_task "$ONE_TASK"
  exit 0
fi

echo "MiniTB AI Control CLI"
echo "AILinker: ${AILINKER_URL}"
echo "Model: ${GEMINI_MODEL}"
echo "Type a task and press Enter."
echo "Commands: :state :caps :stop :estop :quit"

while true; do
  read -r -p "task> " line || break
  case "$line" in
    "") continue ;;
    :q|:quit) break ;;
    :state) curl -s "${AILINKER_URL}/state"; echo; continue ;;
    :caps) curl -s "${AILINKER_URL}/capabilities"; echo; continue ;;
    :stop) curl -s -X POST "${AILINKER_URL}/stop" -H 'Content-Type: application/json' -d '{}'; echo; continue ;;
    :estop) curl -s -X POST "${AILINKER_URL}/estop" -H 'Content-Type: application/json' -d '{}'; echo; continue ;;
    *) ;;
  esac

  if ! health_check; then
    continue
  fi
  run_task "$line" || true
done

echo "Exiting MiniTB AI Control CLI."
