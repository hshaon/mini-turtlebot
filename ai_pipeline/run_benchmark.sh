#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_TASK="${SCRIPT_DIR}/run_task.sh"
ANALYZE="${SCRIPT_DIR}/analyze_metrics.py"

WAIT_SECONDS="${WAIT_SECONDS:-30}"
N_SCENARIO1="${N_SCENARIO1:-6}"
N_SCENARIO2="${N_SCENARIO2:-5}"
MAX_STEPS="${MAX_STEPS:-2}"
LLM_MAX_RETRIES="${LLM_MAX_RETRIES:-2}"
ARCHIVE_DIR="${ARCHIVE_DIR:-/tmp/minitb_ai_pipeline/archive}"
METRIC_DIR="${METRIC_DIR:-/tmp/minitb_ai_pipeline}"
AILINKER_URL="${AILINKER_URL:-http://127.0.0.1:7070}"

PROMPT_S1="move forward for 2 seconds, then stop"
PROMPT_S2="move forward for 1 second, then rotate 180 degree and move forward for 1 second"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --wait-seconds N      Wait time before each iteration (default: 30)
  --n1 N                Iterations for scenario 1 (default: 6)
  --n2 N                Iterations for scenario 2 (default: 5)
  --max-steps N         max-steps passed to run_task.sh (default: 2)
  --llm-max-retries N   retries passed to run_task.sh (default: 2)
  --archive-dir DIR     archive output directory
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wait-seconds) WAIT_SECONDS="$2"; shift 2 ;;
    --n1) N_SCENARIO1="$2"; shift 2 ;;
    --n2) N_SCENARIO2="$2"; shift 2 ;;
    --max-steps) MAX_STEPS="$2"; shift 2 ;;
    --llm-max-retries) LLM_MAX_RETRIES="$2"; shift 2 ;;
    --archive-dir) ARCHIVE_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 2 ;;
  esac
done

if [[ ! -x "${RUN_TASK}" ]]; then
  echo "Missing executable: ${RUN_TASK}"
  exit 2
fi

if [[ -z "${GEMINI_API_KEY:-}" ]]; then
  echo "GEMINI_API_KEY is not set."
  exit 2
fi

if ! curl -fsS "${AILINKER_URL}/context" >/dev/null 2>&1; then
  echo "AILinker is not reachable at ${AILINKER_URL}"
  echo "Start stack first: ./start_rosfree_stack.sh --robot-ip <ip> --robot-id <id>"
  exit 2
fi

mkdir -p "${ARCHIVE_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"

AIL_CSV="${METRIC_DIR}/ailinker_metrics.csv"
BEH_CSV="${METRIC_DIR}/behavior_metrics.csv"

run_scenario() {
  local scenario_id="$1"
  local prompt="$2"
  local runs="$3"

  echo "=== ${scenario_id}: ${prompt} (runs=${runs}) ==="
  rm -f "${AIL_CSV}" "${BEH_CSV}"

  for ((i=1; i<=runs; i++)); do
    local run_id
    run_id="$(printf '%s_run%02d' "${scenario_id}" "${i}")"
    echo "[${scenario_id}] waiting ${WAIT_SECONDS}s before ${run_id}..."
    sleep "${WAIT_SECONDS}"
    echo "[${scenario_id}] ${run_id} start"
    set +e
    "${RUN_TASK}" \
      --task-id "${run_id}" \
      --task "${prompt}" \
      --max-steps "${MAX_STEPS}" \
      --llm-max-retries "${LLM_MAX_RETRIES}"
    local rc=$?
    set -e
    if [[ ${rc} -ne 0 ]]; then
      echo "[${scenario_id}] ${run_id} failed (rc=${rc})"
      curl -s -X POST "${AILINKER_URL}/stop" -H 'Content-Type: application/json' -d '{}' >/dev/null || true
    fi
  done

  local ail_out beh_out rep_out
  ail_out="${ARCHIVE_DIR}/ailinker_${scenario_id}_${STAMP}.csv"
  beh_out="${ARCHIVE_DIR}/behavior_${scenario_id}_${STAMP}.csv"
  rep_out="${ARCHIVE_DIR}/report_${scenario_id}_${STAMP}.txt"

  cp "${AIL_CSV}" "${ail_out}"
  cp "${BEH_CSV}" "${beh_out}"
  python3 "${ANALYZE}" --ailinker-csv "${ail_out}" --behavior-csv "${beh_out}" | tee "${rep_out}"

  echo "[${scenario_id}] archived:"
  echo "  ${ail_out}"
  echo "  ${beh_out}"
  echo "  ${rep_out}"
}

run_scenario "s1_forward2s" "${PROMPT_S1}" "${N_SCENARIO1}"
run_scenario "s2_fwd1s_rot180_fwd1s" "${PROMPT_S2}" "${N_SCENARIO2}"

echo "Benchmark complete. Archive dir: ${ARCHIVE_DIR}"

