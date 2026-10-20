#!/usr/bin/env bash
# Canonical headless DUPLO physics entry; executes source without a colcon rebuild.
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || $# -gt 3 ]]; then
  echo "Usage: bash $0 [product.yaml] [NEW_OUTPUT_DIR] [seed]"
  echo "Oracle / physics / plastic / speed 1.5; simulation only."
  echo "For parallel runs, assign distinct DUPLO_ROS_DOMAIN_ID values (0–232)."
  [[ $# -le 3 ]]
  exit
fi
PRODUCT="$(realpath "${1:-${REPO_DIR}/examples/products/workbenchmark_latest_20261014/two_brick_stack_001.yaml}")"
OUTPUT="$(realpath -m "${2:-${REPO_DIR}/runs/duplo-$(date +%Y%m%d-%H%M%S)-$$}")"
SEED="${3:-42}"
if [[ ! -f "${PRODUCT}" || ! "${SEED}" =~ ^[0-9]+$ ]]; then
  echo "Provide an existing product YAML and a nonnegative integer seed." >&2
  exit 2
fi
if [[ -e "${OUTPUT}" ]]; then
  echo "Output already exists; choose a new directory to preserve evidence: ${OUTPUT}" >&2
  exit 2
fi
SIM_DOMAIN="${DUPLO_ROS_DOMAIN_ID:-229}"
if [[ ! "${SIM_DOMAIN}" =~ ^[0-9]+$ ]] || (( 10#${SIM_DOMAIN} > 232 )); then
  echo "DUPLO_ROS_DOMAIN_ID must be an integer from 0 to 232." >&2
  exit 2
fi
set +u
source "${REPO_DIR}/enter_sim_env.sh" >/dev/null
set -u
export PYTHONPATH="${REPO_DIR}/src/mj_bridge:${PYTHONPATH:-}"
export MJ_BRIDGE_BENCHMARK_CONFIG="${REPO_DIR}/examples/config/plastic_benchmark.yaml"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="$((10#${SIM_DOMAIN}))"
export ROS_LOG_DIR="${OUTPUT}/ros-log"
export PYTHONDONTWRITEBYTECODE=1
if [[ "$(dirname "${PRODUCT}")" == "${REPO_DIR}/examples/products/workbenchmark_latest_20261014" ]]; then
  "${REPO_DIR}/.venv/bin/python" "${REPO_DIR}/scripts/import_workbenchmark_latest.py" \
    --output "${REPO_DIR}/examples/products/workbenchmark_latest_20261014" \
    --verify-task "$(basename "${PRODUCT}")"
fi
echo "DUPLO physics results: ${OUTPUT}"
exec "${REPO_DIR}/.venv/bin/python" -B -m mj_bridge.benchmark_cli run \
  --product "${PRODUCT}" --seed "${SEED}" --output-dir "${OUTPUT}" --headless \
  --connection-mode physics --contact-profile plastic --observation oracle \
  --backend oracle --executor baseline --robot-base-x=-0.05 --speed-scale 1.5
