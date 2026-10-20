#!/usr/bin/env bash
# Reproduce a simulation experiment without replacing the active source tree.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
AUDIT_DIR="${REPO_DIR}/runs/project-audit-20261013"
if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: bash $0 VARIANT product.yaml NEW_OUTPUT_DIR [seed]" >&2
  echo "Variants: latest-depth, latest-bulk, latest-bounded-lift, latest-rectangular, latest-supported, latest-supported-contact, latest-metered-square, latest-state-release, latest-layer-guard, contact-aware, tilt-unload, long-unload, airborne-v2, feedback-contact, fine-feedback, catalog-mass, fast-clear, guarded, stable, guided, material" >&2
  exit 2
fi
case "$1" in
  latest-depth|latest-bulk|latest-bounded-lift|latest-rectangular|latest-supported|latest-supported-contact|latest-metered-square|latest-state-release|latest-layer-guard)
    AUDIT_DIR="${REPO_DIR}/runs/project-audit-20261015"
    MANIFEST="${AUDIT_DIR}/${1}-candidate-manifest.json" ;;
  contact-aware) MANIFEST="${AUDIT_DIR}/contact-aware-candidate-manifest.json" ;;
  tilt-unload) MANIFEST="${AUDIT_DIR}/tilt-unload-candidate-manifest.json" ;;
  long-unload) MANIFEST="${AUDIT_DIR}/long-unload-candidate-manifest.json" ;;
  airborne-v2) MANIFEST="${AUDIT_DIR}/airborne-return-v2-candidate-manifest.json" ;;
  feedback-contact) MANIFEST="${AUDIT_DIR}/feedback-contact-candidate-manifest.json" ;;
  catalog-mass) MANIFEST="${AUDIT_DIR}/catalog-mass-candidate-manifest.json" ;;
  fine-feedback) MANIFEST="${AUDIT_DIR}/fine-feedback-candidate-manifest.json" ;;
  fast-clear) MANIFEST="${AUDIT_DIR}/fast-clear-candidate-manifest.json" ;;
  guarded) MANIFEST="${AUDIT_DIR}/guarded-candidate-manifest.json" ;;
  stable) MANIFEST="${AUDIT_DIR}/stable-candidate-manifest.json" ;;
  guided) MANIFEST="${AUDIT_DIR}/guided-candidate-manifest.json" ;;
  material) MANIFEST="${AUDIT_DIR}/candidate-manifest.json" ;;
  *) echo "Unknown frozen variant; see the audit report for acceptance status." >&2; exit 2 ;;
esac
if [[ -e "$3" ]]; then
  echo "Output already exists; choose a new directory to preserve evidence." >&2
  exit 2
fi
CANDIDATE_DIR="$("${REPO_DIR}/.venv/bin/python" - "$MANIFEST" <<'PY'
import hashlib, json, pathlib, sys
manifest = json.loads(pathlib.Path(sys.argv[1]).read_text())
candidate = pathlib.Path(manifest['path'])
for name, expected in manifest['runtime_sha256'].items():
    actual = hashlib.sha256((candidate / 'src/mj_bridge/mj_bridge' / name).read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit('Candidate changed since validation: ' + name)
config = candidate / 'examples/config/plastic_benchmark.yaml'
if hashlib.sha256(config.read_bytes()).hexdigest() != manifest['config_sha256']:
    raise SystemExit('Benchmark configuration changed since validation')
print(candidate)
PY
)"
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="${CANDIDATE_DIR}/src/mj_bridge:${PYTHONPATH:-}"
export MJ_BRIDGE_BENCHMARK_CONFIG="${CANDIDATE_DIR}/examples/config/plastic_benchmark.yaml"
export ROS_LOG_DIR="${AUDIT_DIR}/ros-log"
export ROS_DOMAIN_ID="${DUPLO_ROS_DOMAIN_ID:-228}"
export ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec "${REPO_DIR}/.venv/bin/python" -B -m mj_bridge.benchmark_cli run \
  --product "$2" --seed "${4:-42}" --output-dir "$3" --headless \
  --connection-mode physics --contact-profile plastic --observation oracle \
  --backend oracle --executor baseline --robot-base-x=-0.05 --speed-scale 1.5
