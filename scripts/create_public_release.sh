#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-${REPO}/dist/lego-bench-public}"
DEST="$(realpath -m "${DEST}")"
RELEASE_ROOT="$(realpath -m "${REPO}/dist")"
if [[ "${DEST}" != "${RELEASE_ROOT}"/* ]]; then
  echo "Refusing unsafe release destination: ${DEST}" >&2
  echo "Release destinations must be below ${RELEASE_ROOT}" >&2
  exit 2
fi
mkdir -p "${DEST}"
if [[ -e "${DEST}/.git" ]]; then
  echo "Refusing to overwrite a Git checkout: ${DEST}. Choose a new candidate directory." >&2
  exit 2
fi
touch "${RELEASE_ROOT}/COLCON_IGNORE"
rsync -a --delete --delete-excluded \
  --exclude '.git/' --exclude '.venv/' --exclude 'build/' --exclude 'install/' \
  --exclude 'log/' --exclude 'runs/' --exclude 'FoundationPose/' \
  --exclude '__pycache__/' --exclude '.pytest_cache/' --exclude '*.pyc' \
  --exclude 'weights/' --exclude '*.pt' --exclude '*.pth' --exclude '*.onnx' \
  --exclude 'LEGO_Duplo_brick_2x2.stl' --exclude 'LEGO_Duplo_brick_2x4.stl' \
  --exclude 'O3P_API-*/' --exclude 'calibration_images/' --exclude 'real_robot_eval/' \
  --exclude '/docs/WORK_PROGRESS.md' \
  --include '/src/' --include '/src/mj_bridge/' \
  --include '/src/mj_bridge/package.xml' --include '/src/mj_bridge/setup.py' \
  --include '/src/mj_bridge/setup.cfg' --include '/src/mj_bridge/BENCHMARK.md' \
  --include '/src/mj_bridge/resource/' --include '/src/mj_bridge/resource/mj_bridge' \
  --include '/src/mj_bridge/launch/' --include '/src/mj_bridge/launch/lego_bench.launch.py' \
  --include '/src/mj_bridge/test/' --include '/src/mj_bridge/test/test_benchmark_core.py' --include '/src/mj_bridge/test/test_public_regressions.py' --include '/src/mj_bridge/test/test_grasp_planning.py' \
  --include '/src/mj_bridge/mj_bridge/' \
  --include '/src/mj_bridge/mj_bridge/assets/' --include '/src/mj_bridge/mj_bridge/assets/***' \
  --include '/src/mj_bridge/mj_bridge/__init__.py' \
  --include '/src/mj_bridge/mj_bridge/LICENSE' \
  --include '/src/mj_bridge/mj_bridge/benchmark.yaml' \
  --include '/src/mj_bridge/mj_bridge/benchmark_cli.py' \
  --include '/src/mj_bridge/mj_bridge/assembly_planner.py' \
  --include '/src/mj_bridge/mj_bridge/reference_executor.py' \
  --include '/src/mj_bridge/mj_bridge/perception.py' \
  --include '/src/mj_bridge/mj_bridge/product_geometry.py' \
  --include '/src/mj_bridge/mj_bridge/benchmark_core.py' \
  --include '/src/mj_bridge/mj_bridge/hand.xml' \
  --include '/src/mj_bridge/mj_bridge/initial.yaml' \
  --include '/src/mj_bridge/mj_bridge/initial_positions.yaml' \
  --include '/src/mj_bridge/mj_bridge/mj_bridge3.py' \
  --include '/src/mj_bridge/mj_bridge/panda.xml' \
  --include '/src/mj_bridge/mj_bridge/part_registry.yaml' \
  --include '/src/mj_bridge/mj_bridge/product_schema.json' \
  --include '/src/mj_bridge/mj_bridge/scene_builder.py' \
  --include '/src/mj_bridge/mj_bridge/scene_template.xml' \
  --include '/examples/***' --include '/docs/***' --include '/.github/***' \
  --include '/scripts/' --include '/scripts/plan_assembly.py' --include '/scripts/test_product_suite.py' --include '/scripts/audit_product_geometry.py' --include '/scripts/create_public_release.sh' --include '/scripts/write_validation_report.py' \
  --include '/.gitignore' --include '/pytest.ini' --include '/conftest.py' --include '/README.md' --include '/THIRD_PARTY_NOTICES.md' \
  --include '/LICENSE' --include '/.dockerignore' \
  --include '/requirements-sim.txt' --include '/Dockerfile.benchmark' \
  --include '/docker-compose.benchmark.yaml' --include '/enter_sim_env.sh' \
  --include '/install_sim_system_deps.sh' --include '/run_panda_lego_sim.sh' \
  --exclude '*' "${REPO}/" "${DEST}/"
echo "Public release candidate written to ${DEST}"
echo "Review THIRD_PARTY_NOTICES.md and mesh licenses before publishing."
