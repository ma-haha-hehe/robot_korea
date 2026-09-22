#!/usr/bin/env bash
# 必须 source 本文件，不能用 bash enter_sim_env.sh。
_lego_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误: 未找到 ROS 2 Humble (/opt/ros/humble)。" >&2
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -f "${_lego_repo}/.venv/bin/activate" ]]; then
  source "${_lego_repo}/.venv/bin/activate"
else
  echo "错误: 未找到 ${_lego_repo}/.venv" >&2
  return 1 2>/dev/null || exit 1
fi
# ros2 run 的入口脚本可能固定使用 /usr/bin/python3，因此显式暴露 venv 包。
export PYTHONPATH="${_lego_repo}/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export ROBOT_KOREA_ROOT="${_lego_repo}"
# The baseline solves tiny matrices; many BLAS threads add overhead and cause
# oversubscription when product episodes run concurrently. Allow explicit tuning.
export OPENBLAS_NUM_THREADS="${LEGO_BENCH_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${LEGO_BENCH_BLAS_THREADS:-1}"
export MJ_BRIDGE_BENCHMARK_CONFIG="${_lego_repo}/src/mj_bridge/mj_bridge/benchmark.yaml"
if [[ -f "${_lego_repo}/install/mj_bridge/share/mj_bridge/package.bash" ]]; then
  # 只加载本项目的公开基准包；完整工作区可能含有用户遗留的失效包。
  source "${_lego_repo}/install/mj_bridge/share/mj_bridge/package.bash"
  export PATH="${_lego_repo}/install/mj_bridge/lib/mj_bridge:${PATH}"
fi
cd "${_lego_repo}"
echo "已进入 Panda + MuJoCo 乐高抓取环境"
echo "Python: $(command -v python)"
echo "构建: colcon build --base-paths src/mj_bridge --packages-select mj_bridge --symlink-install"
echo "测试: python -m pytest -q src/mj_bridge/test/test_benchmark_core.py"
echo "验证产品: lego-bench validate examples/products/traffic_light.yaml"
echo "仅启动仿真: ./run_panda_lego_sim.sh --headless"
echo "完整 MoveIt pipeline: ros2 launch mj_bridge lego_bench.launch.py headless:=true"
unset _lego_repo
