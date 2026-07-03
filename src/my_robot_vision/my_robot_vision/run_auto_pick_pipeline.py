#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""【装配总控 · 宿主机侧】自动抓放流水线: 回观察位 -> 视觉识别 -> 执行抓放, 逐块循环。

在哪里被调用:
    run_carrot_onrobot.sh / run_estop_onrobot.sh / run_pick_onrobot.sh 等装配入口脚本
    都是 `python3 run_auto_pick_pipeline.py <参数>` 的封装。本文件是装配的"大脑",
    协调"宿主机 ROS(动机械臂) + docker 视觉容器(识别积木) + cpp 执行节点(抓放)"。

一轮(一块积木)的完整流程 = main() 里的循环, 每轮做:
    1) move_to_observe_pose()      机械臂 movej 回观察(拍照)位, 让相机看到积木
    2) ensure_vision_service()     确保 docker 里视觉桥进程活着(等待目标物名)
    3) trigger_and_wait_vision()   写目标物名 -> 视觉识别 -> 等 vision_output.yaml 出结果
    4) copy_and_convert_vision()   把视觉结果 cp 回宿主机, 调 vision_to_execution_yaml.py
                                    换算成 pick_place_task.yaml(示教位 + dx/dy/dz/yaw 偏移)
    5) execute_pick_place()        ros2 launch 起 cpp 节点(joint_pick_place)执行这块抓放
    (可选) run_async_next_vision() 边执行边预跑下一块视觉, 省时间

关键路径常量(文件顶部): 蓝图 plan、视觉输出、bridge 配置、pick_place_task.yaml、示教关节 env。

主要函数速查(起什么作用):
    run() / capture()          跑 shell 命令(实时输出 / 抓输出)
    ros_bash()                 在 source 好 ROS 的 bash 里跑命令
    docker_exec()              进 vision_node_final 容器执行命令
    write_plan()/generate_product_plan()  生成/写装配蓝图(要抓什么、放哪)
    ensure_vision_service()    起/保活容器里的视觉桥(注意: 严禁 docker cp bridge.py, 见内部注释)
    move_to_observe_pose()     机械臂回观察位
    trigger_vision()/wait_for_vision_output()/trigger_and_wait_vision()  触发并等视觉结果
    copy_vision_output()/convert_vision_output()/copy_and_convert_vision()  搬运+换算视觉结果
    execute_pick_place()       组 launch 命令并执行 cpp 抓放(安全限制/静止检测在这里)
    run_async_next_vision()    异步预跑下一块视觉(流水线加速)
    main()                     解析参数 + 逐块主循环
"""

import argparse
import importlib.util
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

import yaml


REPO = Path("/home/i6user/Desktop/robot_lego")
DOCKER_CONTAINER = "vision_node_final"
MIN_ROBOT_PIPELINE_WAIT_S = 50.0
VISION_BRIDGE_HOST = REPO / "src/my_robot_vision/my_robot_vision/vision_node_test1_5_bridge.py"
PLAN_HOST = REPO / "src/panda_pick/src/plan_perception_test.yaml"
PLAN_FROM_PRODUCT_HOST = REPO / "src/panda_pick/src/plan_from_product.yaml"
PLANNER_SCRIPT = REPO / "src/panda_pick/src/myplanner.py"
PLAN_DOCKER = "/vision_code/plan_perception_test.yaml"
TARGET_DOCKER = "/shared_data/current_target.txt"
VISION_OUTPUT_DOCKER = "/shared_data/vision_output.yaml"
VISION_PID_DOCKER = "/shared_data/vision_bridge.pid"
VISION_OUTPUT_HOST = REPO / "FoundationPose/root/FoundationPose/vision_output.yaml"
VISION_CACHE_DIR = REPO / "FoundationPose/root/FoundationPose/vision_cache"
BRIDGE_CONFIG = REPO / "src/my_robot_vision/config/vision_execution_bridge.yaml"
TASK_FILE = REPO / "src/panda_pick/config/pick_place_task.yaml"
TAUGHT_ENV = REPO / "src/panda_pick/config/ur5_taught_joints.env"
VISION_TO_EXEC = REPO / "src/my_robot_vision/my_robot_vision/vision_to_execution_yaml.py"


def run(cmd, *, check=True, cwd=REPO):
    print(f"\n[AUTO] $ {cmd}")
    result = subprocess.run(cmd, shell=True, cwd=str(cwd), text=True)
    if check and result.returncode != 0:
        raise RuntimeError(f"command failed with code {result.returncode}: {cmd}")
    return result.returncode


def capture(cmd, *, check=True, cwd=REPO):
    result = subprocess.run(
        cmd,
        shell=True,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if check and result.returncode != 0:
        print(result.stdout)
        raise RuntimeError(f"command failed with code {result.returncode}: {cmd}")
    return result.stdout


def print_timing(metric, seconds, **fields):
    details = [f"{metric}={float(seconds):.3f}"]
    for key, value in fields.items():
        details.append(f"{key}={shlex.quote(str(value))}")
    print("[AUTO][TIMING] " + " ".join(details))


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} must contain a YAML mapping")
    return data


def write_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def ros_bash(command):
    return (
        "bash -lc "
        + shlex.quote(
            "cd /home/i6user/Desktop/robot_lego && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            "source src/panda_pick/config/ur5_taught_joints.env && "
            + command
        )
    )


def docker_exec(docker_cmd, command, *, check=True):
    env_args = []
    display = os.environ.get("DISPLAY")
    if display:
        env_args.extend([
            "-e",
            f"DISPLAY={display}",
            "-e",
            "QT_X11_NO_MITSHM=1",
            "-e",
            "XAUTHORITY=",
        ])
    env_text = " ".join(shlex.quote(item) for item in env_args)
    if env_text:
        env_text += " "
    return run(f"{docker_cmd} exec {env_text}{DOCKER_CONTAINER} bash -lc {shlex.quote(command)}", check=check)


def write_plan(target, place_xyz, grasp_spin, blueprint_yaw):
    PLAN_HOST.parent.mkdir(parents=True, exist_ok=True)
    PLAN_HOST.write_text(
        f"""# Auto-generated by run_auto_pick_pipeline.py
tasksh:
  - id: 0
    name: {target}
    grasp_spin: {grasp_spin}
    blueprint_yaw: {blueprint_yaw}
    pick:
      pos: [0.0, 0.0, 0.0]
      orientation: [0.70710678, 0.70710678, 0.0, 0.0]
    place:
      pos: [{place_xyz[0]:.4f}, {place_xyz[1]:.4f}, {place_xyz[2]:.4f}]
      orientation: [0.70710678, 0.70710678, 0.0, 0.0]
""",
        encoding="utf-8",
    )
    print(f"[AUTO] wrote planner target: {PLAN_HOST}")


def task_target(task):
    for key in ("vision_label", "vision_name", "target", "name"):
        value = task.get(key)
        if value:
            return str(value).strip()
    raise RuntimeError(f"planner task has no target/name field: {task}")


def write_active_plan_task(task):
    write_yaml(PLAN_HOST, {"tasksh": [task]})
    print(f"[AUTO] wrote active planner task: {PLAN_HOST}")


def write_plan_tasks(tasks):
    tasks = list(tasks)
    write_yaml(PLAN_HOST, {"tasksh": tasks})
    print(f"[AUTO] wrote {len(tasks)} planner task(s): {PLAN_HOST}")


def load_plan_tasks(path):
    plan = load_yaml(path)
    tasks = plan.get("tasksh", plan.get("tasks", []))
    if not isinstance(tasks, list) or not tasks:
        raise RuntimeError(f"planner output has no tasksh/tasks list: {path}")
    return tasks


def generate_product_plan(product_yaml, output_yaml):
    if not PLANNER_SCRIPT.exists():
        raise RuntimeError(f"missing planner script: {PLANNER_SCRIPT}")
    start = time.perf_counter()
    spec = importlib.util.spec_from_file_location("panda_pick_myplanner", str(PLANNER_SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.process_blueprint(str(product_yaml), str(output_yaml))
    elapsed = time.perf_counter() - start
    print(f"[AUTO] generated product plan: {output_yaml}")
    print_timing("planner_time_s", elapsed, product_yaml=product_yaml.name)
    return elapsed


def safe_cache_name(text):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text).strip()).strip("_")
    cleaned = cleaned.strip("._-")
    return cleaned[:80] or "target"


def vision_cache_path(index, target):
    return VISION_CACHE_DIR / f"{index:02d}_{safe_cache_name(target)}.yaml"


def stop_vision_service(docker_cmd):
    # Keep this in a separate shell from the start command. If the same shell
    # also contains "python vision_node_test1_5_bridge.py", pkill -f can match
    # and terminate its own docker exec session with SIGTERM/code 143.
    stop_cmd = (
        f"if [ -s {shlex.quote(VISION_PID_DOCKER)} ]; then "
        f"pid=$(cat {shlex.quote(VISION_PID_DOCKER)}); "
        'kill "$pid" 2>/dev/null || true; '
        "fi; "
        "pkill -f '[v]ision_node_test1_5_bridge.py' || true; "
        f"rm -f {shlex.quote(VISION_PID_DOCKER)}"
    )
    docker_exec(docker_cmd, stop_cmd, check=False)


def ensure_vision_service(docker_cmd, target, *, frozen_observation=False, show_windows=False):
    run(f"{docker_cmd} start {DOCKER_CONTAINER}", check=False)
    # 2026-07-03: 不要 docker cp bridge.py! /vision_code 是宿主机 src/my_robot_vision/my_robot_vision
    #   的 bind mount(见 docker inspect Mounts), docker cp 源=目标会先截断目标再读源 ->
    #   把宿主机 bridge.py 清成 0 字节(反复踩坑的真凶)。bind mount 已实时同步宿主机改动, 无需 cp。
    run(f"{docker_cmd} cp {shlex.quote(str(PLAN_HOST))} {DOCKER_CONTAINER}:{PLAN_DOCKER}")
    stop_vision_service(docker_cmd)

    # Keep one bridge process alive inside Docker. It waits for TARGET_DOCKER.
    start_cmd = (
        f"rm -f {shlex.quote(TARGET_DOCKER)} {shlex.quote(VISION_OUTPUT_DOCKER)} "
        f"{shlex.quote(VISION_PID_DOCKER)}; "
        "cd /vision_code; "
        "nohup env "
        f"PLAN_YAML={shlex.quote(PLAN_DOCKER)} "
        f"TARGET_FILE={shlex.quote(TARGET_DOCKER)} "
        f"OUTPUT_FILE={shlex.quote(VISION_OUTPUT_DOCKER)} "
        f"VISION_FROZEN_OBSERVATION={'1' if frozen_observation else '0'} "
        f"VISION_SHOW_WINDOWS={'1' if show_windows else '0'} "
        "/opt/conda/envs/my/bin/python vision_node_test1_5_bridge.py "
        ">/shared_data/vision_bridge.log 2>&1 < /dev/null & "
        f"echo $! > {shlex.quote(VISION_PID_DOCKER)}; "
        "sleep 1; "
        f"pid=$(cat {shlex.quote(VISION_PID_DOCKER)}); "
        'kill -0 "$pid" 2>/dev/null || '
        "(tail -80 /shared_data/vision_bridge.log; exit 1)"
    )
    docker_exec(docker_cmd, start_cmd)
    mode = "frozen-observe async" if frozen_observation else "live"
    print(f"[AUTO] vision bridge starting for target: {target} ({mode})")


def move_to_observe_pose():
    print("[AUTO] moving robot to observe pose from UR5_OBSERVE_JOINTS")
    cmd = (
        "ros2 launch panda_pick run_ur5.launch.py "
        "motion_control_mode:=joint_ptp "
        "gripper_control_mode:=none "
        'urscript_pregrasp_joints:="$UR5_OBSERVE_JOINTS" '
        'urscript_home_joints:="$UR5_OBSERVE_JOINTS" '
        "joint_ptp_return_home:=false "
        "joint_ptp_dwell_seconds:=0.1 "
        "urscript_pipeline_wait_seconds:=8.0 "
        "urscript_movej_velocity:=2.50 "   # 2026-07-03 回观察位大幅提速(0.90->2.50, 与去装配区同速)
        "urscript_movej_acceleration:=5.0"
    )
    run(ros_bash(cmd))


def trigger_vision(docker_cmd, target):
    trigger = (
        f"rm -f {shlex.quote(VISION_OUTPUT_DOCKER)}; "
        f"printf %s {shlex.quote(target)} > {shlex.quote(TARGET_DOCKER)}"
    )
    docker_exec(docker_cmd, trigger)
    print(f"[AUTO] triggered vision target: {target}")


def wait_for_vision_output(docker_cmd, timeout, *, timing_metric="vision_wait_s", target=None):
    start = time.perf_counter()
    deadline = time.time() + timeout
    while time.time() < deadline:
        code = run(
            f"{docker_cmd} exec {DOCKER_CONTAINER} bash -lc "
            + shlex.quote(f"test -s {shlex.quote(VISION_OUTPUT_DOCKER)}"),
            check=False,
        )
        if code == 0:
            print("[AUTO] vision output is ready")
            elapsed = time.perf_counter() - start
            if timing_metric:
                fields = {}
                if target is not None:
                    fields["target"] = target
                print_timing(timing_metric, elapsed, **fields)
            return elapsed
        time.sleep(0.3)  # 2026-06-30: 2.0->0.3 少空等(视觉好了更快被发现)

    print(capture(f"{docker_cmd} exec {DOCKER_CONTAINER} bash -lc 'tail -80 /shared_data/vision_bridge.log'", check=False))
    raise TimeoutError(f"vision output not ready after {timeout} seconds")


def trigger_and_wait_vision(docker_cmd, target, timeout):
    start = time.perf_counter()
    trigger_vision(docker_cmd, target)
    wait_for_vision_output(docker_cmd, timeout, timing_metric=None, target=target)
    elapsed = time.perf_counter() - start
    print_timing("vision_wall_time_s", elapsed, target=target)
    return elapsed


def copy_vision_output(docker_cmd, host_path):
    host_path = Path(host_path)
    host_path.parent.mkdir(parents=True, exist_ok=True)
    run(f"{docker_cmd} cp {DOCKER_CONTAINER}:{VISION_OUTPUT_DOCKER} {shlex.quote(str(host_path))}")
    try:
        output = load_yaml(host_path)
        timing = output.get("timing", {})
        if "vision_compute_time_s" in timing:
            print_timing(
                "vision_compute_time_s",
                float(timing["vision_compute_time_s"]),
                target=output.get("target", ""),
                source=host_path.name,
            )
    except Exception as exc:
        print(f"[AUTO][WARN] could not read vision timing from {host_path}: {exc}")
    return host_path


def convert_vision_output(vision_path):
    vision_path = Path(vision_path)
    start = time.perf_counter()
    # vision_to_execution_yaml.py 是纯 YAML 转换(无 ROS 依赖)，直接用带 yaml/numpy 的
    # /usr/bin/python3 调用，避免 ros2 run + source ROS 的启动开销(每轮省 ~1-2s 空等)。
    run(
        f"/usr/bin/python3 {shlex.quote(str(VISION_TO_EXEC))} "
        f"--config {shlex.quote(str(BRIDGE_CONFIG))} "
        f"--plan {shlex.quote(str(PLAN_HOST))} "
        f"--vision {shlex.quote(str(vision_path))} "
        f"--output {shlex.quote(str(TASK_FILE))}"
    )
    elapsed = time.perf_counter() - start
    print_timing("conversion_time_s", elapsed, vision_yaml=vision_path.name)
    print(capture(f"cat {shlex.quote(str(TASK_FILE))}"))
    return elapsed


def copy_and_convert_vision(docker_cmd):
    copy_vision_output(docker_cmd, VISION_OUTPUT_HOST)
    return convert_vision_output(VISION_OUTPUT_HOST)


def execute_pick_place(
    gripper_mode,
    dry_run,
    gripper_speed,
    gripper_force,
    gripper_wait,
    gripper_pregrasp_position,
    gripper_release_position,
    gripper_release_wait,
    onrobot_rg_model,
    onrobot_open_width_mm,
    onrobot_close_width_mm,
    onrobot_pregrasp_width_mm,
    onrobot_release_width_mm,
    onrobot_force_n,
    movej_velocity,
    movej_acceleration,
    movel_velocity,
    movel_acceleration,
    pick_approach_movej_velocity,
    pick_approach_movel_velocity,
    pick_lift_velocity,
    pipeline_wait,
    done_still_seconds,
    place_descend_velocity,
    place_wiggle_enabled,
    place_wiggle_xy_amplitude,
    place_wiggle_z_amplitude,
    place_wiggle_velocity,
    place_wiggle_steps,
    return_home=True,
    keep_gripper_closed=False,
):
    if dry_run:
        print("[AUTO] dry-run enabled: not executing robot pick/place.")
        print_timing("robot_execution_time_s", 0.0, dry_run=True)
        return 0.0

    gripper_speed = max(0, min(255, int(gripper_speed)))
    gripper_force = max(0, min(255, int(gripper_force)))
    gripper_pregrasp_position = max(0, min(255, int(gripper_pregrasp_position)))
    gripper_release_position = max(0, min(255, int(gripper_release_position)))
    return_home_text = "true" if return_home else "false"
    # 静止检测窗口必须大于脚本中夹爪闭合/张开造成的机械臂停顿(≈gripper_wait+0.5)，
    # 否则会在抓取过程中误判"已完成"。这里强制安全下限，无论用户设多小。
    safe_still_floor = float(gripper_wait) + 1.5
    effective_done_still = max(float(done_still_seconds), safe_still_floor)
    if effective_done_still > float(done_still_seconds):
        print(
            f"[AUTO] done_still_seconds raised to {effective_done_still:.1f}s "
            f"(safety floor = gripper_wait {float(gripper_wait):.1f}s + 1.5s)"
        )
    effective_pipeline_wait = float(pipeline_wait)
    if not dry_run and effective_pipeline_wait < MIN_ROBOT_PIPELINE_WAIT_S:
        print(
            "[AUTO][WARN] pipeline_wait too short for strict robot serialization; "
            f"using {MIN_ROBOT_PIPELINE_WAIT_S:.1f}s instead of {effective_pipeline_wait:.1f}s"
        )
        effective_pipeline_wait = MIN_ROBOT_PIPELINE_WAIT_S
    cmd = (
        "ros2 launch panda_pick run_ur5.launch.py "
        f"gripper_control_mode:={shlex.quote(gripper_mode)} "
        f"gripper_socket_speed:={gripper_speed} "
        f"gripper_socket_force:={gripper_force} "
        f"gripper_socket_pregrasp_position:={gripper_pregrasp_position} "
        f"gripper_socket_open_wait_seconds:={float(gripper_wait):.3f} "
        f"gripper_socket_release_position:={gripper_release_position} "
        f"gripper_socket_release_wait_seconds:={float(gripper_release_wait):.3f} "
        f"gripper_urscript_wait_seconds:={float(gripper_wait):.3f} "
        f"keep_gripper_closed:={'true' if keep_gripper_closed else 'false'} "
        f"onrobot_rg_model:={shlex.quote(str(onrobot_rg_model))} "
        f"onrobot_rg_open_width_mm:={int(onrobot_open_width_mm)} "
        f"onrobot_rg_close_width_mm:={int(onrobot_close_width_mm)} "
        f"onrobot_rg_pregrasp_width_mm:={int(onrobot_pregrasp_width_mm)} "
        f"onrobot_rg_release_width_mm:={int(onrobot_release_width_mm)} "
        f"onrobot_rg_force_n:={int(onrobot_force_n)} "
        f"task_file:={shlex.quote(str(TASK_FILE))} "
        "task_max_xy_offset:=0.50 "  # 2026-07-03 0.25->0.35->0.50 (用户要求再增大); 与bridge safety.max_xy_offset一致
        "task_max_z_offset:=0.15 "   # 旧0.15
        "task_max_descend:=0.40 "    # 旧0.40->0.80->取消
        "io_gripper_wait_seconds:=1.5 "  # 2026-07-03 放置开爪等待加长(1.0->1.5), 保证抬升前一定张开
        'urscript_pregrasp_joints:="$UR5_PREGRASP_JOINTS" '
        'urscript_preplace_joints:="$UR5_PREPLACE_JOINTS" '
        'urscript_home_joints:="$UR5_HOME_JOINTS" '
        f"joint_ptp_return_home:={return_home_text} "
        f"urscript_movej_acceleration:={float(movej_acceleration):.3f} "
        f"urscript_movej_velocity:={float(movej_velocity):.3f} "
        f"urscript_pick_approach_movej_velocity:={float(pick_approach_movej_velocity):.3f} "
        f"urscript_movel_acceleration:={float(movel_acceleration):.3f} "
        f"urscript_movel_velocity:={float(movel_velocity):.3f} "
        f"urscript_pick_approach_movel_velocity:={float(pick_approach_movel_velocity):.3f} "
        f"urscript_pick_lift_velocity:={float(pick_lift_velocity):.3f} "
        "urscript_descend_acceleration:=0.35 "
        "urscript_descend_velocity:=0.40 "  # 2026-07-03 pick下降提速...->0.30->0.40(place前段另用place_fast_descend_velocity)
        "urscript_place_fast_descend_velocity:=0.60 "  # 2026-07-03 place前段提速...->0.50->0.60(超慢末段不变)
        f"urscript_place_descend_velocity:={float(place_descend_velocity):.6f} "
        f"urscript_place_wiggle_enabled:={'true' if place_wiggle_enabled else 'false'} "
        f"urscript_place_wiggle_xy_amplitude:={float(place_wiggle_xy_amplitude):.6f} "
        f"urscript_place_wiggle_z_amplitude:={float(place_wiggle_z_amplitude):.6f} "
        f"urscript_place_wiggle_velocity:={float(place_wiggle_velocity):.6f} "
        f"urscript_place_wiggle_steps:={int(place_wiggle_steps)} "
        "urscript_place_slow_final_descend:=0.020 "
        "urscript_place_settle_wait_seconds:=0.1 "  # 2026-07-03: 到位后更快开爪(原0.5)
        f"urscript_done_still_seconds:={effective_done_still:.3f} "
        f"urscript_pipeline_wait_seconds:={effective_pipeline_wait:.3f}"
    )
    start = time.perf_counter()
    output = capture(ros_bash(cmd))
    elapsed = time.perf_counter() - start
    print(output)
    failure_markers = (
        "process has died",
        "exit code 1",
        "超过安全限制",
        "[ERROR]",
    )
    if any(marker in output for marker in failure_markers):
        raise RuntimeError("robot execution launch reported an error; see log above")
    print_timing("robot_execution_time_s", elapsed, return_home=return_home_text)
    print("[AUTO] execution complete; URScript home pose is UR5_HOME_JOINTS")
    return elapsed


def run_async_next_vision(args, rounds):
    """Overlap current robot execution with next-target perception.

    The vision Docker freezes the first observe RGB-D frame, so the next target
    is computed from the same observation while the wrist camera is moving.
    """
    if any("task" not in item for item in rounds):
        raise RuntimeError("--async-next-vision currently requires --product-yaml planner tasks")

    selected_tasks = [item["task"] for item in rounds]
    if not args.skip_observe:
        move_to_observe_pose()

    write_plan_tasks(selected_tasks)
    ensure_vision_service(args.docker_cmd, rounds[0]["target"], frozen_observation=True, show_windows=args.show_windows)

    cache_paths = {}
    first_target = rounds[0]["target"]
    print(f"\n[AUTO] async bootstrap vision target: {first_target}")
    trigger_and_wait_vision(args.docker_cmd, first_target, args.vision_timeout)
    cache_paths[0] = copy_vision_output(args.docker_cmd, vision_cache_path(0, first_target))

    for index, item in enumerate(rounds):
        target = item["target"]
        print(f"\n[AUTO] ===== async round {index + 1}/{len(rounds)} target: {target} =====")

        write_active_plan_task(item["task"])
        convert_vision_output(cache_paths[index])

        next_item = rounds[index + 1] if index + 1 < len(rounds) else None
        if next_item is not None:
            print(
                "[AUTO] starting next vision in Docker before current robot execution: "
                f"{next_item['target']}"
            )
            next_vision_trigger_time = time.perf_counter()
            trigger_vision(args.docker_cmd, next_item["target"])
        else:
            next_vision_trigger_time = None

        execute_pick_place(
            args.gripper_mode,
            args.dry_run,
            args.gripper_speed,
            args.gripper_force,
            args.gripper_wait,
            args.gripper_pregrasp_position,
            args.gripper_release_position,
            args.gripper_release_wait,
            args.onrobot_rg_model,
            args.onrobot_open_width_mm,
            args.onrobot_close_width_mm,
            args.onrobot_pregrasp_width_mm,
            args.onrobot_release_width_mm,
            args.onrobot_force_n,
            args.movej_velocity,
            args.movej_acceleration,
            args.movel_velocity,
            args.movel_acceleration,
            args.pick_approach_movej_velocity,
            args.pick_approach_movel_velocity,
            args.pick_lift_velocity,
            args.pipeline_wait,
            args.done_still_seconds,
            args.place_descend_velocity,
            not args.disable_place_wiggle,
            args.place_wiggle_xy_amplitude,
            args.place_wiggle_z_amplitude,
            args.place_wiggle_velocity,
            args.place_wiggle_steps,
            False,
            keep_gripper_closed=args.keep_gripper_closed,
        )

        if next_item is not None:
            print(
                "[AUTO] waiting for next vision output after current execution "
                f"if it is not already ready: {next_item['target']}"
            )
            wait_after_execution = wait_for_vision_output(
                args.docker_cmd,
                args.vision_timeout,
                timing_metric="next_vision_wait_after_execution_s",
                target=next_item["target"],
            )
            if next_vision_trigger_time is not None:
                print_timing(
                    "next_vision_elapsed_since_trigger_s",
                    time.perf_counter() - next_vision_trigger_time,
                    target=next_item["target"],
                )
            print(
                "[AUTO][TIMING] "
                f"next_vision_ready_before_execution_done={str(wait_after_execution < 1.0).lower()} "
                f"target={shlex.quote(str(next_item['target']))}"
            )
            cache_paths[index + 1] = copy_vision_output(
                args.docker_cmd,
                vision_cache_path(index + 1, next_item["target"]),
            )


def parse_place(text):
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--place must be dx,dy,dz")
    return parts


def parse_places(text):
    if not text.strip():
        return []
    return [parse_place(item.strip()) for item in text.split(";") if item.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="yellow 2x4 brick.")
    parser.add_argument("--rounds", type=int, default=1,
                        help="repeat --target this many times when --targets is not provided")
    parser.add_argument("--targets", default="",
                        help="comma-separated targets for repeated rounds, e.g. 'yellow 2x4 brick.,red 2x2 brick.'")
    parser.add_argument("--place", type=parse_place, default=[0.0, 0.0, 0.0],
                        help="placement offset relative to assembly origin, meters: dx,dy,dz")
    parser.add_argument("--places", type=parse_places, default=[],
                        help="semicolon-separated placement offsets, one per target: '0,0,0;0.08,0,0'")
    parser.add_argument("--grasp-spin", type=float, default=0.0,
                        help="extra grasp yaw in degrees; use 90 or -90 for alternate grasp")
    parser.add_argument("--blueprint-yaw", type=float, default=0.0,
                        help="place yaw in radians, kept for compatibility with planner output")
    parser.add_argument("--docker-cmd", default="sudo docker")
    parser.add_argument("--vision-timeout", type=float, default=180.0)
    parser.add_argument("--skip-observe", action="store_true",
                        help="skip moving to the home/perception pose before vision")
    parser.add_argument("--async-next-vision", action="store_true",
                        help="compute the next target from the frozen observe frame while the robot executes the current task")
    parser.add_argument("--dry-run", action="store_true",
                        help="generate pick_place_task.yaml but do not execute robot")
    parser.add_argument("--show-windows", action="store_true",
                        help="pop up the vision bridge debug windows (Detection Logic / 6D Pose) via X11; needs DISPLAY + container X11 access")
    parser.add_argument("--gripper-mode", default="robotiq_socket",
                        choices=["none", "robotiq_socket", "urscript", "onrobot_rg", "onrobot_io"])
    parser.add_argument("--gripper-speed", type=int, default=180,
                        help="Robotiq socket SPE value, 0-255")
    parser.add_argument("--gripper-force", type=int, default=80,
                        help="Robotiq socket FOR value, 0-255")
    parser.add_argument("--gripper-wait", type=float, default=4.0,
                        help="seconds to wait after each Robotiq open/close command")
    parser.add_argument("--gripper-pregrasp-position", type=int, default=0,
                        help="Robotiq POS to pre-close to BEFORE descending onto the brick; "
                             "0=fully open (old behavior), e.g. 128 half-close then full-close after descent")
    parser.add_argument("--gripper-release-position", type=int, default=140,
                        help="partial Robotiq POS after placing, before lifting; 255 is closed, 0 is fully open")
    parser.add_argument("--gripper-release-wait", type=float, default=0.5,
                        help="seconds to wait after the partial release command before lifting")
    parser.add_argument("--onrobot-rg-model", default="rg2", choices=["rg2", "rg6"],
                        help="OnRobot RG model when --gripper-mode onrobot_rg")
    parser.add_argument("--onrobot-open-width-mm", type=int, default=90,
                        help="OnRobot RG open target width in mm")
    parser.add_argument("--onrobot-close-width-mm", type=int, default=12,
                        help="OnRobot RG close target width in mm; set below object width so force grip happens")
    parser.add_argument("--onrobot-pregrasp-width-mm", type=int, default=90,
                        help="OnRobot RG target width before descending onto the object")
    parser.add_argument("--onrobot-release-width-mm", type=int, default=45,
                        help="OnRobot RG partial release width before lifting away at place")
    parser.add_argument("--onrobot-force-n", type=int, default=20,
                        help="OnRobot RG grip force in N")
    parser.add_argument("--movej-velocity", type=float, default=0.60,
                        help="UR movej speed for large joint moves, including transfer from one placed block to the next task")
    parser.add_argument("--movej-acceleration", type=float, default=1.5,
                        help="UR movej acceleration for large joint moves")
    parser.add_argument("--movel-velocity", type=float, default=0.24,
                        help="UR movel speed for normal Cartesian moves; slow final place descent is controlled separately")
    parser.add_argument("--movel-acceleration", type=float, default=1.0,
                        help="UR movel acceleration for normal Cartesian moves")
    parser.add_argument("--pick-approach-movej-velocity", type=float, default=0.35,
                        help="UR movej speed from observation/current pose to the pick pregrasp joint pose")
    parser.add_argument("--pick-approach-movel-velocity", type=float, default=0.18,
                        help="UR movel speed from taught pregrasp to the vision-corrected pose above the object")
    parser.add_argument("--pick-lift-velocity", type=float, default=0.14,
                        help="UR movel speed for lifting the grasped brick up to pregrasp; raise it to carry faster")
    parser.add_argument("--pipeline-wait", type=float, default=50.0,
                        help="host-side hard-timeout cap after publishing each URScript task; "
                             "actual wait is now close-loop on robot_program_running, so this rarely matters")
    parser.add_argument("--done-still-seconds", type=float, default=3.5,
                        help="joint-stillness window to declare a task done (fallback when robot_program_running "
                             "doesn't toggle); auto-raised to gripper_wait+1.5s for safety. Lower gripper_wait to lower this")
    parser.add_argument("--place-descend-velocity", type=float, default=0.002,
                        help="m/s downward speed for the final placement insertion; 0.002 means 2 mm/s")
    parser.add_argument("--keep-gripper-closed", action="store_true",
                        help="放置点不松开夹爪, 一直夹紧(手臂照常抬起离开)")
    parser.add_argument("--disable-place-wiggle", action="store_true",
                        help="disable micro wiggle during the final slow placement descent")
    parser.add_argument("--place-wiggle-xy-amplitude", type=float, default=0.0006,
                        help="meters of lateral micro wiggle during final placement descent")
    parser.add_argument("--place-wiggle-z-amplitude", type=float, default=0.0,
                        help="kept for compatibility; separated wiggle mode uses lateral motion only")
    parser.add_argument("--place-wiggle-velocity", type=float, default=0.012,
                        help="m/s velocity for lateral wiggle moves; downward insertion uses slow descent velocity")
    parser.add_argument("--place-wiggle-steps", type=int, default=12,
                        help="number of slow downward layers in the final placement wiggle descent")
    parser.add_argument("--product-yaml", default="",
                        help="run myplanner.py on a final_product YAML and execute its generated sequence")
    parser.add_argument("--planner-output", default=str(PLAN_FROM_PRODUCT_HOST),
                        help="where to write the full plan generated from --product-yaml")
    parser.add_argument("--start-index", type=int, default=0,
                        help="first planner task index to execute when --product-yaml is used")
    parser.add_argument("--max-tasks", type=int, default=0,
                        help="limit number of planner tasks when --product-yaml is used; 0 means all")
    args = parser.parse_args()

    if not TAUGHT_ENV.exists():
        raise RuntimeError(f"missing taught joints file: {TAUGHT_ENV}")
    if args.rounds < 1:
        raise RuntimeError("--rounds must be at least 1")

    if args.product_yaml:
        product_yaml = Path(args.product_yaml).expanduser()
        if not product_yaml.is_absolute():
            product_yaml = REPO / product_yaml
        planner_output = Path(args.planner_output).expanduser()
        if not planner_output.is_absolute():
            planner_output = REPO / planner_output
        generate_product_plan(product_yaml, planner_output)
        planner_tasks = load_plan_tasks(planner_output)
        if args.start_index < 0 or args.start_index >= len(planner_tasks):
            raise RuntimeError(f"--start-index {args.start_index} is outside 0..{len(planner_tasks) - 1}")
        end_index = len(planner_tasks) if args.max_tasks <= 0 else min(len(planner_tasks), args.start_index + args.max_tasks)
        rounds = [{"target": task_target(task), "task": task} for task in planner_tasks[args.start_index:end_index]]
    else:
        targets = [item.strip() for item in args.targets.split(",") if item.strip()]
        if not targets:
            targets = [args.target] * args.rounds
        if args.places and len(args.places) < len(targets):
            raise RuntimeError(
                f"--places has {len(args.places)} item(s), but --targets/rounds needs {len(targets)}. "
                "Provide one place offset per target."
            )
        rounds = []
        for index, target in enumerate(targets):
            place = args.places[index] if args.places else args.place
            rounds.append({
                "target": target,
                "place": place,
                "grasp_spin": args.grasp_spin,
                "blueprint_yaw": args.blueprint_yaw,
            })

    if args.async_next_vision:
        run_async_next_vision(args, rounds)
        print("[AUTO] pipeline finished")
        return

    # ===== 逐块装配主循环: 每块 = 写蓝图 -> 回观察位 -> 视觉 -> 换算 -> 抓放 =====
    for index, item in enumerate(rounds, start=1):
        target = item["target"]
        print(f"\n[AUTO] ===== round {index}/{len(rounds)} target: {target} =====")
        if "task" in item:
            write_active_plan_task(item["task"])   # 用规划器生成的整任务
        else:
            write_plan(target, item["place"], item["grasp_spin"], item["blueprint_yaw"])  # 单目标临时蓝图
        if not args.skip_observe:
            move_to_observe_pose()                 # 机械臂回观察位拍照
        ensure_vision_service(args.docker_cmd, target, show_windows=args.show_windows)  # 保活视觉桥
        trigger_and_wait_vision(args.docker_cmd, target, args.vision_timeout)           # 识别并等结果
        copy_and_convert_vision(args.docker_cmd)   # 搬回宿主机 + 换算成 pick_place_task.yaml
        execute_pick_place(
            args.gripper_mode,
            args.dry_run,
            args.gripper_speed,
            args.gripper_force,
            args.gripper_wait,
            args.gripper_pregrasp_position,
            args.gripper_release_position,
            args.gripper_release_wait,
            args.onrobot_rg_model,
            args.onrobot_open_width_mm,
            args.onrobot_close_width_mm,
            args.onrobot_pregrasp_width_mm,
            args.onrobot_release_width_mm,
            args.onrobot_force_n,
            args.movej_velocity,
            args.movej_acceleration,
            args.movel_velocity,
            args.movel_acceleration,
            args.pick_approach_movej_velocity,
            args.pick_approach_movel_velocity,
            args.pick_lift_velocity,
            args.pipeline_wait,
            args.done_still_seconds,
            args.place_descend_velocity,
            not args.disable_place_wiggle,
            args.place_wiggle_xy_amplitude,
            args.place_wiggle_z_amplitude,
            args.place_wiggle_velocity,
            args.place_wiggle_steps,
            True,
            keep_gripper_closed=args.keep_gripper_closed,
        )
        # The next round always starts by moving/confirming UR5_OBSERVE_JOINTS
        # before perception. The URScript execution also returns there after
        # each completed pick/place task.
    print("[AUTO] pipeline finished")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[AUTO][ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
