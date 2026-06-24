import subprocess
import os
import time
import signal
import json
import re
from pathlib import Path

# ================= 配置区 =================
PACKAGE_NAME = "panda_pick"
LAUNCH_FILE = "demo.launch1.py"
ITERATIONS = 20           # 总实验次数
TIMEOUT = 60              # 每一把强制超时的秒数
LOG_DIR = "../eval_logs"  # 日志和JSON存放路径

def run_pipeline():
    # 1. 准备环境
    log_path = Path(LOG_DIR)
    log_path.mkdir(parents=True, exist_ok=True)
    
    all_trials_data = []
    summary = {
        "success_count": 0,
        "total_compute_time": 0.0,
        "total_manip_time": 0.0,
        "collision_count": 0,
        "stability_violations": 0
    }

    print(f"🚀 RCSML 自动化评测启动 | 目标次数: {ITERATIONS}")

    for i in range(1, ITERATIONS + 1):
        print(f"\n[Run {i}/{ITERATIONS}] 正在启动仿真...")
        current_log_file = log_path / f"trial_{i}.log"
        
        # 启动 ROS 2 Launch 进程组
        # os.setsid 用于创建一个新的进程会话，方便后面一键杀死所有子节点
        cmd = f"ros2 launch {PACKAGE_NAME} {LAUNCH_FILE}"
        process = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
            text=True, preexec_fn=os.setsid
        )

        trial_metrics = {}
        is_capturing = False
        start_wall_time = time.time()

        with open(current_log_file, "w") as f_log:
            try:
                # 2. 实时解析终端输出 (核心逻辑)
                while True:
                    line = process.stdout.readline()
                    if not line: break
                    
                    # 将所有输出同步写入日志文件，方便后期 Debug
                    f_log.write(line)
                    f_log.flush()

                    # 匹配标记：开始捕获
                    if "[METRICS_START]" in line:
                        is_capturing = True
                        continue
                    
                    # 匹配标记：结束捕获
                    if "[METRICS_END]" in line:
                        is_capturing = False
                        # 成功抓到数据，主动关掉当前进程，进入下一轮
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                        break
                    
                    # 提取数据行，例如 "success: 1" -> {"success": 1.0}
                    if is_capturing and ":" in line:
                        try:
                            key, val = line.split(":")
                            trial_metrics[key.strip()] = float(val.strip())
                        except ValueError:
                            continue

                # 等待进程彻底退出
                process.wait(timeout=5)

            except subprocess.TimeoutExpired:
                print(f"⚠️ 第 {i} 次实验超时，强制击杀...")
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                trial_metrics = {"success": 0.0, "error": "timeout"}

        # 3. 统计本轮数据
        if trial_metrics:
            all_trials_data.append(trial_metrics)
            if trial_metrics.get("success") == 1.0:
                summary["success_count"] += 1
            if trial_metrics.get("collision", 0) > 0:
                summary["collision_count"] += 1
            if trial_metrics.get("stability_violation", 0) > 0:
                summary["stability_violations"] += 1
            
            summary["total_compute_time"] += trial_metrics.get("compute_time", 0)
            summary["total_manip_time"] += trial_metrics.get("manip_time", 0)
            
            status = "✅ 成功" if trial_metrics.get("success") == 1.0 else "❌ 失败"
            print(f"结果: {status} | 精度误差: {trial_metrics.get('accuracy_error', 0)*1000:.2f} mm")
        else:
            print(f"❌ 第 {i} 次实验未捕获到任何 Metrics 数据")

    # 4. 持久化存储为 JSON
    final_report = {
        "metadata": {"total_iterations": ITERATIONS, "timestamp": time.ctime()},
        "summary": summary,
        "raw_data": all_trials_data
    }
    
    with open(log_path / "evaluation_summary.json", "w") as f_json:
        json.dump(final_report, f_json, indent=4)

    print("\n" + "="*40)
    print(f"🏁 评测全流程完成！最终成功率: {(summary['success_count']/ITERATIONS)*100:.1f}%")
    print(f"📂 报告位置: {log_path / 'evaluation_summary.json'}")

if __name__ == "__main__":
    run_pipeline()