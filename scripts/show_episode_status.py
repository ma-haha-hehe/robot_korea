#!/usr/bin/env python3
"""Read an episode's stage/result without importing ROS or changing simulation."""
import argparse
import json
from pathlib import Path
import time

PHASES = {
    'empty_approach': '空载接近', 'grasp_approach': '抓取接近',
    'grasp_closure': '夹持稳定', 'lift': '抬升确认',
    'loaded_transfer': '带载搬运', 'alignment_and_insertion': '对齐与插入',
    'release': '松爪确认', 'retreat': '撤离', 'final_stability': '最终稳定性检查',
}


def read_snapshot(path):
    # Existing writers replace the JSON contents in place. A concurrent read
    # can briefly see an incomplete write; retry without touching the episode.
    for attempt in range(3):
        try:
            return json.loads(path.read_text())
        except FileNotFoundError:
            return None
        except json.JSONDecodeError:
            if attempt == 2:
                raise
            time.sleep(.02)


def find_episode_processes(run_dir):
    """Inspect direct CLI/trace processes on Linux; never trust an old PID file."""
    proc = Path('/proc')
    if not proc.is_dir():
        return None
    expected = run_dir.resolve()
    matches = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().decode().split('\0')
            if not argv or not Path(argv[0]).name.startswith('python'):
                continue
            if 'mj_bridge.benchmark_cli' in argv and 'run' in argv:
                flag = '--output-dir'
            elif any(Path(arg).name in ('trace_failure.py', 'trace_release.py') for arg in argv):
                flag = '--output'
            else:
                continue
            if flag not in argv or argv.index(flag) + 1 >= len(argv):
                continue
            output = Path(argv[argv.index(flag) + 1])
            if not output.is_absolute():
                output = (entry / 'cwd').resolve(strict=True) / output
            if output.resolve() == expected:
                matches.append(int(entry.name))
        except (OSError, UnicodeError):
            # Processes may exit between listing /proc and reading their args.
            continue
    return sorted(matches)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    if not args.run_dir.is_dir():
        parser.error('run directory does not exist')
    result = read_snapshot(args.run_dir / 'result.json')
    progress = read_snapshot(args.run_dir / 'progress.json') or {}
    trace = read_snapshot(args.run_dir / 'phase_trace.json') or {}
    aborted = read_snapshot(args.run_dir / 'ABORTED.json')
    if result:
        print('结果:', '成功' if result.get('success') is True else '失败')
        print(f"末态合格积木: {result.get('placed', '?')}/{result.get('total', '?')}")
        print('已释放积木:', len(result.get('events', progress.get('events', []))))
        print(f"仿真时间: {result.get('simulation_time_s', 0):.2f} s；"
              f"计算时间: {result.get('elapsed_s', 0):.2f} s")
        if result.get('execution_error'):
            print('原因:', result['execution_error'])
    elif aborted:
        print('结果: 已中止，未完成验收')
        print('原因:', aborted.get('reason', 'unknown'))
    else:
        print('结果: 尚无最终结果')
        processes = find_episode_processes(args.run_dir)
        if processes:
            print('当前匹配的执行进程 PID:', ', '.join(map(str, processes)))
        else:
            print('未发现直接执行进程；若使用其他启动器，请核对运行终端')
        print('已释放积木:', len(progress.get('events', [])))
    phases = trace.get('phases', [])
    if phases:
        current = phases[-1]
        print('最后记录阶段:', PHASES.get(current['phase'], current['phase']),
              current.get('block') or '', f"({current['status']})")
        if current.get('error'):
            print('阶段错误:', current['error'])
        try:
            age = max(0., time.time() - (args.run_dir / 'phase_trace.json').stat().st_mtime)
            print(f'阶段记录距今: {age:.1f} s（不是仿真时间）')
        except FileNotFoundError:
            pass


if __name__ == '__main__':
    main()
