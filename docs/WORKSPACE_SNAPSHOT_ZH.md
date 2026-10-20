# Ubuntu 操作工作区源码快照

[ubuntu-workspace-20261020](https://github.com/ma-haha-hehe/robot_korea/tree/ubuntu-workspace-20261020)
保留了 2026-10-20 Ubuntu 独立操作工作区的源码。它与公开基准 `main`
来自不同的开发线，不能只比较提交时间或直接互相覆盖。

- `main`：公开基准运行入口、当前任务导入器、接触模型和控制器。
- `ubuntu-workspace-20261020`：Ubuntu 完整操作源码，包括执行阶段、支撑检查、
  Panda/DUPLO 诊断脚本，以及原工作区中的视觉和机械臂规划修改。

快照基于原工作区提交 `d09cdda`，保存源文件与示例；构建产物、安装目录、日志、
Python 缓存和本地进度报告不属于该快照。原 Ubuntu 工作目录及其未提交修改保持原状。

快照源码在 Ubuntu 22.04、ROS 2 Humble、MuJoCo 3.11.0 环境中通过322项代码测试。
这不表示完整任务集通过物理装配或实机验收。对应测试记录见
[workspace_regression_tests.log](validation/workspace_20261020_regression_tests.log)。

下载这一开发线时可使用：

```bash
git clone --branch ubuntu-workspace-20261020 --single-branch \
  https://github.com/ma-haha-hehe/robot_korea.git robot_korea_ubuntu_snapshot
```

标签固定对应源码快照，后续开发应在新分支上进行。`main` 的回归结果不能替代
这一开发线的验证，代码测试也不能替代真实视觉与实机验收。
