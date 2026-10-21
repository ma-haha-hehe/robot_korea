# 开发工作区与运行环境

仿真依赖 Ubuntu 22.04、ROS 2 Humble 和本仓库的 Python 环境。
可以在 Mac 上使用编辑器，通过 SSH 打开 Ubuntu 中的项目目录；代码、依赖、
终端和仿真进程都在 Ubuntu 上运行。运行期间 Ubuntu 需要开机、联网且不休眠。

## 仓库与本地产物

每个 Git 克隆都有自己的工作目录和未提交修改。远端 GitHub 只在 `push` 后更新，
另一台电脑也只在 `fetch`/`pull` 后获得这些提交。SSH 编辑直接修改远端工作目录，
不需要额外复制一份代码到 Mac。

建议让仿真代码、完整任务集与实验输出分别管理：

```text
workspace/
├── robot_korea/       # 本仓库：代码、示例、配置与指南
├── benchmark_lego/    # 完整任务集、版本与预览图
└── experiment_data/   # 可选的外部实验归档
```

本仓库的 `runs/`、`build/`、`install/`、`log/`、`.venv/` 和缓存均为本地产物。
源代码和可复现配置可以同步，实验结果可按需要另行归档。不要将构建目录复制到另一操作系统。

## 初始化与日常开发

在 Ubuntu 克隆后按 [README](../README.md#installation) 安装依赖并构建。
每个新终端执行：

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
```

修改 Python 源码时，`--symlink-install` 通常允许直接读取修改；新增模块、入口、
资源或依赖后重新执行：

```bash
colcon build --base-paths src/mj_bridge --packages-select mj_bridge --symlink-install
source enter_sim_env.sh
```

同步前先运行 `git status`，确认需要保留的本地修改。不要用重置或覆盖工作目录的方式
代替正常提交合并。多人或多台电脑同时开发时使用独立分支，并保持实验所对应的提交或源码指纹。

## 环境边界

本仓库的公开入口面向仿真，机械臂型号相同也不能直接替代实机驱动。
实机项目需要独立核对关节映射、坐标标定、控制周期、驱动力/速度限制和传感输入。
远程编辑的网络延迟与仿真/实机内部控制周期不同；控制进程应在机器人所在的 Ubuntu 上运行。
