# Robotiq 2F on UR5 CB3.0 安装记录

## 结论

你的机器人序列号是 `2015351585`，在 Robotiq 给出的 UR5 CB3.0 范围 `2014350001` 到 `2016351863` 内，所以按 CB3.0 处理。

不要再安装 `Robotiq_Grippers-1.2.1.urcap`。你上次装它以后示教器点击无响应，说明这个 URCap 在你的 CB3.0/PolyScope 3.15 状态下不稳定。

这次使用 legacy driver package:

```text
DCU-1.0.10_20200630_20200630.zip
```

我已经准备好 USB 根目录文件夹:

```text
/home/i6user/Desktop/Robotiq_DCU_CB3_USB_ROOT
```

备用卸载 USB 根目录文件夹:

```text
/home/i6user/Desktop/Robotiq_DCU_UNINSTALL_USB_ROOT
```

## 拷贝到 USB

插入一个空 USB 盘，先看它挂载到哪里:

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS,MODEL,TRAN
```

假设挂载点是 `/media/i6user/USB`，执行:

```bash
USB=/media/i6user/USB
rsync -a --delete /home/i6user/Desktop/Robotiq_DCU_CB3_USB_ROOT/ "$USB"/
sync
```

注意：USB 根目录必须直接看到这些文件/文件夹，不能再包一层文件夹:

```text
urmagic_robotiq_gripper.sh
driver/
ftdi_driver/
gripper_client_gui/
robotiq_2f_gripper_programs_CB3/
robotiq_2f_gripper_programs_CB2/
robotiq_2f_gripper.sh
uninstall.sh
```

## 在机器人上安装

1. 确认旧 Robotiq URCap 已经删除，不要让 `.urcap` 和 DCU driver 同时存在。
2. 把 USB 插入 UR 控制柜或示教器 USB 口。
3. 正常会自动弹出大字提示 `! USB !`，然后显示正在安装 `Robotiq 2F Gripper Driver DCU-1.0.10`。
4. 安装过程中不要拔 U 盘，不要关机。
5. 看到 `<- USB` 后拔出 USB。
6. 重启机器人。

## 安装后检查

SSH 到机器人后可以检查:

```bash
ls -lah /root/robotiq_2f_gripper_driver
ls -lah /root/gripper_client_gui
ls -lah /programs/robotiq_2f_gripper_programs
/etc/init.d/robotiq_2f_gripper.sh status 2>/dev/null || true
ps aux | grep -i robotiq | grep -v grep
```

如果 `/programs/robotiq_2f_gripper_programs/rq_script.script` 存在，说明脚本已经复制到控制器。

## 外部控制程序需要改的地方

你现在用的 `ExternalControl.urp` 只负责接 ROS 的机械臂运动。要让 ROS 发 `rq_open_and_wait()`、`rq_close_and_wait()` 生效，ExternalControl 程序里还要在 `Before Start` 里加入两个 script file:

```text
/programs/robotiq_2f_gripper_programs/rq_before_start.script
/programs/robotiq_2f_gripper_programs/rq_script.script
```

然后在 `Before Start` 里创建两个变量并赋值为 0:

```text
rq_obj_detect = 0
rq_pos = 0
```

这样 ExternalControl 运行时，`rq_*` 函数才会存在。之后 ROS 里才能用:

```bash
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String \
"{data: 'def rq_open_test():\n  rq_open_and_wait()\nend\n'}"
```

## 如果安装后又出问题

把 `/home/i6user/Desktop/Robotiq_DCU_UNINSTALL_USB_ROOT` 里的 `uninstall.sh` 放到空 USB 根目录，插入机器人，它会自动卸载 DCU driver。

