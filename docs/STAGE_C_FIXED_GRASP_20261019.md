# 固定关节抓取程序

本入口替代手工在测试终端输入 `wb` 和两次 `a`。每次调用只执行一轮：后转待命、等待水果进入参考区域并稳定、固定接近、实测到位后等待 3 秒、闭爪、实测保持。没有 IK、自动微调底盘、自动抬升、运输或 D 阶段交接。

| 动作 | 终端语义角度，单位 ° |
|---|---|
| 后转待命 | `[-90, 0, 90, 90, 0, 60]` |
| 接近并张开 | `[-1, 56, 58, 50, 72, 60]` |
| 闭爪 | `[-1, 56, 58, 50, 72, 20]` |

这些数组在 `jzzj_grasp/fixed_grasp.py` 中固定，参考文件必须与它们完全匹配。夹爪不取反。SDK 映射为符号 `[1,-1,1,1,1,1]`、零偏 `[0,3,0,0,0,0]`，来自 2026-10-19 对现场左臂旧入口参数和控制源码的只读核实。三个关节目标都先进行 96–4000 原始值和关节范围检查，再允许后转动作。该映射不是通用标定，换硬件或零偏后应重新核验程序。

## 参考点与触发

现场已采集文件为 `/home/orangepi/jzzj/.local/fixed_grasp/orange_reference_20261019_174301.json`，原始坐标是 `[-0.2239400495909601, 0.06185350241391599, -0.049999999999999996] m`，标签 `class_id=1, label=lemon, color=yellow`。此前对话将 X 转述为 -0.23940 是误读，程序仅读取实际参考文件。

程序绑定采集时的相机内参、定位参数和桌面配置哈希。后转到位后重新建立观测窗口，要求当前会话、不同时间戳、采集年龄不超过 0.5 秒的同一类同色单目标，连续至少 1 秒、至少 3 帧，每帧到参考点的总距离不超过 5mm，且点间散布不超过 5mm。空帧清空稳定窗口，多目标、轨迹 ID 换位、观测失败或断流停止流程。最长等待 60 秒，超时保持，不自动重试。

平面模式的 Z 是桌面投影，不代表水果高度。5mm 触发范围、给定姿态和整个关节运动路径尚未完成实机验收；视觉坐标不能证明这条固定路径不会碰撞。

## 驱动与互锁

使用独立的 `stage_c_fixed_arm.launch` 启动现有轨迹驱动，保留 SDK/I2C 错误上报、0.035rad 到位容差、四次扫描的静止检查、取消和保持。该配置显式选择逐舵机写入模式，避开现场旧程序曾标注不可靠的批量写入链路。原有轨迹驱动默认仍使用原批量模式。

不能同时启动 `arm_test_rviz.launch`、旧 `arm_control_node` 或其他左臂控制器。执行程序拒绝旧关节命令发布源、重复关节反馈源及底盘控制入口，使用既有 C 执行锁，并在动作前锁存 `/stage_c/taught/recovery_required`。底盘应关闭控制并物理固定；软件图检查不能证明实体停车或检测所有不注册 ROS 的程序。

每次启动驱动会读六路反馈、写当前姿态保持、使能力矩。程序的 `check` 子命令完全离线，不启动驱动、不发送运动。参考配置 `real_motion_enabled=false`、launch 三个使能确认默认 false。不要为了运行而猜测零偏或伪造确认。

## 后续部署与运行

本次仅本地实现；报告留在本地。部署代码时保留现场所有未提交修改，尤其是 `arm_control_node.py`。正式部署应重新构建并验证 ARM64/ROS1 启动；本地未声称完成该集成验证。现有消息定义没有改变。

在上位机工程内先检查参考文件，不执行动作：

```bash
bash tools/stage_c/fixed_grasp.sh check \
  --reference .local/fixed_grasp/orange_reference_20261019_174301.json
```

关闭旧机械臂测试入口，确认当前六路回读、SDK 映射、空载路径及现场安全条件后，再启动专用驱动：

```bash
source tools/stage_c/env.sh
roslaunch jzzj_grasp stage_c_fixed_arm.launch \
  real:=true calibration_confirmed:=true workspace_clear_confirmed:=true
```

保留原始采集文件，新建现场审核后的私有副本，仅在实际准备执行时将其 `real_motion_enabled` 改为 true。相机、检测和 observer 需已运行，底盘固定。另一个终端显式调用一次：

```bash
bash tools/stage_c/fixed_grasp.sh run \
  --reference .local/fixed_grasp/orange_reference_run.json \
  --real --confirm-base-immobilized --confirm-fixed-path-clear
```

输出 Session 并保存 `.local/fixed_grasp/runs/SESSION.json`。三个关节运动目标的动作时长各为 3 秒，接近确认到位之后再独立等待 3 秒才闭爪。第一版没有碰撞模型/力控；夹爪碰到水果而未到 20° 时可能返回到位失败，不能把阻挡猜测为抓取成功。

正常完成只返回 `FIXED_GRASP_SEQUENCE_COMPLETE_UNVERIFIED`，`grasp_confirmed=false`，人工确认是否夹住。不会循环触发、自动返回或自动释放恢复锁存。下一次运行首先会打开夹爪，因此应先人工移除水果、确认机械臂停稳和场地清空，再用原 Session 释放：

```bash
bash tools/stage_c/fixed_grasp.sh release \
  --reference .local/fixed_grasp/orange_reference_run.json \
  --session ORIGINAL_SESSION --real --confirm-base-immobilized \
  --confirm-fixed-path-clear --confirm-stopped-and-clear
```

释放验证无活动轨迹并重新确认保持，只能清除本固定抓取会话的恢复标志，不能清除旧 C 服务 STOP_FAILED/持物锁存。Ctrl+C 或 `/stage_c/fixed_grasp/stop=true` 请求取消；反馈陈旧、通信错误、轨迹部分发送或停止失败时不推进后续步骤。不确认停稳时使用实体停止措施，不删锁文件强行交权。

## 一键启动（2026-10-19 增补）

在 Orange Pi 的 SSH 终端中执行，当前目录为 `/home/orangepi/jzzj`。部署本增补代码后先运行离线检查：

```bash
cd /home/orangepi/jzzj
bash tools/stage_c/start_fixed_grasp.sh
```

默认只校验现有私有参考文件，不启动 ROS 节点、不发送动作。准备执行一次固定抓取时：

```bash
bash tools/stage_c/start_fixed_grasp.sh \
  --reference .local/fixed_grasp/orange_reference_run.json \
  --real --confirm-base-immobilized --confirm-fixed-path-clear
```

该入口加载项目环境，检查恢复锁存和设备/ROS 节点冲突，再启动缺少的相机、检测/observer、专用左臂驱动和一次性抓取程序。完整且名称匹配的现有节点复用；视觉链不完整、未知关节发布者、旧机械臂控制或底盘活动时停止并提示，不自动杀进程或清锁。无 master 且没有残留设备进程时由 roslaunch 启动 master。真实运行需要私有参考文件中的 `real_motion_enabled=true`。

启动后最多等待 40 秒获得实际六关节反馈、轨迹状态、相机内参和新观察帧，随后仍执行原有标定、反馈、图检查及视觉门控。新启动的左臂驱动为 required 节点；I2C 无应答或六舵机反馈失败会使本次 managed launch 停止，不重试运动、不跳过反馈。相机/视觉就绪超时同样不会发送抓取轨迹。启动驱动本身包含当前姿态保持及使能力矩。

成功一次后抓取程序退出，其余节点保持运行，便于检查与按原 Session 释放；不会自动再抓。终端需保持打开；Ctrl+C 关闭本入口启动的节点，复用的外部节点不受本入口管理。Ctrl+C 不等于关闭舵机力矩。重复执行仍需按上文用原 Session 完成恢复释放。

本增补离线相关回归共 136 项通过（含既有未提交示教回归），新加 10 项覆盖依赖复用、冲突、残留进程、恢复锁存、显式真实运行门禁和实际消息就绪检查。尚未部署本增补或进行实体抓取；现场最新的 `Errno 6` I2C 无应答未被本增补解决。

## 回退操作

先停稳、清空夹爪并退出固定抓取程序与专用驱动；保留参考文件和现场修改，反向回退本次代码提交。不要通过切换入口直接执行旧 IK 流程。逐舵机模式是显式配置，旧驱动默认行为不变。原 C Action 和 A/B/D/E 任务逻辑未修改。

## 本地验证

2026-10-19：当前工作区相关离线回归 126 项通过，包括固定序列、视觉范围与稳定窗口、空帧/旧帧/目标换位、实际驱动逐舵机编码、部分发送、取消、保持和会话释放。Python AST、launch/package XML、shell 语法与换行检查通过。没有用本次代码执行实体动作；尚未进行 ROS1/ARM64 部署构建或固定路径的实机抓取验收。此前用户已取消的示教恢复改动仍作为未提交工作区修改保留，不纳入本次固定抓取提交。

## 位置触发半径调整为 20 mm

按用户本轮要求，新采集配置的 trigger_radius_m 为 0.020（米），使用目标与参考点的三维欧氏距离判定。运行时读取参考文件中的该值；兼容旧的 0.005 参考文件，旧文件不会静默放宽，现场需显式将私有运行副本更新为 0.02，保留原始采集文件。稳定时间 1 秒、至少 3 帧、时效 0.5 秒、帧间散布不超过 5 mm 以及固定角度均保持原值。采集时稳定性也仍按 5 mm 检查。

固定序列与启动相关离线回归 26 项通过，覆盖 19 mm 通过、21 mm 拒绝、三维距离组合超限、20 mm 范围内的抖动拒绝和旧 5 mm 文件兼容。尚未推送或执行实体动作。
## 位置触发半径调整为 50 mm

用户随后要求触发半径改为 50 mm；新配置 trigger_radius_m=0.050。保持 5/20 mm 旧参考文件兼容，现场私有运行副本必须显式更新为 0.05 才会扩大范围。三维距离判定、稳定观察 1 秒、至少 3 帧、帧间散布 5 mm、固定角度与参考点均保持原值。放宽触发范围不会根据目标坐标修正抓取角度。

固定序列和一键启动相关 27 项离线测试通过，包括 49 mm 接受、51 mm 拒绝、各轴分别不超但三维合成距离超限、抖动拒绝、5/20 mm 文件兼容。本地代码已修改；未执行实体动作。
## roslaunch 参数解析修复

现场报错 unrecognized arguments: __name:=... __log:=...。原因是 argparse 直接读取了 roslaunch 自动附加的 ROS 重映射参数，抓取程序在初始化 ROS/视觉判断之前就退出。现在先使用 rospy.myargv 去除 ROS 参数，再进行严格 argparse 校验；未知业务选项仍报错。普通离线 check 不导入 rospy。固定抓取与启动相关 30 项离线测试通过，包括 launch 参数兼容、未知选项拒绝和无 ROS 环境 check。尚未执行实体动作。