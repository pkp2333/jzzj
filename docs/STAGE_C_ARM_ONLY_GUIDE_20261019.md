# C 阶段单独左臂示教测试

此入口只执行人工录制的关节序列，不运行 IK，不调整底盘，不自动交接 D 阶段。与底盘对准模式使用不同模板，空载验收哈希也不能混用。运动结果仍为 `MOTION_COMPLETE_AWAITING_OPERATOR`，水果是否抓住和运输是否掉落由人工验收。

## 前置条件

停止底盘驱动：在启动 `roslaunch chassis stage_c_micro.launch` 的终端按 Ctrl+C，等待其退出。确认底盘动力已禁用且车体已可靠固定；软件检查不能替代实体停车。其他导航、遥控、底盘控制器也应退出。保留唯一左臂轨迹驱动和正运动学服务。单独左臂动作不要求相机在线、目标坐标、底盘方向矩阵或低速标定；若使用可选的 `target` 记录命令，仍需要观测节点及相机。

每次真实运行必须显式传入 `--confirm-base-immobilized`。入口会持续检查底盘命令话题没有发布者和订阅者，底盘状态没有发布者，且六关节反馈仅来自 `/arm_left/arm_trajectory`。底盘控制器重新出现、反馈陈旧、取消、超时、到位失败时停止后续动作并请求机械臂保持。ROS 图检查不证明未注册到 ROS 的外部程序或实体绝对停稳。

沿用原配置中的硬件/标定/工作空间门控、发送原始位置 96–4000、URDF/FK 边界及桌面间隙、起点误差 0.035 rad、每段臂关节变化不超过 0.1 rad、夹爪单独动作、实测到位和保持检查。

## 1. 新建专用模板

以下命令在上位机 `/home/orangepi/jzzj` 执行；首次创建不会覆盖已有模板。

```bash
bash tools/stage_c/arm_only_grasp.sh init \
  --example deploy/stage_c_taught.example.yaml \
  --template .local/taught/arm_only.yaml
```

默认所有真实运动门控仍为 false、起点和序列为空。模板包含 `execution_mode: arm_only`，不能拿它运行底盘对准。仅在实际确认当前场地、桌面高度和准备执行真实测试后，编辑该副本中的 `real_motion_enabled`、`workspace_clear_confirmed`、`table_height_confirmed` 为 true。不要改 `base_calibrated`、矩阵或编造关节角，这些数据在本入口不需要。

## 2. 示教

```bash
bash tools/stage_c/arm_only_grasp.sh teach \
  --template .local/taught/arm_only.yaml --config deploy/stage_c.yaml \
  --real --confirm-base-immobilized
```

记下打印的 Session。进入 `taught>` 本身不发送抓取轨迹。先输入 `start`，采集六关节实测起点和桌面/TCP/URDF/舵机映射绑定。用 `jog 关节编号 增量角度` 显式点动，建议每次 1°，单次最多 3°；例如 `jog 6 1` 只表示第六关节增加 1°，正负方向和实际张合方向必须现场确认。每次都经过 FK 和实测到位检查。

到达真实示教点后依次记录：

```text
record open 0.8
record approach 0.8
record descend 0.8
record close 0.8
record lift 0.8
record transport 0.8
check
quit
```

这些 `record` 命令只保存当前回读姿态，不会自动移动到对应阶段。必须先点动到位再记录；接近、下降、抬升和运输可录制多个中间点。open/close 只改变夹爪，其余阶段保持夹爪。没有安全完整的序列时不能执行 run。每次记录写盘；模板变化会清除空载验收。工作空间或桌面检查失败时停止查明原因，不改容差强行通过。

## 3. 会话释放

真实动作前锁存 `/stage_c/taught/recovery_required`，会话退出仍保留，以阻止旧 C Action 和新测试并行接管。确认机械臂停稳、空载、区域清空后，用原 Session：

```bash
bash tools/stage_c/arm_only_grasp.sh release \
  --template .local/taught/arm_only.yaml --config deploy/stage_c.yaml --real \
  --confirm-base-immobilized --session ORIGINAL_SESSION --confirm-stopped-and-clear
```

释放只接受实际执行过动作的同一单臂 Session，不能代替底盘对准会话释放。它检查无活动轨迹、四帧新鲜静止反馈，并重新执行测量保持确认，然后清除本流程恢复标志。它不释放/使能底盘，也不清除原 C 服务的 STOP_FAILED 或持物锁存。保持失败不会报告释放成功。若仅进入示教后退出且未发送任何动作，不会产生单臂动作恢复锁存，无需释放。

## 4. 空载及水果测试

人工恢复到相同示教起点后，先运行空载序列：

```bash
bash tools/stage_c/arm_only_grasp.sh run --empty \
  --template .local/taught/arm_only.yaml --config deploy/stage_c.yaml \
  --real --confirm-base-immobilized
```

实测起点不匹配时直接拒绝，不自动返回起点。完整动作和保持成功、人工确认空载路径安全后，绑定这次记录：

```bash
bash tools/stage_c/arm_only_grasp.sh approve-empty \
  --template .local/taught/arm_only.yaml --log .local/taught-runs/SESSION.json
```

释放上次会话、人工恢复起点，将水果放到示教夹取位置，手移开后执行相同 run 命令但去掉 `--empty`。本入口不会视觉纠偏；水果位置错误时固定动作也不会跟随。人工验收用原工具的 `confirm --verdict success|failure|unknown`，同样通过 `arm_only_grasp.sh` 调用。

Ctrl+C 或 `/stage_c/taught/stop=true` 请求取消。不会自动重试、返回起点或推进任务。若保持未确认，按实体停止措施处理；不要删除锁或直接清参数恢复。

## 本地验证

2026-10-19：Linux 环境 `test_taught`、`test_taught_base`、`test_taught_ros`、`test_control_repair`、`test_ros_contracts`、`test_servo_position_limits`、`test_hold_confirmation` 共 108 项通过，包含单独左臂模式与原底盘对准模式、无底盘调用、FK-only、模式隔离、确认门控、控制器重现、真实示教入口、会话恢复隔离、取消及保持失败。未用本次代码发送实体动作；现场测试另行记录。
