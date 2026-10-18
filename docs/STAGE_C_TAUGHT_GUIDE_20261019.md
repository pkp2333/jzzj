# 单示教点抓取：底盘停车复测，机械臂固定序列

本轮仅在本地实现和离线验证，没有部署、启停上位机任务或发送运动。本说明及现场验收记录保存在本地，部署时只打包代码和必要配置。

## 行为与边界

视觉得到水果在左臂基座坐标系中的位置，与示教参考坐标比较。底盘仅在桌面平面内沿一个底盘轴小步平移，停车后重新测量；进入 5mm 范围并稳定至少 1s 后锁住底盘，再执行固定机械臂关节序列。第一版只支持一个抓取点、同一桌面高度、黄色目标，不转向、不在线修改机械臂落点。

新流程禁止调用 IK，正运动学仅用于已记录关节路径的 TCP 工作空间和桌面间隙检查。没有完整网格碰撞检查，路径必须现场空载验证。平面定位的 Z 来自标定桌面，不是水果高度测量。

默认上限：单步 10mm、0.02m/s、累计 100mm、15 步、60s；同时检查视觉估计的累计位移，单步实测超过 15mm（10mm 加观测余量）直接退出。停稳观测至少三个不同时间戳、时间跨度至少 1s，目标散布不超过 5mm；错误会话、旧帧、空帧和多目标不能沿用旧坐标。对准期间机械臂必须保持示教起始姿态。

## 新接口与互锁

- `user_msgs/StageCBaseCommand`，话题 `/stage_c/base/command`：采集时间、会话、递增 command_id、操作 ACQUIRE/MOVE/HOLD/LOCK/RELEASE、vx/vy、显式人工释放确认。没有角速度字段。
- `user_msgs/StageCBaseStatus`，话题 `/stage_c/base/status`：会话、已接收 command_id、IDLE/ALIGN/LOCKED/FAULT、串口零速写入结果、诊断。
- `/stage_c/taught/stop`（std_msgs/Bool）：true 请求取消。也可以在前台工具按 Ctrl+C。
- `/stage_c/taught/recovery_required`：取得底盘后锁存。旧 C Action 和新示教运行都拒绝新任务，直到人工确认停稳、手已清空并显式释放。

原 `/stage_c/grasp` Action 定义和 A/B/D/E 的任务逻辑不变；原入口与新入口使用同一个 Linux flock 执行锁。新入口不返回自动抓取成功、不推进 D 阶段。

底盘专用模式 20Hz 发令/写串口，0.3s 指令失效立即停车并锁存 FAULT，不能通过重发 MOVE 恢复。普通导航/遥控的运动及仓门命令在专用模式内被拒绝，外部 HOLD、ARRIVE、零速和 task=false 始终停车。退出/切换模式清空缓存，避免重放旧速度。只有专用模式绕过 0.06m/s 起步补偿；普通速度换算保持原行为。串口写超时设为 0.1s。

串口写成功只是主机发送证据，停车判定还需要新鲜视觉稳定性；不是编码器/MCU 对停稳的确认。协调器退出而底盘驱动存活时软件看门狗可以停车；底盘驱动本身崩溃或主机失联必须依靠控制板看门狗。必须验证后才能设置 `controller_watchdog_verified=true`，否则专用模式拒绝取得控制权。若固件没有该能力，先修复固件，不能只改确认标志。

## 构建准备（后续部署时执行，本轮未执行）

本地缺少 ROS1 genmsg/genpy/rospy，因此没有声称完成 ROS1 消息生成或 ARM64 构建。部署只复制审核过的代码，保留现场未提交配置，先建回退提交或备份，不上传本地报告和日志。

在上位机工程根目录使用 `bash tools/stage_c/build.sh`，先生成新增 user_msgs 并构建 chassis/user_params，再构建 mission_ws。源码及安装空间都需验证新工具和底盘 helper 可找到。该脚本只构建，不启动任务。

不要重复启动底盘驱动。后续现场先停用其他导航、遥控和视觉底盘控制入口；新工具检查 `/cmd_vel`、`/car_cmd` 没有其他发布源。只允许一个真实左臂驱动、一个底盘驱动、一个目标观测源。相机、检测、stage_c_observer 和提供正运动学的 stage_c_kinematics 仍需运行。

底盘独立启动文件 `chassis/launch/stage_c_micro.launch` 不启动键盘控制。其 `micro_config` 默认指向 `chassis/params/stage_c_micro.example.yaml`，三个真实启用/低速标定/控制板看门狗确认均为 false。复制为现场配置，核验方向、速度和停车后再明确设置；不要修改全局普通模式的起步补偿。

低速校验与方向矩阵标定不同：先实测驱动 0.02m/s 命令的实际低速、量化和停车距离，校准线速度系数后才能确认驱动 `calibrated`；随后再采集视觉方向矩阵。低速不能稳定执行时不要提高上限硬凑结果。

## 工具与模板

所有模板均使用 JSON 格式（是 YAML 的子集），坐标为米、关节为弧度、时长为秒。示例 `deploy/stage_c_taught.example.yaml` 没有伪造坐标、关节角或矩阵，真实运动默认关闭。

离线创建副本，不覆盖已有文件：

```bash
bash tools/stage_c/taught_grasp.sh init \
  --example deploy/stage_c_taught.example.yaml --template .local/taught/tabletop.yaml
```

工具的离线子命令也可直接用具备 NumPy 的 Python 运行脚本（需将两个 mission Python src 路径加入 PYTHONPATH）。`check` 只检查数据结构，不代表现场绑定和路径已通过。

真实操作统一要求 `--real --config deploy/stage_c.yaml`，还要求模板 `real_motion_enabled`、`workspace_clear_confirmed`、`table_height_confirmed` 为 true，以及现有驱动/模型/标定门控已确认。模板 table_normal 必须与现场配置一致。不得为了通过工具而猜测这些条件。

### 1. 方向矩阵标定

在左臂处于收拢、安全、停稳姿态时执行：

```bash
bash tools/stage_c/taught_grasp.sh calibrate-base \
  --template .local/taught/tabletop.yaml --config deploy/stage_c.yaml --real
```

终端打印本次 Session，保留它用于退出后的显式释放。交互命令 `pulse x 0.01`、`pulse x -0.01`、`pulse y 0.01`、`pulse y -0.01` 每次仅作 5..10mm 的名义小步，前后都停车观测；每步必须独立测量并输入实际有符号底盘位移（米），不能把命令位移当作实测值。`fit` 用至少四个正反方向样本拟合矩阵，检查双轴可辨识、尺度和不超过 3mm 的残差；失败不启用模板。`quit` 保存后保持底盘锁定。

也可离线 `fit --samples samples.json --template ...`，样本字段为 `base_delta_m:[dx,dy]`、`before_m:[x,y,z]`、`after_m:[x,y,z]`。纯离线 fit 不采集当前标定绑定，仍需后续示教 capture 绑定。矩阵列是底盘 X/Y 的实测位移引起的水果平面坐标变化，允许轴反转与旋转，不写死正负号。

### 2. 关节示教

每次会话结束，先确认停稳、手和夹爪清空，再按下文释放；之后才能进入下一会话。示教前把水果放在拟定的固定抓取点，把机械臂恢复到已验证的收拢起点。

```bash
bash tools/stage_c/taught_grasp.sh teach \
  --template .local/taught/tabletop.yaml --config deploy/stage_c.yaml --real
```

交互命令：

- `target`：停稳后记录水果参考坐标、三个以上新鲜观测的时间、类别颜色及相机/桌面/TCP/关节映射绑定。
- `start`：在尚未录制动作时记录完整六关节起点。已经有动作时不允许悄悄覆盖起点；重教应新建模板。
- `jog 1 1`：1 号关节增加 1°；允许关节 1..6，单次绝对增量不超过 3°。使用轨迹 Action、关节反馈和正运动学边界检查，禁止直接写 I2C。每个 jog 是明确的手动动作。
- 到位后 `record open 0.8`，随后依次录制 `approach`、`descend`、`close`、`lift`、`transport`。接近、下降、抬升、运输可各有多个中间点。open/close 各一次，只改变夹爪；其他阶段保持夹爪。采集实际回读角度及时间，不写入旧 IK 轨迹。
- `check`：检查完整顺序、硬限位与固定轨迹的正运动学扫掠，不运动。
- `quit`：保存。每次记录立即写盘，改动模板会清除原空载验收绑定。示教会话最长 30 分钟。

如果点动无法到位或保持失败，立即退出排查，不跳过对应阶段、不放宽容差。位置编码保持发送 96..4000、反馈域独立；第 5 舵机机械零位不得当作原始编码零。

### 3. 空载执行与绑定

```bash
bash tools/stage_c/taught_grasp.sh run --empty \
  --template .local/taught/tabletop.yaml --config deploy/stage_c.yaml --real
```

仅用于真实空载验证。底盘只取得锁定，不做视觉对准或平移；机械臂执行完整示教序列并确认最终保持。模板起点必须已匹配，工具不自动回到起点。结果保存 `.local/taught-runs/SESSION.json`。

```bash
bash tools/stage_c/taught_grasp.sh approve-empty \
  --template .local/taught/tabletop.yaml --log .local/taught-runs/SESSION.json
```

只接受同一模板哈希、完整空载动作完成且保持确认成功的记录。任何模板改动或绑定改变都需要重新验证。

### 4. 单次水果测试和人工验收

释放上一次会话、恢复示教起点、确认场地和桌面条件后运行：

```bash
bash tools/stage_c/taught_grasp.sh run \
  --template .local/taught/tabletop.yaml --config deploy/stage_c.yaml --real
```

只有匹配绑定、已有同版本空载记录、全部固定关节目标有效时才启动底盘对准。每个细分动作不超过 0.1rad，执行后必须真实回读到位。抬升和运输完成只记录 `MOTION_COMPLETE_AWAITING_OPERATOR`，不是 C 阶段自动成功。

```bash
bash tools/stage_c/taught_grasp.sh confirm \
  --template .local/taught/tabletop.yaml --log .local/taught-runs/SESSION.json --verdict success
```

`--verdict` 可为 success/failure/unknown，只更新人工记录，不发布 D 阶段交权。抓空、掉物、未看清均不能写成功。不同固定工况需重新标定/示教；同一工况连续成功 5 次后才接受。

### 5. 释放与异常停止

Ctrl+C 或发布 `/stage_c/taught/stop=true` 请求取消；工具确认机械臂保持、底盘零速后退出。没有自动重试、自动返回或自动释放。超时/故障后底盘留在 LOCKED/FAULT，即使工具进程退出也不能被普通导航指令解锁。

物理确认停稳、空载及周围清空，且没有正在执行的机械臂目标后，使用原 Session 显式释放：

```bash
bash tools/stage_c/taught_grasp.sh release \
  --template .local/taught/tabletop.yaml --config deploy/stage_c.yaml --real \
  --session ORIGINAL_SESSION --confirm-stopped-and-clear
```

仍存在原 C 服务的 STOP_FAILED/持物锁存时，先进行人工恢复；工具不会代清其锁存。若无法确认停车，使用物理停止措施，不能用重启掩盖状态。重启 ROS master 后锁存参数会丢失，仍必须重新检查实体状态。

## 回退

本地代码按底盘互锁、示教协调器和说明分别提交。部署前保存现场提交和配置。回退前先停稳、清空并停止相关 C 会话，保留所有现场改动，再反向回退对应提交。新增消息不会改变已有消息定义；普通模式默认关闭专用入口。恢复旧模式前必须清空底盘速度缓存，不能通过删锁文件或直接发送导航速度强行交权。

## 本轮本地验证记录

- 分支：`fix/stage-c-arm-control`。
- 底盘互锁、专用消息和构建依赖：`67d56e3`。
- 示教协调器、无 IK 适配、轨迹等待修复与测试：`023409a`。
- WSL 本地相关包测试 229 项：226 通过、3 项缺外部资料跳过，包含实际 URDF/KDL 测试。新增测试覆盖实际共享底盘驱动的普通模式、起步补偿隔离、看门狗、停止覆盖、锁存恢复，示教流程收敛/超限/取消/异常，以及 ROS 适配器只允许 forward_only 的边界。
- Python AST、XML/launch、JSON 模板、shell 语法及 LF 换行检查通过。原有未提交报告保留。
- ROS1 消息生成、ARM64 集成构建、速度/方向标定、完整实机示教及 5 次抓取验收尚未执行；本机没有 genmsg/genpy/rospy。本轮无 SSH 部署和运动命令。
- 验证摘要：`.local/stage-c-taught-validation-20261019.json`。后续现场必须另存实际消息生成、驱动状态、每段反馈和人工验收记录，不能把本轮离线结果替代现场验收。
