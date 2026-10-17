# 阶段 C：YOLO RKNN + RGB-D + 固定底盘左臂抓取

实现日期：2026-10-17。**真实后端代码已提供，但未完成 ROS/ARM64 编译或实机验收。** 当前本机只有离线 Python/NumPy 验证条件；模型、标签、Arm_Lib、机器人标定须由现场补齐。

## 新增模型接入资料

已补充外部 `yolov5-rknn` 水果模型的六类原始标签、完整哈希清单、专用配置、RGB-only 验证入口和历史输出回归工具。2026-10-18 用户确认类别顺序后，业务输出改为 `0/cherry`（樱桃）和 `1/lemon`（柠檬），原 0/3/5 与 1/2/4 分别合并，权重及六类张量结构保持原值。详见[水果模型识别链路接入说明](MODEL_FRUIT6.md)。该入口不自动开启抓取；原文中“尚需提供模型”指实际抓取部署/验证门槛，真实模型、相机和机械臂仍需目标机验收。

## 1. 能力和边界

- `yolov5_rknn` 是唯一目标发现来源。只在 YOLO 框内按 HSV 筛颜色，融合深度生成三维位置；不全图靠颜色找目标，不读 `ObjDet.pose` 默认零值。
- 底盘固定，不发布 `/cmd_vel`。仅控制左臂，支持独立 Action、取消、交接心跳、模拟后端和现有任务的可选适配。
- 通过目标位置与当前 TCP 做小步位置闭环，不采用固定相机下不成立的“左臂移动让目标回画面中心”。
- 流程：CHECK_READY → PREPARE → SEARCH_TARGET → SERVO_PREGRASP → DESCEND → CLOSE_GRIPPER → LIFT → VERIFY_GRASP → MOVE_TO_TRANSPORT_POSE → SUCCESS。
- 必须先得到新鲜、合法且符合桌面高度约束的目标，才允许发开爪命令；无目标不做盲目扫描。
- 默认不自动重试、不移动底盘、不释放物体、不实现 D/E、不提供通用碰撞规划。安全工作区只约束 TCP，不能证明全部连杆不会碰撞。

## 2. 目录与依赖方向

| 包 | 内容 |
|---|---|
| `jzzj_grasp_interfaces` | Action、观测/交接消息、运动学服务定义；不依赖业务包 |
| `jzzj_visual_servo` | ROS-free YOLO 框内颜色/深度算法、彩深配准、位置限步原语、ROS 观测节点；依赖接口包和 user_msgs |
| `jzzj_grasp` | 执行状态逻辑、模拟/真实后端、ActionServer、MyArm/KDL 服务、客户端和测试；依赖视觉包和接口包 |
| `arm` | opt-in 轨迹执行脚本；不依赖任何任务包，保留旧模式 |
| `jzzj_misson` | 默认关闭的 Action 客户端薄适配，原 A/B/D/E 算法不变 |

纯算法位于 `src/jzzj_grasp/core.py`、`src/jzzj_grasp/backends.py`、视觉包的 `src/jzzj_visual_servo/`。Action、ROS、设备库不会在纯算法导入时启动。

## 3. 构建与模拟

仓库原有构建前置问题仍需目标环境处理：`catkin_ws/src/CMakeLists.txt` 缺失，`mission_ws/src/CMakeLists.txt` 是普通绝对路径文本，不是有效脚本。请先备份并在实际 ROS 环境正确初始化；本轮没有替别人修复这些工作空间文件。旧整车流程的 GrabCmd/后置 AprilTag 缺项也未修复，C 不依赖它们。

完成前置处理后，在兼容 ROS 1 Linux 上先构建/source catkin_ws，再构建/source mission_ws。新接口包由 catkin 自动先生成。默认不构建 ARM64 MyArm 适配器，模拟不需要相机、模型或 Arm_Lib。

```bash
cd /home/orangepi/catkin_ws
catkin_make
source devel/setup.bash
cd /home/orangepi/mission_ws
catkin_make
source devel/setup.bash

roslaunch jzzj_grasp stage_c_sim.launch session_id:=stage-c-demo
# 另开终端，加载同样环境后执行：
rosrun jzzj_grasp stage_c_client.py _session_id:=stage-c-demo _target_color:=red
```

模拟目标会在夹取后随模拟 TCP 移动，属于功能回路示例，不是识别/夹持效果证明。`sim_context.py` 只允许连接私有 backend 参数为 sim 的标准服务端；真实服务端绝不能用模拟心跳冒充上游交接。

## 4. 上游接入契约

### `/stage_c/grasp`：`jzzj_grasp_interfaces/VisualGraspAction`

请求：`session_id`（非空且每次请求唯一）、`target_color`（red/green/blue/yellow）、`grasp_profile`（默认 tabletop）、`timeout`（ROS duration，正数且不超过 120 秒）。

- 单任务；忙时拒绝，不自动抢占。每个已接受的 session 只能使用一次，防止旧交接心跳被重复利用。运行失败后重试也要新 session。
- 一旦成功持物，或闭爪后失败导致持物未知，服务端锁存禁止新的开爪抓取请求。D/E 完成并由现场确认空爪后重启 C 服务端；当前首版不提供远程清除此锁存的接口，避免新 session 意外松开正在运输的物体。
- Feedback：step、arm_frame 下 target、position_error、diagnostic。Action GoalID 自带反馈/结果关联。
- Result：code、message、grasp_confirmed、holding_known、holding_object、ready_for_transport、stop_confirmed。

| code | 语义 |
|---|---|
| 0 SUCCESS | 持物已复核，运输姿态到位，允许交给 D |
| 1 NOT_READY | 配置/上游交权不完整、丢失或过期，或请求被拒绝 |
| 2 NO_TARGET | 等待内无匹配新鲜目标 |
| 3 NEEDS_REPOSITION | 目标/TCP 不在工作区、高度不合适或 IK 不可达，C 不自行移车 |
| 4 SENSOR_INVALID | RGB-D、内参、深度/配准等不合法 |
| 5 MOTION_FAILED | 运动/反馈/求解异常，已尝试保持 |
| 6 GRASP_UNCONFIRMED | 视觉持物证据不足，不能凭动作结束放行 |
| 7 TIMEOUT | 总时限到，已尝试保持 |
| 8 CANCELED | 取消且驱动已确认保持 |
| 9 STOP_FAILED | 停臂无法确认；服务端锁存故障，不再接受目标，现场处理 |

失败结果持物状态保守为未知，绝不能用 `holding_object=false` 而忽略 `holding_known=false` 来推断可以松爪或移车。

### `/stage_c/context`：`StageCContext`

上游以 **2 Hz** 发布：header.stamp、session_id、base_locked、arm_control_granted、stop_requested。时间戳年龄及接收间隔都不能超过 **1 秒**；拒绝明显未来时间戳。所有节点时钟须一致。

进入 C 前，上游必须真实保证：车停稳且导航不再输出、其他左臂控制源停用、抓取区无障碍并授予独占控制权。C 不以这个消息代替物理静止测量。

```bash
# 仅为消息形状示例。真实系统应持续提供实时状态，不可用固定 true 冒充检测。
rostopic pub -r 2 /stage_c/context jzzj_grasp_interfaces/StageCContext \
  "{header: auto, session_id: session-001, base_locked: true, arm_control_granted: true, stop_requested: false}"
```

动作执行途中交接失效、stop_requested 或取消均触发保持；不使用 `/task=false` 停止共享相机/检测器。等待 Action 终态并检查 stop_confirmed 后，其他模块才能接管左臂。物理停止无法确认时不继续 D。

## 5. 模型与观测接入

### 5.1 模型文件

使用你提供的 RKNN 和标签，不沿用旧水果模型。先运行元数据工具：

```bash
rosrun jzzj_grasp check_model.py --model /path/model.rknn --labels /path/labels.txt --class-num 3
```

标签首行是类别数，后面每行一个非空、不重复标签。工具输出 SHA256，仅检查文件/标签，不验证模型张量兼容性。当前推理适配器要求单 batch RGB 输入、三个按 stride 8/16/32 排列的 NCHW INT8 affine 量化 YOLOv5 输出头；其他模型明确拒绝，不能认为所有 YOLOv5 导出格式通用。模型转换/训练不在本轮范围。

复制 `params/detector.example.yaml` 到部署配置，填写实际路径、class_num、18 个 anchors。可选填写 expected_input_width/expected_input_height（0 表示以模型查询尺寸为准）。这个文件是**平铺键**，由 launch 加载到 `/yolov5_rknn_rc`，不要再套一层同名 YAML 根键。

本轮修正检测图像与 Header 成对快照、同一采集时间只处理一次；无目标的新帧清空结果，运行失败明确终止而非输出空检测。跟踪类名会随本次关联更新，但 C 仍不把 track_id 当可靠物体身份证明。

### 5.2 RGB-D 与观测配置

`params/stage_c.yaml` 是 JSON 格式的合法 YAML。复制为部署配置后填写 perception：

- allowed_labels 必须是实际模型允许抓取的标签；空列表拒绝生成目标。allowed_class_ids 可额外约束。
- rgb/depth/Image/CameraInfo 话题及各自 frame 必须与实机一致，不从设备名称猜光学坐标轴。
- 使用**已矫正、畸变 D 为零的图像与内参**；当前实现对非零畸变明确拒绝，不能只手动把 D 改零。
- 如果深度已配准到 RGB，确认 depth_registered_to_rgb，并提供尺寸及内参 K 都与 RGB 一致的配准流；否则需要标定 depth_to_rgb，启用 registration_calibrated，由代码做 Z-buffer 配准。
- 16UC1、32FC1 的比例分别标定，不能只由编码猜单位；示例 0.001/1.0 不是对厂商实际值的保证。
- camera_to_arm 是**相机光学坐标到左臂基坐标**的 4×4 刚体矩阵；可由相机→车体→左臂外参组合。与 TCP 标定分开记录。
- calibration_confirmed 默认 false；身份矩阵、示例参数不表示可上车。

检测 Header 必须能匹配原 RGB 时间戳，RGB/深度差默认不超过 40 ms，总观测年龄不超过 0.5 秒；ROI 内颜色掩膜经腐蚀、深度范围与 MAD 离群过滤后估计三维中位位置。相机停止、空检测、源帧重复都不能维持稳定计数。

观察调试（不启动任何抓取 ActionServer 或机械臂）：

```bash
roslaunch jzzj_grasp observe.launch config:=/path/stage_c.yaml target_color:=red
rostopic echo /stage_c/observations
# 有图形显示环境时，可启用 HSV 滑块；红色双区间请最终写入配置，而非只保存一个滑块区间。
roslaunch jzzj_grasp observe.launch config:=/path/stage_c.yaml target_color:=red tune_color:=true
```

`/stage_c/debug_image` 为 YOLO 候选叠加图；`/stage_c/observations` 为每帧完整候选（含 diagnostic）；`/stage_c/candidate_observation` 为候选调试流；真实执行时 `/stage_c/observation` 是执行器实际锁定的目标，不要把候选流当作唯一目标。

## 6. 实机左臂执行与运动学

1. 停止旧 arm_control_node、旧抓取脚本以及其他 SDK 直接调用者。旧/新驱动使用同一 Linux 进程锁，防止同时访问机械臂；轨迹节点也拒绝已有 joint_states 发布者。第三方不遵守锁的脚本仍需由现场停用。
2. 填写 `params/arm.example.yaml` 中六关节符号、SDK 零偏、弧度限位及 SDK 度限位。映射固定为 `raw_deg = rad*180/pi*sign+offset`，反馈严格反变换。不要盲抄旧代码的第 2/6 轴符号与偏置。
3. 显式启用硬件、标定和工作区开关后，轨迹节点才构造 Arm_Lib；它不会执行旧驱动的默认归位动作，但会读取姿态、发当前姿态保持并使能扭矩，因此启动本身也需现场安全条件。
4. `/arm_left/follow_joint_trajectory` 支持一个目标点、六个唯一关节名、position-only，时长 0.2–5 秒；多点、速度/力矩/路径容差等当前不支持，明确拒绝而非静默忽略。上层以小步目标实现闭环。
5. 执行完成需多个**不同反馈样本**到位且变化足够小；取消清待执行任务并保持实际姿态，确认后才 PREEMPTED。失败保持不自动松爪/卸力，STOP_FAILED 锁存至现场排查和重启。
6. Arm_Lib 本仓缺失，SDK 通信阻塞/内部位置缓存的新鲜度不能由本代码保证；若底层失效，Action 取消也不是物理急停。实机必须先核对 SDK 超时、反馈和急停。

运动学：目标机编译 `catkin_make -DSTAGE_C_BUILD_KINEMATICS=ON`，链接现有 ARM64 MyArm 库；不是 Windows/x86 可运行适配器。填写 `kinematics.example.yaml`，标定 link5→TCP 偏移和夹爪独立范围。MyArm IK 返回必须通过 URDF 五臂关节限位、连续性和 KDL 正解位置/姿态残差；不可满足时返回失败，不截断角度冒充成功。现有 claw joint6 的零限位不用于夹爪行程。

**不要假定示例俯抓四元数在五自由度臂上处处可达。** 空载验证抓取姿态、TCP、桌面平面及整个连杆通道后再设置 calibrated=true。

真实启动示例（所有部署文件必须已校准；默认例子不满足启动门槛）：

```bash
roslaunch jzzj_grasp stage_c.launch backend:=real config:=/path/stage_c.yaml \
  start_arm:=true arm_config:=/path/arm.yaml \
  start_kinematics:=true kinematics_config:=/path/kinematics.yaml \
  start_detector:=true detector_config:=/path/detector.yaml
```

已有相机/检测/驱动时，不重复启动，对应开关保持 false。只有需要本模块开 hp60c 时显式 start_camera:=true。服务器 execution 的 real_motion_enabled、robot_calibration_confirmed、workspace_clear_confirmed、model_validated 及所选 profile.calibrated 都必须确认；这些门槛不能代替前述实际测量。

桌面抓取 profile：table_plane_offset_m、vertical_axis、物体表面高度范围、surface_to_grasp、approach_height、lift_height、transport_position、transit_clearance_height_m、开闭爪角度。桌面平面定义为 `dot(vertical_axis, position)=table_plane_offset_m`，法向为 arm_frame 下单位向上方向。必须使用真实安装外参标定该法向，不能假定倾斜安装的左臂基坐标 z 就是物理竖直；升降、桌面间隙和抬升复核都沿此法向计算。先升至通道高度再横移，最后短段下降允许预期遮挡；这不替代完整连杆碰撞检查。

## 7. 总任务接入

```bash
roslaunch jzzj_misson jzzj_misson.launch external_stage_c:=true \
  stage_c_session_id:=session-001 target_color:=red
```

另行启动 C 服务端、真实交接心跳以及 A/B 模块；此命令不自动启动它们。适配器只在 PICK_ALIGN 发一次目标，并将 C 反馈映射到原抓取相关阶段用于显示，不对外重发旧抓取命令。人工 advance/旧 Bool 不再驱动这些阶段。当前 Action 成功且所有持物/运输标志完整才进入 MOVE_TO_DROP；迟到成功、旧回调、失败、取消不放行。STOPPED/FAULT 时也继续处理取消结果，未确认停车保持则故障锁定。

旧默认 external_stage_c=false 路径保留；本轮同时修正了遗留视觉框架的私有 remap 和参数命名空间，但新的 C 不使用那个 Twist 框架。不要同时启动旧抓取或旧视觉速度控制链。

## 8. 验证结果和未验收项

2026-10-17 本地完成 **63 项离线测试**：39 项纯算法/模拟/安全检查 + 24 项 Action/任务/轨迹回调替身检查。包含完整模拟、抓空/掉落、类别变化、同 ID 不等于持物、陈旧/重复帧、取消和保持失败等。

```bash
python3 -B mission_ws/src/mission/jzzj_grasp/tests/test_stage_c.py
python3 -B mission_ws/src/mission/jzzj_grasp/tests/test_ros_contracts.py
```

仅需 Python3 + NumPy；不需要 ROS、OpenCV、模型或 Arm_Lib。观测调试 ROS 节点另需 cv_bridge/OpenCV。内存替身不能证明真实 ROS API/消息生成/C++链接正确，因此下列工作**未执行，不能视为完成**：

- 两工作空间 catkin 构建、消息生成、MyArm/KDL C++ 编译及 RKNN ARM64 编译。
- 真实 Action/ROS 消息联通、模型推理、录制数据回放、相机配准/外参和 TCP 标定。
- 空载动作/取消/急停、软质目标抓取、运输姿态持物复核。
- 固定工况连续 5 次成功以及抓空、遮挡、交接失效和掉落不会误报成功的现场验收。

按“观测 → 空载执行/取消 → 软目标抓取 → 持物验证 → 上游交接”顺序推进；全程有现场人员、物理急停和必要限位，不把软件门槛当成物理安全系统。

## SSH 连续坐标 INFO（只读，不执行抓取）

RGB-D 观测节点默认启用 `log_arm_coordinates=true`。每个新鲜有效源帧中的每个候选目标输出一行 `[ARM_TARGET]`，包含采集时间、track ID、标签、颜色及 `arm_left/arm_base_link` 下的有符号 X/Y/Z（米）。相同时间戳不重复打印；多候选分别标注 ID，不宣称已经选定抓取对象。坐标轴正向以实际标定的机械臂基坐标为准，不直接假设为车体前/左/上。

`surface_m` 是目标表面的三维观测点；只有所选 `grasp_profile` 的 `calibrated=true` 且偏移有效时，才按与抓取核心相同的 `surface_to_grasp` 显示 `grasp_tcp_m`。否则显示 `UNAVAILABLE(profile_not_calibrated)`，不会把表面点或零偏移冒充实际夹取 TCP 点。日志不代表可达性/IK/碰撞校验通过，均标注 candidate_only=true。

需在真实彩深配准、深度比例、相机到左臂外参已核对的部署配置下启用 `enable_rgbd_observer:=true observation_config:=/path/calibrated_stage_c_fruit6.yaml`；当前 RGB-only 默认入口无法输出机械臂三维坐标。目标丢失、观测无效或标定缺失时不输出旧坐标，不应仅为显示数字开启标定确认开关。可通过 `log_arm_coordinates:=false` 关闭连续日志，`grasp_profile:=tabletop` 选择已标定偏移。
