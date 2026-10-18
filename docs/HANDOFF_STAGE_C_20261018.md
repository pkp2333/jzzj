# 阶段 C 交接文档（无深度平面定位 + 左臂抓取）

> 写给下一位开发者。本文档记录阶段 C 的当前状态、本轮调试解决的全部问题及根因、环境拓扑、关键经验和待办。
> 代码基线：本地 `plus` 分支提交 `04da369`（尚未推远程，缺 GitHub 凭证）。

---

## 一、当前状态总览

| 模块 | 状态 |
|---|---|
| 感知（perception.py） | ✅ 双模式：`localization_mode: depth|plane`；plane 模式用颜色掩膜射线×桌面平面求交，实测精度 **5mm** |
| 观测（stage_c_observer.py） | ✅ plane 模式跳过深度订阅；与部署检测器（分区话题+species 标签）联调通过 |
| 抓取引擎（core.py） | ✅ SERVO_PREGRASP 位置闭环 + disappearance 消失验证分支 |
| 左臂执行（backends.py + arm_trajectory_node） | ✅ 轨迹 Action + IK 服务 + 夹爪实测核对 |
| 离线测试 | ✅ 55 + 24 + 36 全绿 |
| 实机抓取 | ⏳ **尚未成功执行过一次**：TCP 刚修好（工作区门控已放行），差一次实际抓取验证 |

---

## 二、环境拓扑（下一位最先需要知道的）

### 2.1 双工作区（本阶段最大坑源）

- `/home/orangepi/jzzj-stagec`：git 快照（当前在 `plus` 分支）
- `/home/orangepi/mission_ws`：**实际运行副本**，rosrun/roslaunch 解析到的是它

**纪律：改完代码必须 `cp` 同步到 mission_ws，然后 `catkin_make`（C++）或直接重启（Python）。** 曾因只改一边导致 rosrun 跑到旧代码、CRLF 修了不生效等问题。

### 2.2 相机：物理倒装 + 驱动软件翻转（组合 B）

- 相机物理倒装，plus 版驱动 `hp60c.launch flip_vertical:=true`（默认开）把 RGB+深度一起翻，K 同步修正——**光学坐标系不变，外参用原始矩阵即可**
- 结果画面：上下正立、**左右镜像**（预期行为，非 bug）
- 推论：若改物理正装，外参需重新推导（Rz(180) 修正）

### 2.3 检测器链路（plus 版，与旧版不同）

- 部署版 YOLO 只发布 `_left`/`_right` 分区话题 → **必须跑 `detections_merger.py` 合并**（已修 CRLF + 执行位）
- `species_map: [0,1,1,0,1,0]` 使标签变成 **cherry/lemon**（不再是 r_c/m_l），class_id 也变物种编号 → `allowed_labels: ["cherry","lemon"]`、`allowed_class_ids: [0,1]`
- 模型：`/home/orangepi/Documents/yolov5_weights/yolov5s.rknn`（竞赛模型）；比赛目标是仿真水果

### 2.4 I2C 排线是最大硬件隐患

- 曾出现"舵机有扭矩但 I2C 全总线扫不到控制器"→ 排线接触不良，重新插拔恢复
- 恢复后读取速率从 10Hz 降到 ~1.5Hz（排线老化），**瞬时停顿会突破反馈超时窗口**，级联引发 hold 失败 → STOP_FAILED → 服务端锁存。已把 `joint_timeout` 放宽到 2.0s、`feedback_timeout` 1.5s 容错，但**建议重新压接排线**
- I2C 地址：左臂 0x10（bus 1），Arm_Lib 约定 joint1/5 ±135°、joint2/3/4 ±90°、joint6（爪）0~100°

### 2.5 roscore 稳定性

- 后台 nohup/setsid 起的 roscore 会莫名死亡；**死后所有终端链路全断、参数全丢**，需重启终端 + 重 load 参数 + 重 latched selection
- 建议：roscore 由开发者在固定终端手动跑，不要后台化

---

## 三、本轮解决的问题清单（问题 → 根因 → 修复）

### 工程/环境类

1. **CRLF 换行符**：Windows 新建的脚本 shebang `python3\r` 失效 → `sed -i 's/\r$//'`（**两份工作区都要修**）
2. **执行位缺失**：`detections_merger.py` 无 +x，roslaunch 静默不启动 → `chmod +x`
3. **catkin 白名单缓存**：`catkin_make --pkg X` 会把白名单写进缓存，之后全量构建"成功"但什么都没编 → `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`
4. **现役 mission_ws 未重编**：消息包找不到（之前靠叠加快照环境才能跑）→ 全量重编
5. **roscore 反复死亡** → 见 2.5

### 视觉类

6. **相机倒装致 YOLO 失明**：A/B 对比实锤（倒图 num=0 vs 正图 score 0.807）→ 采用驱动翻转（组合 B）
7. **深度不可用（放弃直接原因）**：光滑球面反光，结构光不返回——黄色掩膜区仅 **1%** 深度有效（要求 50%），同框红色桌面背景 53% 有效 → 用户决策弃深度，实施平面定位
8. **光照是检出率的第一因素**：今天约 80% 的"检测不出来"最终都是灯关了（亮度 40 vs 正常 50+）。暗光下把检测阈值降到 0.3（`deploy/detector.yaml`）、感知 min_confidence 0.25
9. **平面模式 targets 为空的三个叠加原因**：① 灯没开 ② 外参旋转错误 ③ selection 锁存被服务端覆盖（见下）

### 标定类（本轮核心工作）

10. **外参历史约定不可信**：旧 TF `car_2_arm [0,90,90]` 与 URDF（joint1 轴=z 竖直）矛盾，沿用它导致 camera_to_arm 旋转错误（光轴算成水平朝上，实际俯视 38°）→ **放弃考古，改用 `cv2.solvePnP` 三点实测标定**（绕开所有欧拉角约定）
11. **180° 偏航修正**：标定时定义的 x 朝箱子，但 URDF 实际 -x 朝箱子（FK 腕部落点暴露）→ 外参绕 z 转 180°，Y 分量也吻合后确认，水平误差 5mm
12. **关节符号映射**：identity 假设错误，**joint2 实际符号为 -1**（FK 腕部与物理位置矛盾暴露；README 早有警告"不要盲抄旧代码第 2/6 轴符号"）→ `deploy/arm.yaml: servo_signs: [1,-1,1,1,1,1]`
13. **TCP 偏移两次错误**：
    - 几何猜测 `[0.13,0,0.015]`（沿 link5 +x）→ 算出爪尖在桌面下 9cm → 工作区门控拒绝（"臂不动"）
    - 按当前姿态物理约束反解：`[0.053, 0.079, -0.051]` → 爪尖 (-0.240, 0.090, -0.030)，**工作区检查 PASS**（约 2~3cm 误差，待第一次抓取后按落点修正）
14. **平面模式参数适配**（我的整合疏漏，已修）：
    - `min_object_height_m` 必须允许 0（plane 模式观测 Z 恒等于平面高度，>0 恒不满足）→ `core.py` 校验改 `0<=min<max`
    - `surface_to_grasp` 从 -0.015（深度模式语义：从顶面下探）改为 **+0.03**（平面模式语义：TCP=桌面上方 3cm=果实半高）

### 会话/状态机类

15. **心跳时间戳冻结**：`rostopic pub -r 2` 的 `header: auto` 只在启动时填一次，之后 stamp 永远旧 → 引擎 context_timeout(1s) 判 stale → 全部 NOT_READY。改用 `deploy/heartbeat.py`（每包刷新 stamp）
16. **observer 静默**：服务端 goal 结束时会 latched 发布 `enabled=false` 的 selection，**盖住观察链路的 enabled=true**，observer tick 全部提前返回。每次抓取结束后要重新 latched observation-only 选择
17. **服务端 poison 机制**：任何失败 abort 时若 hold() 也失败（I2C 停顿），记 STOP_FAILED 并永久锁存 → **每次失败尝试后必须重启 stage_c_server 并换新 session**
18. **会话管理**：heartbeat session、client session 必须一致且每次换新；`rosnode kill` 后参数丢失要重 load

---

## 四、关键经验（血泪总结）

1. **先查灯**：任何检测异常，第一件事看画面平均亮度（≥50 才可靠）
2. **"臂不动"先读 TCP**：工作区门控拒绝 = 当前/目标 TCP 算错（外参/TCP/符号），不是门控的错；用关节角 FK + tcp_offset 算出 TCP 与工作区盒对照
3. **FK 是符号映射的试金石**：identity 假设对不对，算一次腕部落点与物理照片对照即知
4. **双工作区同步**：改哪边都要想到另一边；rosrun 解析的是 mission_ws
5. **detections_merger 必须活着**：observer 的检测输入全靠它；它死了 observer 静默且**不报错**
6. **每次抓取尝试的检查单**：roscore活着 → 终端1相机 → 终端2观察 → selection latched=true → server未poison → heartbeat session匹配 → 灯开着 → 爪不挡柠檬

---

## 五、配置文件（都在 `/home/orangepi/jzzj-stagec/deploy/`，本地专用不入库）

| 文件 | 内容 |
|---|---|
| `stage_c.yaml` | 观测+执行：plane 模式、外参（原始矩阵+Rz180 已烘入）、平面 z=-0.05、workspace ±0.32、门控全 true |
| `detector.yaml` | 竞赛模型、conf 0.3 |
| `arm.yaml` | signs `[1,-1,1,1,1,1]`、URDF 限位、SDK 限位、门控 true |
| `kinematics.yaml` | tcp_offset `[0.053, 0.079, -0.051]`、gripper_limits `[0, 1.7453]` |
| `heartbeat.py` | 2Hz 上下文心跳（刷新 stamp），用法 `heartbeat.py 会话ID` |
| `action_jog.py` | 通过 Action 接口单关节 jog（弧度），不需要硬件租约 |
| `jog_joint.py` | 直接 I2C jog（会被租约拒绝，仅 trajectory 节点未运行时用） |
| `capture_point.py` | 标定点捕获（像素↔臂系坐标） |

启动栈（roscore 稳定后）：
```bash
rosparam load deploy/kinematics.yaml /stage_c_kinematics
rosrun jzzj_grasp stage_c_kinematics                                  # 运动学
rosparam load deploy/arm.yaml /arm_left/arm_trajectory
rosrun arm arm_trajectory_node.py __ns:=/arm_left __name:=arm_trajectory   # 臂轨迹
rosparam load deploy/stage_c.yaml /stage_c_server
rosparam set /stage_c_server/backend real
rosrun jzzj_grasp stage_c_server.py __name:=stage_c_server           # 抓取服务
python3 deploy/heartbeat.py grasp-XXX                                # 心跳
rostopic pub -l /stage_c/selection jzzj_grasp_interfaces/TargetSelection "{session_id: observation-only, target_color: yellow, enabled: true}"
rosrun jzzj_grasp stage_c_client.py _session_id:=grasp-XXX _target_color:=yellow   # 发抓取
```

---

## 六、待办（按优先级）

1. **完成第一次真机抓取**：TCP 已修、门控放行；重启终端 1/2 后直接发 grasp 指令，观察落点偏差并修正 tcp_offset（一次迭代即可收敛）
2. **重新压接 I2C 排线**：读取 1.5Hz 是级联失败的根因，修了能消除一半的异常
3. **README 验收标准**：固定工况连续 5 次成功；抓空/遮挡/交接失效不得误报（结果码须为 NO_TARGET/GRASP_UNCONFIRMED/NOT_READY，holding_known=false 时绝不松爪移车）
4. **远端推送**：本地提交 `04da369`；`/home/orangepi/stagec_changes.bundle`（9.1K）和 `.patch`（34K）已备好，拿到 GitHub 凭证后推
5. **正式目标**：比赛用哑光仿真水果（cherry/lemon）；测试用的光滑黄球对深度模式不可用（平面模式可用，但无反光特征更好）
6. **rocore 固定启动方式**（写进 launch 或开机服务），避免后台进程猝死
7. **I2C 反馈超时**：排线修好后可把 joint_timeout 收回 0.5s、feedback_timeout 0.6s（提高异常发现速度）

---

## 八、常见问答（给下一位）

### Q：机械臂不动时，你是通过哪个入口发送抓取指令的？

**入口是阶段 C 的 Action 客户端脚本**（不是话题、不是服务）：`jzzj_grasp/scripts/stage_c_client.py`，它向 `/stage_c/grasp` action server 发目标。实际命令：

```bash
source /home/orangepi/catkin_ws/devel/setup.bash
source /home/orangepi/mission_ws/devel/setup.bash
rosrun jzzj_grasp stage_c_client.py _session_id:=grasp-009 _target_color:=yellow _timeout:=90
```

前置条件（缺一不可）：roscore、终端1相机、终端2观察链路、selection latched=`enabled:true`、`stage_c_server`（backend=real）未 poison、heartbeat 的 session 与 `_session_id` 一致。

**实际返回样例**（本轮调试真实输出）：

```
# 1) 心跳 stamp 过期（rostopic pub 的 header auto 只填一次）
stage C result: code: 1
message: "base/arm handoff absent, stale, stopped or wrong session"

# 2) 观测无目标（灯没开/柠檬被挡住）
stage C result: code: 9
message: "no fresh matching target"

# 3) TCP 算到桌面下 → 工作区门控拒绝（臂不动的直接原因，非门控故障）
stage C result: code: 9
message: "TCP path outside safe workspace"
```

判读：`code=0` 才是 SUCCESS；`code=9` 表示以 STOP_FAILED 结束（abort 时 hold 失败，通常=I2C 停顿或排线问题），**此后服务端锁存，必须重启 stage_c_server 并换新 session 才能继续**。抓取过程中客户端终端里 Ctrl-C 会触发取消（保持臂姿态）。

### Q：为什么每次失败后要换 session？

服务端对每个 session_id 只接受一次目标（防重放）；且失败 abort 若伴随 hold 失败会 poison 锁存。实践中：`heartbeat.py grasp-新ID` + 客户端 `_session_id:=grasp-新ID` 配对使用。

---

## 九、代码提交说明（`04da369`）

10 个文件 +331/-31：perception 平面求交双模式、observer plane 同步、core 消失验证+高度门控合法化、backends 夹爪实测核对、55 项测试、observe_full.launch、detections_merger、image_flip_node、params 默认键、CMakeLists toplevel 符号链接修复。**不含 deploy/（本地标定数据，设计上就不入库）。**
