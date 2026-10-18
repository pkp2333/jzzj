# 源码审查记录（2026-10-18）

本次核对了阶段 C 的观测、抓取编排、真实/模拟后端、轨迹驱动及运动学适配，以及原任务、导航、定位、底盘和相关启动配置。当前目录不是 Git 仓库，因此审查对象是现有源码快照，没有历史差异可比。初次审查阶段未修改业务代码；随后根据用户确认，已在原项目内实施下述整改。

## 后续整改（2026-10-18）

- 问题 1、2 已修改：导航检查源位姿时间戳与接收超时，失位后每周期发布零速度及 HOLD；USB 相机不重复发布旧帧，AprilTag 仅处理新鲜源帧，定位保留采集时间戳。
- 六类水果已按种类合并。相机方向按用户最新物理安装状态更新为默认不翻转；正常正焦距下 Tag 直接处理原图，旧软件反射仅保留可选兼容路径。物理调整后的外参需在现场重新核验。具体参数、双工作区同步与实机验收步骤见[本轮修改说明](REQUESTED_CHANGES_2026-10-18.md)。
- 最新交接附件中的部署版含平面定位、左右检测合并与实测配置，本目录未包含这些完整现场代码；下文有关本地检测话题/抓取的结论不能直接作为现场版本根因，不能以本地模板覆盖现场修复。
- 以下编号、行号和复现描述对应整改前的源码快照，保留用于解释原因；其余审查项未在本轮一并修复。离线回归通过不代表已完成 ROS/机器人实机验收。

发现 13 项具体问题。以下区分源代码可确认的问题与实机影响：未连接机器人，不能把离线复现当成硬件运行结论。

## 优先修复的运行问题

### 1. [P1] 导航曾获得定位后，再丢失定位不会触发停车分支

- 位置：[wp_nav_pid_node.py:166](../mission_ws/src/mission/wp_nav_pid/scripts/wp_nav_pid_node.py#L166)，另见第 193 行。
- 正常获得定位时把 `stop_flag` 设为 `1`；丢失定位时却只允许 `stop_flag == 0` 下发 `HOLD` 和零速度。之后没有把该标志恢复为 `0` 的代码。
- 触发条件：先正常导航，再让前后定位标志都变为 `False`。即使超过 1 秒也不会发出该分支的停车消息。
- 验证：直接调用真实类方法，模拟上述状态，两个发布器均未收到消息。
- 建议：按定位状态转换管理停车标志，定位失效时立即发送停车，并增加位姿接收时间/源时间戳检查。

### 2. [P1] AprilTag 相机断流后仍会用旧图生成“新定位”

- 位置：[apriltag_localization_node.cpp:136](../catkin_ws/src/algorithm/detection/apriltag_localization/src/apriltag_localization_node.cpp#L136)，另见第 241 行。
- 循环只检查缓存图像非空，不检查是否为新帧、是否过期。成功定位后又用 `ros::Time::now()` 填写位姿时间戳。
- 触发条件：最后一帧包含有效 Tag，随后相机停止发布。缓存图仍会被反复检测，并持续发布成功检测标志和当前时间戳的旧位置。
- 影响：上游无法从输出时间戳识别真实图像已过期，导航可能持续使用冻结位姿。这与问题 1 是独立缺陷。
- 建议：图像和 Header 一起快照，只处理新的采集时间戳，限制图像年龄，位姿保留采集时间，断流时发布定位失效。

### 3. [P1] `/task=false` 不会给底盘发送停车报文

- 位置：[chassis_control_node.py:130](../catkin_ws/src/actuator/chassis/scripts/chassis_control_node.py#L130)，另见第 218 行。
- 回调先等待 1 秒，再把 `task` 置为 `False`；控制定时器看到该值后直接返回。这个路径没有写入 `stop_data`，也没有执行 `shutdown()`。
- 验证：用串口替身调用实际 `task_cb()` 和 `controller()`，没有任何停车写入。
- 影响：软件仅停止更新，无法保证底盘停止；最终硬件行为还取决于未在本次快照中验证的 MCU 超时机制。
- 建议：停用前立即写停车报文，清理缓存速度，增加速度指令接收超时后的停车处理。

### 4. [P1] 视觉节点停用时仍持续发布全局零速度，干扰导航

- 位置：[jzzj_visual_servo_node.py:135](../mission_ws/src/mission/jzzj_visual_servo/scripts/jzzj_visual_servo_node.py#L135)，另见第 175 行。
- `enabled=False` 时每个定时周期仍调用 `_stop_output()`，向全局 `/cmd_vel` 发布零速度。默认频率为 20 Hz，导航节点也使用同一话题。
- 触发条件：视觉节点和导航同时运行，而视觉尚未取得控制权或已停用。
- 验证：直接调用实际 `_tick()`，停用状态依然发布了一条零速度。
- 建议：控制权交接时发送一次停止；停用后退出速度发布，或使用统一速度仲裁器管理各控制源。

### 5. [P1] 大模型输出通过校验后被固定测试任务覆盖

- 位置：[mission_manage_node.py:315](../mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py#L315)。
- 前面已接收、解析并校验 DeepSeek 输出，随后无条件执行 `ds_output.data = "[[熟柠檬, 绿色], ... , [2]]"`，再重新解析。
- 影响：不同用户指令最终执行同一组抓取/颜色配对和 2 号泊车任务；前面的正确解析结果被丢弃。
- 建议：移除覆盖语句，调试任务通过显式参数注入，执行已校验的 `mission`。

### 6. [P1] 原左右抓取节点订阅不存在的分流话题，并把无消息当作抓完

- 位置：[grab_test_left.py:46](../mission_ws/src/mission/mission_manage/scripts/grab_test_left.py#L46)；右侧同类代码位于 `grab_test_right.py:35`。
- 两节点订阅 `/ascamera_hp60c/rgb0/yolov5_detections_left` 和 `_right`。当前检测器发布的是 `/ascamera_hp60c/rgb0/yolov5_detections`，未找到分流发布者或相应启动重映射。
- 两节点初始化的检测数量为 0，等待约 5 秒后会把 0 判定为“果子全部抓完”，发布 `FIANL`。
- 影响：在当前启动配置下，补齐消息定义后仍可能完全没抓就报告完成。
- 建议：实现明确的左右分流或重映射，并区别“未收到有效检测帧”与“有效帧内没有目标”，检查检测时间戳。

### 7. [P1] 投放阶段拿颜色编号查询水果列表，配对结果错误

- 位置：[mission_manage_node.py:540](../mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py#L540)。
- `bakset_color` 是颜色编号，却调用 `fruit_cmd.index(bakset_color)`；已计算出的 `basket_color_cmd` 没有参与匹配。
- 验证：现有固定任务产生 `fruit_cmd=[1,2,0,3]`、`basket_color_cmd=[3,2,0,1]`。遇到绿色编号 `3`，当前代码选择水果 `3`，任务实际要求水果 `1`。
- 如果水果列表缺少某个颜色的同值整数，还会抛出 `ValueError`。
- 建议：先从 `basket_color_cmd` 找颜色对应任务，再映射水果及实际料仓编号，并检查配对完整性。

### 8. [P1] 抓取硬编码夹爪角度，转投放时又切回另一组角度

- 位置：[arm_motions.py:44](../mission_ws/src/mission/mission_manage/scripts/arm_motions.py#L44)，另见第 117 行。
- `grab()` 无条件把传入的 `loose/tight` 改成 `130/150`，忽略左右臂配置；后续 `put_to_basket()` 又使用调用方传入的原始参数。
- 验证：按左臂默认参数 `loose=60, tight=30` 调用真实动作函数，抓取结束的夹爪目标为 150°，投放第一条运动指令即变为 30°。
- 影响：两段动作对“持物”的夹爪目标不一致，投放移动开始时就出现 120° 的目标变化；是否掉物或触及硬件限位需实机标定，但参数失效和目标跳变可离线确认。
- 建议：统一使用经标定的参数，移动到释放位置之前保持同一个持物夹爪目标。

## 构建和原整车启动阻塞

### 9. [P1] 任务工作空间顶层 CMake 文件不是有效脚本

- 位置：[mission_ws/src/CMakeLists.txt:1](../mission_ws/src/CMakeLists.txt#L1)。
- 文件只有 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` 这一行普通文本，不是符号链接，也不是 `include(...)` 语句。
- 验证：执行 `cmake -P mission_ws/src/CMakeLists.txt`，实际报出 `Parse error. Expected a command name`。
- 另外，`catkin_ws/src/CMakeLists.txt` 当前缺失。
- 建议：在目标 ROS 环境中正确初始化两个工作空间的顶层文件，再进行干净构建。此前文档已经提到这些缺项，本次确认仍存在。

### 10. [P1] 原任务所需 `GrabCmd` 消息缺失

- 位置：[mission_manage_node.py:19](../mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py#L19)，左右抓取脚本也导入该消息。
- `catkin_ws/src/user_msgs/msg/GrabCmd.msg` 不存在，`user_msgs/CMakeLists.txt` 的消息列表也没有该项。
- 影响：干净构建的当前快照不会生成 `user_msgs.msg.GrabCmd`，原任务和抓取节点导入失败；机器上残留的旧生成文件可能暂时掩盖问题。
- 建议：从可信原工程恢复精确的消息字段和常量，登记消息生成后重新构建。不要猜测协议定义。阶段 C 使用自己的接口包，不直接依赖它。

### 11. [P1] 原整车启动引用了未构建的后置 AprilTag 可执行文件

- 位置：[car_localization.launch:27](../mission_ws/src/mission/car_localization/launch/car_localization.launch#L27)。
- 启动配置要求 `apriltag_localization_test_node`，包内只有前置节点的源码和 `add_executable(apriltag_localization_node ...)`，未找到后置目标的源码或构建规则。
- 影响：干净部署时后置定位节点无法启动，依赖后置定位的返程/泊车链路不完整。
- 建议：补齐后置节点，或将现有节点正确参数化为前后两个实例，并同步各实例话题和外参。

## 单左臂框架的接口问题

### 12. [P2] 任务与视觉节点的使能/就绪话题不一致

- 位置：[jzzj_misson.launch:30](../mission_ws/src/mission/jzzj_misson/launch/jzzj_misson.launch#L30)。
- 启动文件把任务端接到 `/jzzj_visual_servo/jzzj_visual_servo_node/enable` 和对应的 `/grasp_ready`。
- 视觉节点实际使用绝对话题 `/jzzj_visual_servo/enable`、`/jzzj_visual_servo/grasp_ready`，没有中间的 `jzzj_visual_servo_node`。
- 影响：启用旧视觉框架后，任务的使能到不了视觉节点，就绪信号也回不到任务。绝对话题不会随节点名自动补齐。
- 建议：统一两端的真实话题名。这项属于 `external_stage_c=false` 的框架接线问题，独立阶段 C 观测节点使用其他话题。

### 13. [P1] AprilTag 运动前守卫放在离开运动阶段时才检查

- 位置：[jzzj_misson_node.py:217](../mission_ws/src/mission/jzzj_misson/scripts/jzzj_misson_node.py#L217)。
- `_advance_stage()` 只在当前阶段为 `MOVE_TO_PICK` 时检查 `tag_ready`。从 `LOCALIZE` 进入 `MOVE_TO_PICK` 的那次调用不会检查，随后已经发布运动命令。
- 触发条件：Tag 未就绪时人工推进，或者定位阶段完成后 Tag 丢失、再人工推进。
- 验证：设置 `require_tag_before_motion=True, tag_ready=False`，调用实际方法，仍进入并发布了 `MOVE_TO_PICK`。
- 建议：检查即将进入的阶段，在发出运动命令前验证定位有效性和时效；也应验证自定义阶段序列的启动入口。

## 本次验证结果与边界

- 现有离线测试：`unittest discover` 共发现 104 项，101 项通过，3 项跳过。跳过项需要 `STAGE_C_FRUIT6_ASSETS` 指向外部模型、样例和历史输出资料；不是模型推理通过。
- 语法检查：51 个项目 Python 文件通过 AST 解析；53 个 ROS 启动/包清单 XML 文件通过解析。这不验证 ROS 消息存在、话题连通或 C++ 链接。
- 额外执行了不连接硬件的真实 Python 方法/函数复现：导航失位停车遗漏、底盘停用未写停车报文、停用视觉节点仍发布速度、Tag 守卫触发顺序、抓取与投放夹爪目标不一致，以及投放颜色配对计算。
- CMake 顶层文件错误已通过实际解析确认。
- 已排除一个表面可疑点：旧驱动 `read_deg()` 中关节 2 的符号转换虽然位于循环内，但在第二关节赋值后实际翻转五次，最终符号正确，不列为缺陷。
- 未执行 ROS/catkin 构建、ARM64 C++/RKNN 运行、相机配准/标定、机械臂实测到位和整车抓取验收。因此本次测试结果不能证明真实阶段 C 已可部署。

建议先修停车和控制权问题，再清理构建/消息/启动阻塞，随后修复原流程的任务、检测、投放与夹爪参数，最后在目标机验证阶段 C 的真实链路。
