# 相机方向、失位停车和水果类别合并修改说明

修改日期：2026-10-18。直接修改 `F:\orangepipro\jzzj_plus` 内源码，项目目录与名称保持不变。按用户确认，使用现有模型，不重新训练。

## 1. 相机已物理调整，默认取消软件翻转（本次更新）

- 根据用户最新安装状态，`hp60c.launch`、`usb_cam.launch` 的 `flip_vertical` 默认值均改为 `false`，USB YAML、驱动参数缺省值和成员初值同步关闭。HP60C 驱动缺省值原本为 `false`，本次保持一致。
- `hp60c_clean.launch` 和 `hp60cn.launch` 也显式设置默认 `false`。这两个入口复用 `ascamera_hp60c` 节点名，显式写参数可防止切换入口时沿用参数服务器中残留的旧 `true`。
- 默认发布原传感器方向的 RGB、深度和 mono8；CameraInfo 的 K/P 不做行反射，正常标定的 `fx/fy` 为正，也不额外改变 `cy`。保持采集时间戳、深度单位与像素通道顺序。
- 相机驱动启动时打印实际方向模式，默认应看到 `native sensor pixels (no software flip)`。已检查引用 HP60C 的阶段 C、模型探针、AprilTag、导航和旧任务入口，它们继承新的相机默认值，没有另外开启翻转。
- AprilTag 正常内参路径直接处理原图、输出当前光学坐标系的位姿。仅对旧软件反射输入（负 `fy`）保留还原分支，并打印模式；物理转动相机本身不应通过负 `fy` 表示。USB 离线标定文件来自目标机，不在本目录，需确认它对应原始传感器像素。
- `flip_vertical:=true` 仍作为显式兼容开关保留，此模式才会同步翻转图像与 K/P；公共图像头和有符号焦距支持继续供兼容路径使用。当前安装应使用 `false`，旧脚本或 ROS 参数服务器中残留的 `true` 需要一并改掉。不能用 `abs(fy)` 单独“修复”不匹配的图像与内参。

### 物理调整对 Tag 定位和抓取的影响

关闭软件翻转与物理转动相机是两件事。Tag 可在当前原图上解码和求位姿，但相机相对车体/机械臂的实际安装方向已经改变，因此旧 `camera_to_arm`、车体到相机外参和静态 TF 需要重新核验，必要时重新标定；不能保证旧外参仍可用。内参对应传感器像素，不能仅因物理转动就手动取负焦距或反射主点。

附件中的外参已包含一次机械臂 Z 轴 180° 偏航修正，它与相机物理旋转不等价。本次没有完整实测矩阵和新的安装位姿，不猜测欧拉角、不重复叠加 180°。重装可能还改变光心位置，需用已知实物点检查臂系落点和桌面法向，再决定是否可沿用或需重新标定。

本次代码/配置涉及 `catkin_ws/src` 内的 `sensor/ascamera/launch/hp60c.launch`、`hp60c_clean.launch`、`hp60cn.launch`、`sensor/ascamera/src/CameraPublisher.cpp`、`sensor/usb_cam/launch/usb_cam.launch`、`sensor/usb_cam/src/UsbCam.cpp`、`sensor/usb_cam/include/usb_cam/UsbCam.h`、`user_params/params/usb_cam_config.yaml` 和 `algorithm/detection/apriltag_localization/src/apriltag_localization_node.cpp`。深度投影算法无需为正常正焦距再加一层方向补偿。

### 交接附件与本目录的版本差异

附件记录的 `/home/orangepi/jzzj-stagec/deploy/` 含平面定位、实测外参、TCP、关节符号和反馈超时配置，本目录没有这些现场文件，也没有附件提到的 `detections_merger.py`；当前本地 observer 仍为 RGB-D 版本，本地检测器直接发布总话题。不能把本目录整包覆盖到现场实际运行副本，否则可能丢失平面定位等现场修复。

本次仅调整相机方向默认值及相关日志/说明，不以本地模板替换现场的外参、TCP、关节符号、平面高度或 I2C 容错参数。相机安装变化后的新标定值须在现场部署配置中更新。

## 2. 定位丢失停车

原缺陷：导航成功后 `stop_flag=1`，而失位分支要求 `stop_flag==0`，导致正常行驶后失位反而不发停车。

- `wp_nav_pid_node.py` 每个控制周期检查前后两路位姿，任一路新鲜即可使用；首选相机失效时使用另一新鲜源。
- 两路均失效立即发布全零 `Twist` 与 `CarCmd.HOLD`，随后每周期继续发送，取消原来的 1 秒等待及一次性停车条件。
- `wp_nav/pose_timeout` 默认 `0.5` 秒，同时检查 ROS 源时间与单调时钟接收时间。零、非有限、过期、超前超过 0.05 秒和不递增时间戳不能刷新有效定位；无效位置/四元数不能启用导航。
- `apriltag_detected=true` 不能单独使旧位姿重新有效；`false` 使对应源无效。失位和恢复时清空 PID 历史、到位计数及缓存速度，恢复新位姿后可以继续原目标。退出回调也发送停车。
- USB 发布线程只发布新采集代次，不给缓存旧图重新打时间戳；AprilTag 同步快照图像与 Header，只对新代次计算位姿，默认 `apriltag_localization/max_frame_age=0.5` 秒，断流超时发布未检测状态。
- AprilTag 位姿、TF 与调试图保留采集时间，避免旧图伪装成新定位。地图定位节点原本就传递该时间戳，无需另改。

相关参数见 [导航说明](../mission_ws/src/mission/wp_nav_pid/README.md)。这覆盖定位数据失效路径；不包含进程被强杀、底盘驱动停摆或 MCU 通信失效时的独立硬件看门狗。

## 3. 水果只区分种类

用户确认的旧模型类别顺序：

| 原始模型 ID | 原语义 | 发布 ID | 发布标签 |
|---|---|---|---|
| 0、3、5 | 坏/熟/生樱桃 | 0 | `cherry`（樱桃） |
| 1、2、4 | 熟/坏/生柠檬 | 1 | `lemon`（柠檬） |

- 模型仍为六类，`class_num=6` 和三个 33 通道输出保持原值；保留模型、包内六类原始中性标签及 SHA256 校验。
- `species_map=[0,1,1,0,1,0]`，`species_labels=[cherry,lemon]`。在原始框解码后、NMS（非极大值抑制）之前完成转换；按水果种类去重后再截取最多 64 框、进入跟踪与发布。置信度保留原模型最高分类概率与目标置信度的乘积，不累加三个子类别概率。
- 修复原 NMS 按排序位置取类别的错误，改为取排序索引对应的真实类别，避免不同水果互相误删。
- 原水果检测配置和 `detector_fruit6.yaml` 均开启合并；未配置合并的其他模型保持原类别输出。外部原始标签文件仍须符合六类模型输入校验，不应改成两行标签。
- 识别叠加图、诊断与 `stage_c_fruit6.yaml` 允许列表改为两个水果种类。旧任务的中文类型字典及任务提示改为种类；好坏、生熟中文描述作为输入别名合并到同一水果 ID，不再产生独立类别。
- 新增种类输出不能按旧的 0～5 好坏/生熟含义解释；自定义订阅者需同步更新。原任务中硬编码大模型返回、检测分流话题等其他审查问题仍见原审查报告。

详细模型约定见 [MODEL_FRUIT6.md](../mission_ws/src/mission/jzzj_grasp/MODEL_FRUIT6.md)。

## 4. 已完成验证（本次重新运行）

- Python 共运行 115 项：112 项通过，3 项因未提供外部模型历史资料而跳过。
- 53 个 launch/package XML 解析通过，相机启动参数/YAML 的不翻转默认值及文档链接检查通过；`git diff --check` 通过。
- 新增 9 项回归直接调用导航类的实际回调/控制周期及 RGB-D 算法，覆盖行驶后丢位、持续停车、重复时间戳、旧位姿配合 true 标志、无效时间戳/四元数、双源切换、恢复和退出停车，以及翻转前后三维位置/未配准深度投影一致性。ROS、GUI 和测试中恒等朝向的 SciPy 转换使用内存替身。
- 主机 g++ 编译并运行真实 `postprocess.cc` 与公共图像头文件测试，验证奇偶行/行填充、字节保持、双翻转还原、K/P 反射、六个类别映射、同种去重、不同水果保留、置信度排序、空检测和错误映射拒绝。编译仅有原文件未使用参数/函数警告。
- 原有离线模型历史解码默认仍输出原始六类，用于核对旧 golden 资料，不等于线上二种类输出；增加了实际叠加回调接受种类标签的测试。

在项目根目录、具有 NumPy 的 Python 环境运行：

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py'
python3 -B -m unittest discover -s mission_ws/src/mission/jzzj_grasp/tests -p 'test_*.py'
g++ -std=c++14 -I catkin_ws/src/user_msgs/include \
  -I catkin_ws/src/algorithm/detection/yolov5_rknn/include/yolov5_rknn \
  tests/requested_changes_test.cpp \
  catkin_ws/src/algorithm/detection/yolov5_rknn/src/postprocess.cc \
  -o /tmp/jzzj_requested_changes_test
/tmp/jzzj_requested_changes_test
```

## 5. 目标机仍需验收

当前环境没有 ROS/ARM64 相机与 RKNN 运行环境，未进行目标机完整 catkin 编译、真实推理及车辆停车距离测量。原工作空间顶层 CMake、缺失旧消息/后置节点等构建前置问题仍在原审查范围内。

1. 在目标机重新构建涉及的 `user_msgs`、相机、AprilTag、YOLO 包并加载两个工作空间；不能只拷贝 YAML 而继续运行旧二进制。
2. 使用本节下方的“不翻转部署步骤”，检查原图 RGB/深度方向和同帧 CameraInfo，正常 `fx/fy` 应为正，AprilTag 日志应为不翻转。用已知点重新核验物理安装后的车体定位和臂系坐标；这次不能用“旧外参保持不变、物体坐标应自动一致”作为验收假设。
3. 在受控测试条件下分别遮挡前相机、遮挡两相机和断开图像源；观察 `car_cmd`/`cmd_vel`。两源失效后应收到 HOLD 和持续全零速度；Bool false 在下一控制周期处理，静默断流在超时后处理。恢复定位后确认正常重启导航且无旧速度跳变。
4. 分别展示樱桃、柠檬及不同状态样本，检查输出只有 `0/cherry` 和 `1/lemon`，同水果重复框被抑制，两种水果可同时保留。

### 不翻转部署步骤（尚未远程执行）

附件明确 `/home/orangepi/jzzj-stagec` 是快照，`/home/orangepi/mission_ws` 才是实际 ROS 运行副本。先在启动终端确认包路径，按本次文件清单同步对应改动；相机 C++ 包实际属于哪个工作空间，以 `rospack find` 结果为准，不假定它们都位于 mission_ws。

```bash
rospack find ascamera
rospack find usb_cam
rospack find apriltag_localization
rospack find jzzj_visual_servo
```

在相应工作空间重新编译修改的 C++ 包，source 其 devel/setup.bash；若沿用交接环境存在 catkin 白名单缓存，构建时清空 `CATKIN_WHITELIST_PACKAGES`。保留现场平面模式源码与 deploy 配置。启动相机时显式使用：

```bash
roslaunch ascamera hp60c.launch flip_vertical:=false
# 若还使用 USB 相机，在另一终端启动：
roslaunch usb_cam usb_cam.launch flip_vertical:=false
```

若通过 `rosrun` 或现场自定义脚本启动，需先把对应节点的 `flip_vertical` 参数设为 `false`，并移除脚本里显式传入的 `true`；仅修改驱动缺省值不会覆盖 ROS 参数服务器中的已有值。节点重命名/命名空间变化时调整下列参数路径：

```bash
rosparam get /ascamera_hp60c/flip_vertical
rosparam get /usb_cam/flip_vertical
rostopic echo -n 1 /ascamera_hp60c/rgb0/camera_info
```

重启相机后，也重启读取内参的 AprilTag、观测与检测节点以清除旧缓存。特别是 AprilTag 只在启动时读取一次 CameraInfo，不能让它继续用旧的负焦距和旧方向分支处理新原图。然后先观察框、Tag、已知点落位，再按现场会话/心跳入口验证抓取。此次离线测试没有验证物理安装后的标定，也没有验证附件现场平面模式或真实抓取。
