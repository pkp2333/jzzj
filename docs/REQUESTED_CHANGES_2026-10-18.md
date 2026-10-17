# 相机、失位停车和水果类别合并修改说明

修改日期：2026-10-18。直接修改 `F:\orangepipro\jzzj_plus` 内源码，项目目录与名称保持不变。按用户确认，使用现有模型，不重新训练。

## 1. 相机上下翻转

- HP60C 的 RGB、深度和可选 mono8 发布图像上下翻转；USB 图像同样处理。使用竖直行反射，相当于 `cv::flip(image, image, 0)`。
- `hp60c.launch`、`usb_cam.launch` 的 `flip_vertical` 参数默认 `true`；可通过 `flip_vertical:=false` 关闭。HP60C 的其他 SDK 型号启动文件仍保持原行为。
- RGB 与深度必须一起翻转，CameraInfo 同步应用 `K'=F*K`、`P'=F*P`，其中 `F=[[1,0,0],[0,-1,H-1],[0,0,1]]`。因此 `fy'=-fy`、`cy'=H-1-cy`，物理光学坐标、外参和深度单位保持原定义。当前验证针对全幅图像，不包含 ROI/binning 裁剪组合。
- 公共行翻转与标定实现位于 `catkin_ws/src/user_msgs/include/user_msgs/image_orientation.h`；相机包增加对应头文件依赖。原始模型文件与图像像素通道顺序不变。
- 阶段 C 的深度配准和三维提取允许有符号焦距，仍拒绝零焦距和非有限内参。AprilTag 通过负 `fy` 识别输入已翻转，在解码/PnP 前恢复传感器图像与内参，避免镜像破坏 Tag 手性，输出位姿仍对应原物理坐标系。

主要源码：`sensor/ascamera/src/CameraPublisher.cpp`、`sensor/usb_cam/src/UsbCam.cpp`、`algorithm/detection/apriltag_localization/src/apriltag_localization_node.cpp`（均在 `catkin_ws/src` 内）及 `mission_ws/src/mission/jzzj_visual_servo/src/jzzj_visual_servo/perception.py`。

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

## 4. 已完成验证

- Python 共运行 115 项：112 项通过，3 项因未提供外部模型历史资料而跳过。
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
2. 检查 RGB/深度方向、同帧 CameraInfo 和负 `fy`，确认 AprilTag 仍能正常识别，已标定物体坐标在翻转前后保持一致。
3. 在受控测试条件下分别遮挡前相机、遮挡两相机和断开图像源；观察 `car_cmd`/`cmd_vel`。两源失效后应收到 HOLD 和持续全零速度；Bool false 在下一控制周期处理，静默断流在超时后处理。恢复定位后确认正常重启导航且无旧速度跳变。
4. 分别展示樱桃、柠檬及不同状态样本，检查输出只有 `0/cherry` 和 `1/lemon`，同水果重复框被抑制，两种水果可同时保留。
