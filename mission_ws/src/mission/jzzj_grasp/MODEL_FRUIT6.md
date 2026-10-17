# 外部水果模型：识别链路验证接入

初次核对日期：2026-10-17；种类合并更新：2026-10-18。外部资料：`yolov5-rknn/yolov5_onnx_2_rknn`。本接入**不复制权重/数据集到源码仓库，不重新转换模型，不安装 Toolkit，不启动抓取或机械臂**。

## 当前业务输出：只区分水果种类

用户已于 2026-10-18 确认该六类模型沿用旧 ROS 顺序。保留六类权重与张量结构，将原始 ID 在检测后处理中合并：

| 原模型 ID | 旧语义 | 发布 ID / 标签 |
|---|---|---|
| 0、3、5 | 坏、熟、生樱桃 | `0 / cherry`（樱桃） |
| 1、2、4 | 熟、坏、生柠檬 | `1 / lemon`（柠檬） |

配置为 `species_map: [0,1,1,0,1,0]` 和 `species_labels: [cherry,lemon]`。映射发生在按类 NMS 与 64 框截断之前，跟踪、叠加图和 RGB-D 观测使用合并后的 ID/标签，不再输出好坏、生熟业务类别。置信度沿用原模型获选子类别分数，不累加三个子类别的概率。

`class_num` 必须继续为 **6**，不能改为 2；模型输出仍为三个 33 通道张量。包内 `fruit6_labels.txt` 和 manifest 继续描述原始六类 ABI（模型接口约定）及历史资料，用原字节 SHA256 校验，不把该文件改成二分类标签。当前业务映射的依据是用户对类别顺序的明确确认，不是从冲突资料中推断。更换权重需重新核对顺序与模型哈希。

## 1. 使用哪份资料

| 项目 | 本次采用内容 |
|---|---|
| RKNN | 外部 `yolov5s.rknn`，9,642,046 字节 |
| SHA256 | `e3e96bf0e5aa62937319a62859cd2b8038e194c44ad6b731a827edac74192192` |
| 平台/版本静态证据 | 内嵌 target 为 rk3588，Toolkit 1.5.2+b642f30c；仓库 Runtime 为 1.5.2；**未证明目标机驱动及运行时通过** |
| 模型输入 | `[1,3,640,640]`；应用提供 UINT8/NHWC RGB |
| 模型输出 | 三头 `[1,33,80,80]`、`[1,33,40,40]`、`[1,33,20,20]`；静态资料指向 NCHW INT8 输出；真实量化参数由 Runtime 查询 |
| 原始类别 | 6 类，ONNX 元数据原名 `fruit-1`～`fruit-6`；当前业务按用户确认映射成两个水果种类 |
| 预处理 | BGR→RGB，直接拉伸到 640×640；模型内 mean=0、std=255，应用**不再次除 255** |
| 解码 | 已 sigmoid 的 YOLOv5 三头，18 个 anchors，stride 8/16/32，confidence=0.25、NMS=0.45 |
| 坐标恢复 | 检测框从 640×640 反缩放回源图，X/Y 各自恢复；没有 letterbox padding |

模型、ONNX、anchors、原始冲突标签、样例图、历史输出和原始脚本的完整 SHA256/大小见[资源清单](resources/fruit6_manifest.json)。原始脚本 `test.py` 会构建并覆盖模型，本接入不执行它。也不反序列化 `.pt`、导入训练工程或自动训练。

### 为什么不能复制 classes.txt

外部 `classes.txt` 有 14 类，但模型通道数对应 6 类；初次接入时，示例脚本、数据集类别和旧 ROS 水果映射不能共同证明同一个语义顺序。因此原始 ABI 校验继续使用包内[六类中性标签](resources/fruit6_labels.txt)：首行 `6`，随后 fruit-1～fruit-6。启动前校验完整标签字节哈希和顺序，不自动截断 14 类文件。业务层的二种类发布标签由 `species_labels` 单独配置。

当前支持“樱桃”“柠檬”种类任务，不提供成熟度或好坏筛选。旧中文描述作为输入别名合并，例如“坏樱桃”和“熟樱桃”均指向樱桃，而不再保证挑选所描述的状态。更换模型时重新更新模型接口约定与校验，不能仅修改业务类别数来适配新权重。

## 2. 文件布局和配置

- [detector_fruit6.yaml](params/detector_fruit6.yaml)：平铺 ROS 参数，加载到 `/yolov5_rknn_rc`；不内嵌模型绝对路径。
- [stage_c_fruit6.yaml](params/stage_c_fruit6.yaml)：允许两个水果种类标签进入框内 HSV；其余标定与硬件使能默认保持 false。
- [model_probe_fruit6.launch](launch/model_probe_fruit6.launch)：真实相机话题的识别链路验证入口，默认复用已有相机，RGB-D 观测默认关闭。
- [model_probe_fruit6_sample.launch](launch/model_probe_fruit6_sample.launch)：独立样例图验证入口，样例输入和检测结果与真实相机使用不同话题。

`model_dir` 指向包含 `yolov5s.rknn` 的**绝对目录**。样例测试及离线回归还需要同目录的 `0.jpg`/`.npy` 等源文件。标签与 manifest 从 ROS 包加载，支持 catkin devel/install 两种空间。外部权重不被包安装规则收集，部署时手工归档到目标机并保留哈希。

探针启动先运行 `run_verified_fruit6.py`：验证配置、模型和标签，再原进程 exec 已有 `yolov5_rknn_stream`。它不是第二套推理实现；哈希失败时不会启动 C++ 检测器。C++ 继续打印 SDK/驱动、实际张量名称/形状/布局/类型/量化参数、类别标签和 anchors，继续严格校验不兼容模型。

不要同时运行原 YOLO launch 和本探针，以免相同话题存在两个检测发布者；不要在验证过程中覆盖或修改外部权重。此哈希是部署完整性校验，不是恶意并发文件替换的安全防护。

## 3. 本地只读验证：不需要 ROS、OpenCV 或 RKNN

以下从源码运行的 PowerShell 示例使用你可用的 Python3（需要 NumPy），变量仅作用于当前终端。使用应用附带 Python 时替换 `$python` 为该解释器完整路径，不需要安装目录内的 Linux wheel。

```powershell
$python = 'python'
$pkg = 'D:\Desktop\小车gpt\jzzj\mission_ws\src\mission\jzzj_grasp'
$assets = 'D:\Desktop\小车gpt\yolov5-rknn\yolov5_onnx_2_rknn'
$env:PYTHONPATH = "$pkg\src;$env:PYTHONPATH"
& $python -B "$pkg\scripts\check_fruit6.py" --model-dir "$assets" `
  --manifest "$pkg\resources\fruit6_manifest.json" --labels "$pkg\resources\fruit6_labels.txt" --replay
```

`--replay` 校验所有 9 个外部参考文件，然后只解码历史保存的输出，不读取 RKNN 来重新推理，也不写图片/模型。

初次接入的历史输出回归记录：6 个框，原始类别 ID 为 `[0,1,2,3,3,4]`，不是“六类各一个”；源图尺寸 640×480，Y 坐标按 0.75 恢复。离线历史解码保留六类默认行为，输出 JSON 明确标记 `PREVIOUSLY_SAVED_OUTPUTS_ONLY` 和 `NOT_VALIDATED_ON_DEVICE`。这不是当前线上种类合并后的输出数量或 ID，也不是本轮重新运行推理的结果。本轮未提供外部资料，相应 3 项历史资源测试跳过。

## 4. 目标机步骤一：样例图真实推理（本轮未执行）

前置：正确初始化并构建两个工作空间，source 实际 ROS 和工作空间；安装包级依赖。当前源码依然有原有 catkin 顶层文件问题，见[阶段 C 主文档](README.md)。不需要安装 RKNN Toolkit 来做已转换模型的 Runtime 推理。

```bash
# 示例 Linux 部署目录；先自行将外部资料按原名部署并检查哈希。
roslaunch jzzj_grasp model_probe_fruit6_sample.launch \
  model_dir:=/home/orangepi/Documents/yolov5_fruit6
```

这个入口只启动既有检测器、RGB 框图探针和一个参考图片发布工具：

- 样例 RGB：`/stage_c/model_probe/sample_rgb`
- 样例检测：`/stage_c/model_probe/sample_detections`
- 图像 frame：`fruit6_sample_only`
- 叠加图：`/stage_c/model_probe/image`
- 诊断：`/stage_c/model_probe/status`（DiagnosticArray）

图片工具等检测器和探针都订阅后发送一次带当前时间戳的参考帧，等待同一帧的检测回报；它不会持续伪造新的采集时间。发送一次后输出真实回报数量/类别/分数；无回报超时失败。没有深度、相机标定、交接心跳、抓取服务或任何执行器。样例图不能作为阶段 C 的真实三维观测。

判断标准：节点没有 ABI/哈希错误，实际回报只使用 `0/cherry` 和 `1/lemon`，框位于正确的**源图 640×480**区域。样例文件按文件原方向输入；实时相机默认在发布端上下翻转，检测框应对应该发布图像。不同设备或种类去重后的数量可能与原始六类历史输出有差异，应记录而非修改 golden 输出强行通过。单次样例结果之后显示 STALE 是预期行为，说明没有把旧帧重复当作新检测。

本轮修改与测试清单见[修改说明](../../../../docs/REQUESTED_CHANGES_2026-10-18.md)。主机 C++ 回归已覆盖实际后处理的六类映射、同种去重及跨种类框保留；ROS/ARM64 编译、RKNN 实时识别和相机方向仍需目标机验收。

## 5. 目标机步骤二：接前置相机（本轮未执行）

```bash
# 复用已启动的前置相机（默认 start_camera=false）
roslaunch jzzj_grasp model_probe_fruit6.launch \
  model_dir:=/home/orangepi/Documents/yolov5_fruit6

# 需要本入口启动 hp60c 时才显式开启：
roslaunch jzzj_grasp model_probe_fruit6.launch \
  model_dir:=/home/orangepi/Documents/yolov5_fruit6 start_camera:=true
```

默认输入 `/ascamera_hp60c/rgb0/image`，检测输出仍为 `/ascamera_hp60c/rgb0/yolov5_detections`。允许通过 rgb_topic/detections_topic 覆盖，检测器 remap、RGB 探针和可选 RGB-D 观测会保持一致；不需要设置 CHIP_TYPE。所选模型路径始终从 model_dir 获取。

先只看 RGB 叠加和诊断，确认类别/框/时间戳，停止相机时应出现 STALE，重复源帧不能维持 fresh。空检测是新帧的有效“0 个框”，会清掉旧框。类号/标签不匹配、非法框或非有限值输出诊断错误。

### 可选 RGB-D 适配检查

完成真实彩深配准、内参、外参、深度尺度和框内颜色参数后，复制配置到部署文件并正确填写，再执行：

```bash
roslaunch jzzj_grasp model_probe_fruit6.launch \
  model_dir:=/home/orangepi/Documents/yolov5_fruit6 \
  enable_rgbd_observer:=true observation_config:=/path/calibrated_stage_c_fruit6.yaml target_color:=red
```

这只启用观测和颜色选择，不启动抓取。默认专用配置 `calibration_confirmed=false`，会显式拒绝生成有效三维观测；**模型到位不能解除标定门槛**。RGB-only 叠加不依赖深度，所以可先单独验证检测接入。

## 6. 自动化检查与本轮结果

```powershell
$env:STAGE_C_FRUIT6_ASSETS = 'D:\Desktop\小车gpt\yolov5-rknn\yolov5_onnx_2_rknn'
& $python -B "$pkg\tests\test_fruit6.py"
& $python -B "$pkg\tests\test_stage_c.py"
& $python -B "$pkg\tests\test_ros_contracts.py"
```

没有外部资料时，test_fruit6 的 3 项真实文件/历史张量测试明确 skip，其余合成/配置测试仍可运行，不将缺文件误报为通过。外部目录由环境变量指定，不硬编码进测试或运行配置。CMake 已注册离线测试。

本轮完成：新增模型接入测试 **36 项全部通过**（包含 3 项真实外部文件/历史张量检查），既有阶段 C **63 项全部通过**，合计 **99 项**。覆盖资源/标签 SHA256、配置/话题覆盖、解码/NMS/原图坐标恢复、错误模型/标签/输出、重复/空帧、RGB 叠加回调以及既有抓取/取消逻辑。回调使用内存替身，历史张量测试不是新推理。

尚未执行：目标机 catkin/C++ 构建、真实 RKNN 推理、相机输入、RGB-D 标定验证、抓取、连续五次带物验收。没有修改 A/B/D/E、底盘、MCU 或真实运动安全开关。后续更换实际抓取物体需要对应模型，当前水果模型的接通不代表泛化到其他物体。
