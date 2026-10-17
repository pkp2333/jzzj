# 基于大模型的具身智能抓取分拣机器人

本仓库是比赛交付代码的精简源码版，包含 Orange Pi 上运行的 ROS 1 上位机工程，以及 STM32/RT-Thread 底盘与机械臂控制代码。项目完成自然语言任务解析、目标检测、AprilTag 定位、麦轮底盘导航、双机械臂抓取、分类投放和泊车。

> 当前快照仍有若干影响完整复现的缺项。首次编译或上车前请先阅读 [KNOWN_ISSUES.md](KNOWN_ISSUES.md)，尤其是缺失的 `GrabCmd.msg`。

## 任务范围说明

- [任务目标与历史进度](TASK_PROGRESS.md)：新增单左臂任务的目标与历史记录；当前实现以源码核对结果为准。

新增 `jzzj_misson`（保留现有拼写）与 `jzzj_visual_servo` 分别提供单左臂任务编排框架和视觉误差比例控制框架；不替代下述原双臂流程，也尚未接通真实定位、识别与执行器。其启动入口、私有话题及参数连接仍需核对，请勿把框架启动成功当成整车任务已实现。

## 阶段 C：前置视觉闭环抓取

已新增 YOLOv5 RKNN 检测框内颜色筛选、RGB-D 观测、固定底盘左臂抓取 Action、模拟后端和总任务薄适配。真实运动默认关闭，模型/标签和机器人标定仍需现场提供，尚未进行 ROS/ARM64 构建或实机抓取验收。

- [阶段 C 接入与验证说明](mission_ws/src/mission/jzzj_grasp/README.md)
- [阶段 C 架构变化索引](docs/STAGE_C_IMPLEMENTATION.md)

## 目录结构

```text
.
├── catkin_ws/                  # 通用 ROS 功能包
│   └── src/
│       ├── actuator/           # chassis、arm
│       ├── algorithm/          # YOLOv5 RKNN、AprilTag、DeepSeek RKLLM
│       ├── sensor/             # 前置深度相机、后置 USB 相机
│       ├── user_msgs/          # 自定义 ROS 消息
│       ├── user_params/        # 整车参数与标定
│       └── vision_opencv/      # cv_bridge
├── mission_ws/                 # 比赛任务工作空间
│   └── src/mission/
│       ├── car_localization/   # AprilTag 位姿融合
│       ├── wp_nav_pid/         # 航点 PID 导航
│       ├── mission_manage/     # 原双臂任务、抓取、投放与测试工具
│       ├── jzzj_misson/        # 新单左臂任务状态框架
│       └── jzzj_visual_servo/  # 新视觉闭环控制框架
├── 底盘代码/
│   ├── chassis/                # STM32F407 + RT-Thread 底盘固件
│   └── arm_iic_control/        # STM32F103 舵机/I2C 控制工程
├── 上位机代码调试说明v1.3.pdf
└── arm_test_node_fixed.py      # 独立机械臂调试脚本
```

两个 catkin 工作空间的依赖方向是：`mission_ws -> catkin_ws`。必须先构建并加载 `catkin_ws`，再构建 `mission_ws`。

## 主要 ROS 包

| 包 | 作用 |
|---|---|
| `actuator/chassis` | 串口下发底盘运动与机构控制命令 |
| `actuator/arm` | 左右机械臂控制、URDF 与逆运动学 |
| `sensor/ascamera` | 前置 hp60c 相机 |
| `sensor/usb_cam` | 后置 USB 相机 |
| `algorithm/detection/yolov5_rknn` | RK3588 NPU 目标检测与跟踪 |
| `algorithm/detection/apriltag_localization` | AprilTag 位姿估计 |
| `algorithm/llm/deepseek` | 本地 RKLLM 自然语言任务解析 |
| `car_localization` | 前后相机定位信息融合 |
| `wp_nav_pid` | 航点导航控制 |
| `mission_manage` | 原双臂比赛任务主流程 |
| `jzzj_misson` | 新单左臂任务阶段框架，执行接口待接入 |
| `jzzj_visual_servo` | 新视觉误差比例控制框架，识别与执行适配待接入 |

## 运行环境

代码面向 Orange Pi/RK3588 的 Linux 环境，而不是 Windows 开发机。源码中当前约定的设备路径如下：

| 内容 | 默认路径 |
|---|---|
| 通用工作空间 | `/home/orangepi/catkin_ws` |
| 任务工作空间 | `/home/orangepi/mission_ws` |
| Python 解释器 | `/home/orangepi/miniconda3/envs/rknn/bin/python` |
| YOLOv5 模型 | `/home/orangepi/Documents/yolov5_weights/yolov5s.rknn` |
| YOLOv5 标签 | `/home/orangepi/Documents/yolov5_weights/labels_rc.txt` |
| DeepSeek 模型 | `/home/orangepi/Documents/deepseek_weights/DeepSeek-R1-Distill-Qwen-1.5B_W8A8_RK3588.rkllm` |
| 相机标定文件 | `/home/orangepi/.ros/camera_info/` |

仓库不包含 RKNN/RKLLM 模型权重。部署时应单独准备模型和标签文件，或修改对应 YAML 中的路径：

- `catkin_ws/src/algorithm/detection/yolov5_rknn/params/yolov5_rc_rk3588s_ascamera.yaml`
- `catkin_ws/src/algorithm/llm/deepseek/params/deepseek_config.yaml`

项目依赖 ROS 1/catkin、OpenCV、`cv_bridge`、AprilTag、RKNN Runtime、RKLLM Runtime、相机 SDK 以及串口和常用 ROS 消息包。部分 ARM64 运行库已随源码保留，但具体 ROS 发行版和系统镜像尚未在本地资料中确认，建议以机器人当前系统为准记录版本。

## 构建

> 补充核对：当前 `mission_ws` 顶层 CMake 文件是普通路径文本，新包安装规则也有待验证问题。请先核对工作空间初始化、包安装规则及构建依赖，不要仅按“文件存在”判断工作空间已初始化；下列命令应在相关阻塞项处理后使用。

在 Orange Pi 上先解决 [KNOWN_ISSUES.md](KNOWN_ISSUES.md) 中的阻塞项。当前 `catkin_ws/src` 缺少 catkin 顶层 `CMakeLists.txt`，首次构建时先初始化，再按依赖顺序编译：

```bash
cd /home/orangepi/catkin_ws/src
catkin_init_workspace

cd /home/orangepi/catkin_ws
catkin_make
source devel/setup.bash

cd /home/orangepi/mission_ws
catkin_make
source devel/setup.bash
```

建议把两条 `source` 命令加入机器人用户的 shell 启动脚本，但要保持 `catkin_ws` 在前、`mission_ws` 在后。

## 启动

以下为原双臂比赛任务的整车启动入口，会启动真实驱动；新任务应单独进行隔离验证。当前快照仍需先处理已知阻塞项：

```bash
roslaunch mission_manage mission.launch
```

它会启动 DeepSeek、前后相机、YOLOv5、AprilTag 定位、航点导航、底盘、左右机械臂、左右抓取节点及任务主状态机。

常用调试入口：

| 命令 | 用途 |
|---|---|
| `roslaunch mission_manage mission_test.launch` | 任务解析与管理测试 |
| `roslaunch mission_manage grab_test.launch` | 抓取测试 |
| `roslaunch mission_manage basket_seq_test.launch` | 放置框颜色顺序识别 |
| `roslaunch mission_manage basket_hsv_threshold.launch` | HSV 阈值调节 |
| `roslaunch wp_nav_pid wp_nav_pid_test.launch` | 航点导航测试 |

## 修改入口

| 目标 | 文件 |
|---|---|
| 主流程、状态机 | `mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py` |
| 任务点位、超时和抓取参数 | `mission_ws/src/mission/mission_manage/params/mission_params.yaml` |
| 导航 PID | `mission_ws/src/mission/wp_nav_pid/params/wp_nav_pid.yaml` |
| AprilTag 场地坐标 | `mission_ws/src/mission/car_localization/params/tag_pose.yaml` |
| 机械臂姿态与动作 | `mission_ws/src/mission/mission_manage/scripts/arm_motions.py` |
| 机械臂安装和夹爪参数 | `catkin_ws/src/user_params/params/arm_config.yaml` |
| 底盘尺寸与限速 | `catkin_ws/src/user_params/params/chassis_config.yaml` |
| 前相机外参 | `catkin_ws/src/user_params/params/camera_ep_config.yaml` |

## Git 与发布说明

- `.gitignore` 已排除 catkin 构建目录、Python 缓存、IDE 文件、MCU 构建产物、压缩包和模型权重。
- `.gitattributes` 将 ROS/Python/C++ 等源码统一为 LF，避免从 Windows 检出后在 Linux 上出现 shebang 或编译脚本换行问题。
- `*.so` 和部分厂商 `*.lib` 是现有相机、RKNN、RKLLM 和机械臂代码的运行/链接依赖，未被统一忽略。
- 模型文件较大且未包含；若以后纳入仓库，建议使用 Git LFS，并先确认分发许可。
- 当前仓库没有顶层许可证，若要建立公开仓库，应先确认比赛代码、第三方源码、厂商 SDK 和二进制库的再分发权限，再选择许可证。
- `D:\while\Stm32_project\xiangchengpi` 是历史完整工程与资料来源；本目录是后续开发和 Git 管理的主版本。不要把两套目录直接合并覆盖。

首次在 Windows 上建立 Git 仓库时，应在 `git add .` 后给 ROS 可执行脚本补上 Git 可执行位：

```bash
git add --chmod=+x -- ':(glob)**/scripts/*.py' ':(glob)**/scripts/*.sh'
git ls-files --stage | grep '^100755'
```

否则代码检出到 Orange Pi 后，`roslaunch` 可能报告节点文件不可执行。上面的命令只修改 Git 索引中的文件模式，不会在 Windows 上直接执行这些脚本。

## 提交前检查

```bash
git status --short
git diff --check
```

确认模型、数据集、ROS bag、编译产物、缓存和本机配置均未进入暂存区；再检查代码中是否新增口令、API Key、Wi-Fi 信息或固定设备地址。
