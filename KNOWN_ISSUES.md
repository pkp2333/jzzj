# 已知问题与接手检查

本文记录当前桌面源码快照中已经核实的问题。它们不是一般性优化建议，其中前三项会直接影响完整任务运行。

## 上车前必须处理

### 1. `GrabCmd.msg` 缺失

`mission_manage_node.py`、`grab_test_left.py` 和 `grab_test_right.py` 都导入 `user_msgs/GrabCmd`，但当前 `catkin_ws/src/user_msgs/msg/` 没有该文件，`user_msgs/CMakeLists.txt` 也没有注册它。因此相关 Python 节点会在导入阶段失败。

机器人端曾使用的消息定义需要从实际运行版本同步回来；已知接口常量如下，但应以机器人端源码为唯一依据，避免凭文档重建后产生版本偏差：

```text
uint8 FALSE=0
uint8 FIANL=1
uint8 BUZY=2
uint8 TIMEOUT=3
uint8 START=4
uint8 cmd
```

`FIANL` 和 `BUZY` 是现有代码依赖的历史拼写，不应单方面改名。

### 2. 左右检测话题没有生产者

左右抓取节点订阅：

```text
/ascamera_hp60c/rgb0/yolov5_detections_left
/ascamera_hp60c/rgb0/yolov5_detections_right
```

当前 `yolov5_rknn_det.cc` 只发布：

```text
/ascamera_hp60c/rgb0/yolov5_detections
```

需从机器人运行版本同步左右画面/检测结果拆分逻辑，或明确重新设计抓取节点订阅关系。不要仅通过改话题名绕过，因为左右机械臂可能依赖各自视野内的目标筛选。

### 3. 后置 AprilTag 可执行节点缺失

`car_localization.launch` 启动 `apriltag_localization_test_node`，但当前 `apriltag_localization` 包只构建 `apriltag_localization_node`。后置位姿话题可能因此没有发布者。

需要在机器人上核对：

```bash
ls /home/orangepi/catkin_ws/src/algorithm/detection/apriltag_localization/src/
ls /home/orangepi/catkin_ws/devel/lib/apriltag_localization/
```

### 4. 模型与标签文件不在仓库中

完整任务还需要：

```text
/home/orangepi/Documents/yolov5_weights/yolov5s.rknn
/home/orangepi/Documents/yolov5_weights/labels_rc.txt
/home/orangepi/Documents/deepseek_weights/DeepSeek-R1-Distill-Qwen-1.5B_W8A8_RK3588.rkllm
```

模型应从已验证的机器人环境归档，并记录文件哈希和生成工具版本。不要把来源不明或授权不清晰的模型直接上传公开仓库。

### 5. `catkin_ws` 顶层 CMakeLists 缺失

当前 `catkin_ws/src/CMakeLists.txt` 不存在，直接执行 `catkin_make` 可能失败。首次部署到 ROS 环境后执行：

```bash
cd /home/orangepi/catkin_ws/src
catkin_init_workspace
```

`mission_ws/src/CMakeLists.txt` 已包含在当前快照中。catkin 顶层文件与 ROS 安装路径有关，应在目标环境生成，不要从另一台机器复制绝对路径软链接。

## 已确认的代码问题

### DeepSeek 结果被调试数据覆盖

`mission_manage_node.py` 在校验真实 DeepSeek 返回值后，又把 `ds_output.data` 无条件覆盖为固定任务。当前机器人不会按现场自然语言结果执行。恢复比赛逻辑时应删除这段覆盖，并保留解析失败时的显式报错。

### 左机械臂参数命名空间错误

左臂的 `turn_front`、`turn_middle`、`turn_back` 使用相对参数名读取；右臂使用 `/arm_right/...` 绝对参数名。左臂应与 YAML 的 `/arm_left/...` 层级保持一致，否则调整配置文件时不会生效。

### 后置相机外参未记录

`catkin_ws/src/user_params/params/usb_cam_config.yaml` 只有采集参数和标定文件目录，没有 `translation`、`rotation`。如后置定位实现依赖外参，需从机器人运行版本同步或重新标定。

## 推荐同步方式

不要从历史目录零散猜测和补文件。优先从当前可运行的 Orange Pi 完整同步以下源码目录，再做差异审查：

```text
/home/orangepi/catkin_ws/src/
/home/orangepi/mission_ws/src/
```

同步后依次完成：

1. 记录 Orange Pi 系统、ROS、Python、RKNN、RKLLM 和相机 SDK 版本。
2. 比较并确认所有源码差异，不覆盖本地正在开发的改动。
3. 先编译 `catkin_ws`，再编译 `mission_ws`。
4. 用 `roslaunch --files mission_manage mission.launch` 检查启动链解析。
5. 上电前断开或架空驱动轮，分别验证相机、定位、底盘和机械臂，最后再跑整套任务。
