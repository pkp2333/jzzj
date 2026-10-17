# 阶段 C 开发与 YOLOv5 RKNN 接入改动说明

日期：2026-10-17。范围：固定底盘、单左臂前置视觉闭环抓取，以及新增水果模型资料的识别链路接入。

## 主要改动

- 新增 `jzzj_grasp_interfaces`：抓取 Action、带会话/时间戳的交接状态和目标观测、运动学服务。
- 新增 `jzzj_grasp`：独立抓取编排、模拟/真实后端、小步位置闭环、抬升/运输姿态持物复核、客户端、部署模板与测试。
- 扩展 `jzzj_visual_servo`：YOLO 框内 HSV 筛色、RGB-D 配准和三维定位；不通过整图颜色独立生成目标，不发布底盘速度。
- 左臂新增可选轨迹执行模式：关节范围与单位检查、旧/新驱动互斥、实测到位和取消保持确认；默认旧模式保留。
- 总任务新增默认关闭的 `external_stage_c` 薄适配，处理 Action 反馈、结果和取消；不实现或改写 A/B/D/E 算法。
- `yolov5_rknn` 修正图像/Header 快照、重复/过期帧拒绝、空检测清理、轨迹类别更新；加强标签、模型张量和推理失败检查，打印 Runtime/量化诊断。
- 新增水果模型专用配置、中性标签 fruit-1～fruit-6、外部资源 SHA256 清单和纯 NumPy 历史输出回归。
- 提供无机械臂的相机识别探针与隔离样例图验证入口；启动前校验模型和标签，复用原 C++ 检测器。

## 安全与兼容约束

- 真实运动、标定确认和工作区确认默认关闭；模型资料到位不解除抓取安全门槛。
- 交接状态失效、取消或执行异常时尝试保持；停臂不确定时禁止继续。闭爪后持物/持物未知时禁止新的开爪抓取请求。
- 升降沿标定桌面法向，不假定倾斜安装的左臂坐标 z 为竖直。
- 不修改底盘或 MCU 协议，不纳入右臂、SLAM、避障和通用碰撞规划。
- 模型、ONNX、数据集、Toolkit wheel 和历史 NumPy 输出保持外部管理，未纳入本次提交。
- 补充 Action、JSON 和固定哈希标签的 LF 检出规则，避免 Windows 换行转换破坏标签 SHA256；为 ROS 启动脚本记录 Git 可执行位。
- 标签资料冲突，当前仅用 fruit-1～fruit-6 验证链路，不声称已确认水果种类/成熟度语义。

## 验证结果

- 合并运行 99 项离线测试全部通过：模型接入 36 项、算法/模拟 39 项、Action/任务/轨迹回调替身 24 项。
- 模型测试包含外部 9 个文件的哈希与历史输出解码，后者不是重新运行模型。
- Python 语法、XML、本地文档链接、默认安全开关和 Git 差异格式检查通过。
- 未执行 ROS/catkin 与 ARM64 C++ 编译、真实 RKNN 推理、相机/标定验证、空载/带物抓取和现场连续成功验收。

## Orange Pi 部署入口

外部资料根目录由用户确认为 `/home/orangepi/yolov5-rknn`；若结构与本地一致，模型目录为 `/home/orangepi/yolov5-rknn/yolov5_onnx_2_rknn`。目标机完成源码同步及构建后，先运行无机械臂样例验证：

```bash
roslaunch jzzj_grasp model_probe_fruit6_sample.launch \
  model_dir:=/home/orangepi/yolov5-rknn/yolov5_onnx_2_rknn
```

详细说明见 [阶段 C 接入](../mission_ws/src/mission/jzzj_grasp/README.md) 和 [模型接入](../mission_ws/src/mission/jzzj_grasp/MODEL_FRUIT6.md)。原工程的工作空间顶层 CMake、旧 GrabCmd、后置 AprilTag 等前置问题仍需另行处理，不能直接将本提交视为已验证稳定版。
