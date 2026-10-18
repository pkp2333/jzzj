# C 阶段统一目录与启动

工程唯一源码目录：`/home/orangepi/jzzj`，分支 `refactor/stage-c-layout`，代码基线为现场 plus。
原项目中其他任务独有的源码、底盘工程和资料已保留；未把所有旧工作空间整包删除。

## 目录

| 位置 | 内容 |
|---|---|
| `catkin_ws/src/actuator/arm` | 机械臂控制、轨迹驱动、互斥和运动学依赖 |
| `mission_ws/src/mission/jzzj_grasp*` | C 抓取编排、接口和离线测试 |
| `mission_ws/src/mission/jzzj_visual_servo` | 深度/平面定位与目标观测 |
| `mission_ws/src/mission/jzzj_misson` | 新任务框架及 C 适配器 |
| `deploy` | 现场参数、三组标定点、TCP 历史记录；数值沿用，未重新标定 |
| `tools/stage_c` | 统一构建、观察、控制、单次测试及标定工具 |

`deploy/observe.yaml` 原来是未标定占位配置，仅留在回退压缩包中。
`tcp_calib.json` 与 `kinematics.yaml` 是不同历史记录；实际运动学读取后者，不自动合并。
原始脚本和外部目录的完整备份及 Git bundle 在 `.local/migration-stage-c-20261018/`；该目录不提交 Git。
迁移清单与原文件校验值见 `STAGE_C_MIGRATION_MANIFEST.json`。

## 构建与检查

```bash
cd /home/orangepi/jzzj
bash tools/stage_c/build.sh
source tools/stage_c/env.sh
rospack find arm
rospack find jzzj_grasp
python3 -B -m unittest discover -s mission_ws/src/mission/jzzj_grasp/tests -v
```

包路径必须位于 `/home/orangepi/jzzj/`。无需再 source `jzzj-stagec` 或 `stagec_verify_ws`。
构建脚本只选择 C 所需包，不改变其他任务的实现；后续要构建全部任务时，需要显式清空 catkin 白名单。
RKNN 模型和厂商 Arm_Lib 仍是外部运行依赖，不属于本次重复源码清理范围。

## 只观察（不启动机械臂）

先在独立终端运行 `roscore`，再分别运行：

```bash
bash /home/orangepi/jzzj/tools/stage_c/start_camera.sh
bash /home/orangepi/jzzj/tools/stage_c/start_observe.sh yellow
```

重新构建的检测器直接发布总话题，观察入口默认不启动左右合并器。仅旧分流检测器兼容时显式传入 `use_split_detections:=true`。

停止时在对应终端按 Ctrl+C。新入口不使用 `pkill -9`，不杀其他任务或 ROS master。

## 单次 C 抓取测试

退出观察入口及旧的 arm_test/arm_control 测试；保留独立 roscore 和已启动的相机。
控制入口会检查重复节点，不会强杀已有任务。
控制启动前会展开 launch 参数，检查轨迹节点的私有命名空间、真实执行开关和停止预算。单次测试会检查实际接口、新鲜反馈及当前会话心跳。闭环调用修复及本地验证见 [修复记录](STAGE_C_ARM_CONTROL_REPAIR_20261018.md)。

```bash
bash /home/orangepi/jzzj/tools/stage_c/start_control.sh real
# 另一终端；只有操作员确认标定、舵机反馈和停车交权后才执行：
bash /home/orangepi/jzzj/tools/stage_c/grasp_once.sh grasp-unique-session yellow
```

默认 `start_control.sh` 运行 sim。real 启动本身会使能左臂并保持姿态。
`heartbeat.py` 是现场调试交权心跳，不是底盘传感器提供的停车证明。
当前现场原版仍含消失式持物确认等已审查问题，本次目录迁移没有合入此前暂停的算法整改。
舵机反馈仍需另行确认；迁移成功、构建成功均不等于实机抓取成功。

## 标定工具与回退

激活 `env.sh` 后使用 `capture_point.py X Y`；数据保存至本项目 `deploy/calib_points.json`。
优先使用 `action_jog.py` 通过轨迹 Action 调整关节。
`jog_joint.py` 保留为离线维修工具，会直接访问 I2C，只能在控制驱动退出后使用。

回退时先退出本次测试节点，再解压备份恢复外部路径；Git 基线标签是
`baseline/pre-stage-c-layout-20261018`。恢复前保留当前新增改动，避免覆盖后续工作。
