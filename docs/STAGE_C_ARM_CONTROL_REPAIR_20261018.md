# C 阶段闭环调用修复记录

本地分支：`fix/stage-c-arm-control`。对应问题与原始证据见 [诊断报告](STAGE_C_ARM_CONTROL_DIAGNOSIS_20261018.md)。本轮仅修改本地工程，没有更新上位机文件、配置或运行节点。

## 已实现

- 逆解改用当前反馈作为初值的 KDL 限位求解，在单次关节步长与 URDF 上下限交集内寻找邻近解；阻尼求解作为后备，同样校验限位、连续性及 TCP 位姿残差。C 运动学服务不再依赖厂商 ARM64 逆解动态库。
- 允许最多 `0.001 rad` 的候选边界数值误差；投影回合法边界后重新做完整 FK 校验。测得的 seed 仍须严格合法，真实超限、分支跳变及不可达目标仍拒绝。错误信息包含关节号或具体残差。
- 统一真实启动入口执行 launch 参数展开预检，拒绝旧命名空间、缺少关节映射、未确认的真实执行配置，以及不足的停止预算。重复控制节点检查覆盖旧版节点名。
- 单次抓取工具先检查实际接口和新鲜关节反馈，再等待当前 session 的有效交权心跳后发目标，避免刚启动心跳就提交目标的竞争。
- 新增项目内 `checked_arm_sdk.py`，使厂商 SDK 吞掉的底层 I/O 异常重新传到轨迹驱动；不修改共享的外部 `Arm_Lib`。
- 顺序回读以采样起点作为时间戳，记录完整读取耗时，慢读取会给出诊断。使能后必须重新取得有效反馈才能开放 Action。
- 到位附加等待由 `2 s` 调整为 `4 s`，C 停止等待由 `4 s` 调整为 `8 s`，覆盖驱动的 `5 s` 停止窗口与反馈延迟；客户端与任务适配器同步等待该停止预算。坐标外参、TCP、关节符号、硬限位和持物算法保持原值。

串行 SDK 一帧六关节实测约 `0.642 s`，这些修改不将它宣称为 10 Hz。批量反馈协议仍需核对现场烧录固件及回包格式，本轮保留现有串行读取，不启用未经现场验证的协议。

## 本地验证

在本机 WSL Ubuntu 中，使用 KDL 1.5.2 和项目实际 URDF 编译并运行 C++ 回归，覆盖现场当前位姿、历史数值越界候选、投影后的位姿残差、硬限位、连续性、不可达及非有限输入，以及分布在多个关节配置的小步可达目标。

结果：24 项 KDL 数学回归全部通过；C 阶段 Python 131 项（128 通过、3 项缺少外部资料跳过），工程与目录 Python 13 项全部通过。6 个 shell 入口通过语法检查，实际 launch XML 与部署 YAML 的命名空间和停止预算交叉检查通过。日志及汇总保存在本地 `.local/control-repair-validation.json` 和同目录的回归日志。

```bash
cmake -S mission_ws/src/mission/jzzj_grasp/tests/kinematics -B .local/kinematics-build
cmake --build .local/kinematics-build -j2
ctest --test-dir .local/kinematics-build -V
python3 -B -m unittest discover -s mission_ws/src/mission/jzzj_grasp/tests -v
python3 -B -m unittest discover -s tests -v
bash -n tools/stage_c/start_control.sh tools/stage_c/grasp_once.sh
```

本地具备 ROS 2 环境，未运行 ROS Noetic 全栈或 ARM64 构建。KDL 数学回归通过不等于完整 ROS 接口集成或实机动作验收。

## 后续现场验证

由此本地版本部署并构建后，先运行 `python3 tools/stage_c/check_control.py` 展开并验证参数。进入控制启动后，再用 `--ready` 检查实际接口和反馈。现场仍须按当前姿态保持、空载单关节小角度、取消保持、可达 Cartesian 小步、C 抓取的顺序验证。

真实启动会使能机械臂，单次抓取工具会发送动作；本轮未执行这些现场步骤。原现场版本仍为部署基线，回退时应整体切回该基线的代码与配置，不混用新旧 SDK 适配或超时值。
