# jzzj_grasp_interfaces

无业务包依赖的阶段 C 协议定义。

- `VisualGrasp.action`：会话、颜色、profile、超时 → 反馈/取消/可信结果。
- `StageCContext.msg`：时间戳、会话、底盘锁定、左臂交权及停止请求；2 Hz，超过 1 秒失效。
- `TargetSelection.msg`：当前识别 session/color，结束后 enabled=false。
- `TargetObservation.msg` / `TargetObservations.msg`：带采样时间及坐标系的目标/完整候选帧，不使用旧 ObjDet.pose 作为三维真值。
- `ComputeArmMotion.srv`：六关节弧度 seed、TCP 米制目标，返回有效性、解、TCP 正解及失败原因。

字段及接入责任见[完整说明](../jzzj_grasp/README.md)。
