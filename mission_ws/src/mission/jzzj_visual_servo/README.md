# jzzj_visual_servo

阶段 C 的观测与位置闭环算法包：**YOLOv5 RKNN 检测框 → 框内 HSV 颜色筛选 → RGB-D 三维观测 → 左臂坐标系**。

- `scripts/stage_c_observer.py`：订阅 ObjDets/RGB/深度/CameraInfo，按源帧同步；只处理 YOLO 框，不根据整图颜色独立生成目标。
- `src/jzzj_visual_servo/perception.py`：纯 NumPy 颜色/深度估计和 Z-buffer 配准，离线可测。
- `src/jzzj_visual_servo/servo.py`：有界位置小步原语；由 jzzj_grasp 调用，不输出底盘速度。
- `/stage_c/observations` 为候选帧；空检测移除旧候选，diagnostic 解释传感器/标定失败。
- 实际锁定目标由抓取执行端发布 `/stage_c/observation`，不是依赖 tracker ID 自动换目标。

完整启动、标定、接口与验证说明见[阶段 C 接入文档](../jzzj_grasp/README.md)。

## 遗留框架

`jzzj_visual_servo_node.py` 和 `jzzj_visual_servo.launch` 保留兼容旧调试。接口是节点私有话题，实际全名例如 `/jzzj_visual_servo/jzzj_visual_servo_node/enable`，而非省略节点名的旧写法。该旧框架的 Vector3→Twist 不属于新的阶段 C 抓取实现，不要接到底盘或同时启动。新 C 入口为 `roslaunch jzzj_grasp stage_c_sim.launch` / `stage_c.launch`。
