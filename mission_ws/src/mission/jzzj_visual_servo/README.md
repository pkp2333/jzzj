# jzzj_visual_servo

阶段 C 的观测与位置闭环算法包：**YOLOv5 RKNN 检测框 → 框内 HSV 颜色筛选 → RGB-D 三维观测 → 左臂坐标系**。

- `scripts/stage_c_observer.py`：订阅 ObjDets/RGB/深度/CameraInfo，按源帧同步；只处理 YOLO 框，不根据整图颜色独立生成目标。
- `src/jzzj_visual_servo/perception.py`：纯 NumPy 颜色/深度估计和 Z-buffer 配准，离线可测。
- `src/jzzj_visual_servo/servo.py`：有界位置小步原语；由 jzzj_grasp 调用，不输出底盘速度。
- `/stage_c/observations` 为候选帧；空检测移除旧候选，diagnostic 解释传感器/标定失败。
- 实际锁定目标由抓取执行端发布 `/stage_c/observation`，不是依赖 tracker ID 自动换目标。

完整启动、标定、接口与验证说明见[阶段 C 接入文档](../jzzj_grasp/README.md)。

## 2026-10-18 相机方向与种类更新

用户已物理调整相机，HP60C 和 USB 相机启动入口、USB YAML 和驱动缺省值统一为 `flip_vertical:=false`。默认发布原传感器方向的图像与 K/P，正常 `fx/fy` 为正，不额外反射主点。启动日志应显示 `native sensor pixels (no software flip)`；现场旧脚本/ROS 参数中的 `true` 也须取消。

AprilTag 对正常正焦距输入直接解码/PnP，不翻图。显式启用旧软件反射模式时才按负 `fy` 恢复 Tag 手性，有符号焦距算法继续兼容。物理调整改变了相机相对车体/臂的安装关系，必须重新核验 `camera_to_arm` 和相关 TF，不能把旧外参自动视为有效。更换方向后也要重启仅在启动时读取内参的 Tag 节点。

交接附件中的现场版本已采用平面定位，含不入库的 deploy 实测配置；本目录仍为 RGB-D 实现。本次相机方向更新不包含现场平面版本的回迁，部署时按文件同步相机改动，保留现场观测与标定代码。

`stage_c_fruit6.yaml` 现仅允许 `0/cherry`（樱桃）和 `1/lemon`（柠檬），沿用框内 HSV 筛色。模型的六个原始类别在检测器中合并，模型权重与六类张量结构保持原值。详见[本轮修改说明](../../../../docs/REQUESTED_CHANGES_2026-10-18.md)。

## 遗留框架

`jzzj_visual_servo_node.py` 和 `jzzj_visual_servo.launch` 保留兼容旧调试。接口是节点私有话题，实际全名例如 `/jzzj_visual_servo/jzzj_visual_servo_node/enable`，而非省略节点名的旧写法。该旧框架的 Vector3→Twist 不属于新的阶段 C 抓取实现，不要接到底盘或同时启动。新 C 入口为 `roslaunch jzzj_grasp stage_c_sim.launch` / `stage_c.launch`。
