# 固定抓取的一键底盘微调

用户最终参数：单步目标位移不超过 50 mm；水果在左臂参考点三维距离 20 mm 内、稳定至少 1 秒后抓取；总行程、步数和整体对齐时间默认不设上限。每次停车重新识别，不连续盲走。固定抓取角度及到位后等待 3 秒不变。

调度层发布 `/car_target`（geometry_msgs/PoseStamped，map）和 `/car_cmd`（user_msgs/CarCmd，NAVIGATION/HOLD）。导航层 `/stage_c_fixed_navigation` 处理这些接口并向原底盘驱动发布 `/cmd_vel`，到位发布 ARRIVE。没有修改 A/B/D 的旧导航流程。该入口不同时运行旧 wp_nav_pid、键盘或其他调度器。

专用导航平移限速 0.02 m/s，不旋转；新目标最多 50 mm；每步 10 秒到位超时，定位/指令心跳 0.3 秒超时，测得单步位移超过 55 mm 或朝向偏转超过 2° 时停车。到位必须有四个不同时间戳的定位反馈。停车后用至少 1 秒的新定位确认位置散布不超过 2 mm，随后机械臂运行期间监测底盘偏移。底盘专用配置关闭原来的速度启动加成，并在驱动层增加仅本模式启用的 0.3 秒速度指令看门狗。看门狗依赖底盘驱动继续运行，不宣称替代硬件急停。

底盘微调默认关闭。启用必须提供实测矩阵、四组标定样本和与当前相机/左臂参考一致的标定摘要。例文件 calibration_confirmed=false、base_matrix=null，未标定时入口在启动节点之前拒绝运行。原单臂模式仍按参考文件中的 trigger_radius_m（现场目前 50 mm）判断；底盘微调模式单独收敛到 20 mm。

## 部署

补丁基于本地提交 2ff3fd8（已包含 50 mm 单臂配置及 ROS launch 参数修复），只含本功能代码与例配置，不包含文档或私有参考文件。Windows PowerShell：

```powershell
scp "D:\Desktop\小车gpt\jzzj\.local\stage-c-fixed-base-navigation\fixed-base-navigation.patch" orangepi@10.89.74.74:/home/orangepi/jzzj/.local/
```

先使现有任务停稳、取下水果并按原 Session 完成恢复释放，然后退出旧控制终端。Orange Pi SSH 中应用：

```bash
cd /home/orangepi/jzzj
git apply --check .local/fixed-base-navigation.patch &&
git apply .local/fixed-base-navigation.patch
bash tools/stage_c/build.sh --with-base-navigation
```

必须看到 STAGE_C_BUILD_COMPLETE。新增节点使用 catkin_install_python 安装，因此需要构建。定位构建依赖现场的 OpenCV/AprilTag 库；本地没有 ROS1/ARM64 构建环境，尚未确认现场依赖可用。

## 四组标定采集

需要场地 AprilTag 已在 tag_pose.yaml 配好 map 坐标，前相机能持续输出 `/front_car_pose_orig_map`。没有定位或定位陈旧时，记录程序和微调都拒绝运行。不要用水果相机坐标代替底盘 map 定位。

水果固定在桌上，保持同一个目标；标定期间机械臂保持后转待命角度 `[-90,0,90,90,0,60]`，不改变相机安装或桌高。先启动纯观察与定位（各一个 SSH 终端，先 source 环境）：

```bash
cd /home/orangepi/jzzj
source tools/stage_c/env.sh
bash tools/stage_c/start_camera.sh
```

```bash
cd /home/orangepi/jzzj
source tools/stage_c/env.sh
bash tools/stage_c/start_observe.sh yellow
```

```bash
cd /home/orangepi/jzzj
source tools/stage_c/env.sh
roslaunch car_localization car_localization.launch open_camera:=false visualize:=false rviz:=false
```

另需保持专用左臂驱动提供真实关节反馈；将机械臂放至后转待命姿态以后使用现有 `stage_c_fixed_arm.launch` 保持。该驱动的启动会使能力矩，不是卸力入口。不要在夹着水果时重新初始化。标定的 record 命令不会移动机械臂或底盘，只切换观察会话并记录数据。

每组都从停车开始，采集 before，然后由操作员让底盘沿指定车体方向平移约 10 mm（允许 5..30 mm），停稳后采集 after。可在确认底盘已卸载驱动时人工推移；不要强推锁住的车轮。保持朝向变化小于 0.5°。四组分别为车体 +X、-X、+Y、-Y，每组移动前后均记录。

在采集终端：

```bash
cd /home/orangepi/jzzj
source tools/stage_c/env.sh
mkdir -p .local/fixed_grasp/base_samples
python3 tools/stage_c/calibrate_fixed_base.py record \
  --output .local/fixed_grasp/base_samples/x_plus_before.json
```

操作员完成 +X 平移并停稳后：

```bash
python3 tools/stage_c/calibrate_fixed_base.py record \
  --output .local/fixed_grasp/base_samples/x_plus_after.json
```

以同样流程记录 `x_minus_before/after.json`、`y_plus_before/after.json`、`y_minus_before/after.json`。不要一次连续走完四个方向后才记录；每个方向都要自己的 before/after。文件不覆盖已有结果，重采时先保留旧数据并换新文件名。

离线拟合，使用独立 map 定位差及车体朝向得到真实车体位移，再拟合其与左臂水果坐标变化的关系，不假设正负方向：

```bash
python3 tools/stage_c/calibrate_fixed_base.py fit \
  --pair .local/fixed_grasp/base_samples/x_plus_before.json .local/fixed_grasp/base_samples/x_plus_after.json \
  --pair .local/fixed_grasp/base_samples/x_minus_before.json .local/fixed_grasp/base_samples/x_minus_after.json \
  --pair .local/fixed_grasp/base_samples/y_plus_before.json .local/fixed_grasp/base_samples/y_plus_after.json \
  --pair .local/fixed_grasp/base_samples/y_minus_before.json .local/fixed_grasp/base_samples/y_minus_after.json \
  --output .local/fixed_grasp/base_navigation.json
```

最大拟合残差必须不超过 3 mm；两个方向都需要正负样本，比例异常、朝向变化、机械臂姿态变化、目标类别或相机标定变化均拒绝拟合。拟合输出仍保留 calibration_confirmed=false。操作员审核真实采集过程和拟合输出以后才启用：

```bash
python3 - <<'PY'
import json, shutil, datetime
from pathlib import Path
p=Path('.local/fixed_grasp/base_navigation.json')
c=json.loads(p.read_text())
shutil.copy2(p,str(p)+'.bak-'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
c['calibration_confirmed']=True
p.write_text(json.dumps(c,indent=2)+'\n')
PY
bash tools/stage_c/start_fixed_grasp.sh --base-adjust \
  --base-config .local/fixed_grasp/base_navigation.json
```

最后一条仅离线检查，不启动硬件。不要为了通过校验填单位矩阵或复制其他机器的标定。

## 一键执行

关闭标定时启动的旧控制与观察会话，或保留完整的纯视觉/定位链供入口复用。旧底盘、普通导航或键盘控制必须退出，新入口不自动杀其他任务。水果应已取下，夹爪空，恢复标志已按原 Session 释放。

```bash
cd /home/orangepi/jzzj
bash tools/stage_c/start_fixed_grasp.sh \
  --reference .local/fixed_grasp/orange_reference_run.json \
  --base-adjust --base-config .local/fixed_grasp/base_navigation.json \
  --real --confirm-base-path-clear --confirm-fixed-path-clear
```

与单臂模式不同，此模式不能把底盘物理固定死。流程为初始停车确认、左臂后转待命、稳定观察、向 map 发布不超过 50 mm 的目标、到位停车再观察，直到水果三维距离不超过 20 mm，最终确认停车，然后原固定角度接近、等 3 秒、闭爪。单次完成后保持，不自动再次抓取，不自动抬升或放置。

总行程无上限意味着可以多次执行小步对齐；不会放宽单步、定位、目标连续性、停车反馈和路径确认。该功能未提供全局避障或自主搜寻，目标丢失即停止。Ctrl+C 请求停止；底盘驱动仍运行时看门狗能处理导航发送者消失，不能把主控整体断电/进程全部死亡当成已证实停车。

释放已完成的本模式会话时，保持本模式导航/定位/底盘节点运行，使用原 Session：

```bash
bash tools/stage_c/fixed_grasp.sh release \
  --reference .local/fixed_grasp/orange_reference_run.json \
  --base-adjust --base-config .local/fixed_grasp/base_navigation.json \
  --session ORIGINAL_SESSION --real --confirm-base-path-clear \
  --confirm-fixed-path-clear --confirm-stopped-and-clear
```

运行记录现在保存在参考文件同目录的 runs/ 中，避免 roslaunch 工作目录 ~/.ros 导致记录落到 ~/.ros/.local。新增 base_trace 记录对齐过程；长时间不限行程运行仅保留最近约 1000 条对齐事件以限制内存占用。

## 验证与限制

160 项相关离线回归通过，覆盖单步坐标变换、50 mm 指令上限、20 mm 三维对齐阈值、定位/心跳丢失、实际单步超程、四个不同定位到位、HOLD 取消、底盘漂移拒绝、驱动看门狗、旧控制冲突、未标定启动拒绝和超过旧总行程/步数上限的对齐。ROS1 launch 及 ARM64 实机未运行。此前单臂最后保持首次失败、异常处理保持成功的问题没有在本任务中改写或跳过；抓取成功仍以人工确认记录，不能等同于整套动作验收。

回退应先确认停稳、清空夹爪并退出本模式节点，再反向应用本次代码补丁；保留标定与运行记录。不要删除硬件锁或恢复标志强行交权。

## 现场构建修复：AprilTag 包

现场 CMake 首先报错 catkin_package 导出 include 目录不存在。该包只包含节点源文件，没有包内头文件；移除无效 INCLUDE_DIRS include 导出和本地 include 搜索项。同时将 geometry_msg 修正为 geometry_msgs，补充源码实际使用的 sensor_msgs、tf2_geometry_msgs 声明及 OpenCV include，改用 find_path/find_library 寻找 AprilTag 头文件和库，缺失时直接给出依赖错误。未改变检测、底盘或抓取逻辑。

使用临时桩 catkin/OpenCV 依赖验证 CMake configure/generate 成功并检查 package.xml 依赖一致性；这仅为构建配置检查，不是 ROS1/ARM64 编译或实体测试。上位机仍需重新执行 build.sh --with-base-navigation，无需删除 build/devel 目录。