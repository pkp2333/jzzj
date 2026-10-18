# C 阶段目录迁移验证（2026-10-18）

- 平台：Orange Pi，aarch64，ROS Noetic；现场 plus 基线 aca5c1c。
- 两个规范工作空间使用 tools/stage_c/build.sh 在本机新建 build/devel，编译成功，包含相机、检测、机械臂、C 接口及运动学。
- C 阶段离线用例：117 项，114 通过，3 项缺少外部材料而跳过。
- 工程及目录用例：13 项全部通过。合计 130 项，127 通过，3 跳过。
- 新脚本通过 bash -n；观察 launch 节点解析、真实控制 launch 参数展开成功。观察默认只含一个检测发布链路。
- rospack 和 Python 包路径均落在 /home/orangepi/jzzj；不加载已退役的 jzzj-stagec 或 stagec_verify_ws。
- 完整原文件和旧 C 构建产物先归档、校验 SHA-256，再按清单逐项检查未变化后删除；其他工作空间整体保留。
- 检查日志保存在 .local/migration-stage-c-20261018/。清单包含备份、模型和本次编译产物哈希。

此次只整理目录、参数接线和构建入口；没有合并此前暂停的 RGB 持物算法整改，没有启动机械臂或完成实机抓取验收。
