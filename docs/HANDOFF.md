# 交接文档 — lio_sam_hesai 定位稳定化

更新日期：2026-09-09
当前分支：`feature/p2-tiled-map`（分块地图特性；其余稳定化工作已合入 `main`）
基线提交：`caf8e29 docs(localization): refresh TF continuity verification`
工作区：交接时无未提交改动。

## 一、当前结论

原定的稳定化子项目均已合入 `main`：健康可观测性、低风险缺陷清理、外参统一、回放与 RSS 验证框架、LOST 自恢复、启动就绪握手、实时资源治理，以及定位 TF 连续性收尾。当前没有已批准但尚未开始的 P0/P1 子项目。

最后一轮完整验证记录于 2026-08-04：串行构建和安装成功；排除仓库既有的六项 lint/外部 schema 检查后，28 个已启用 CTest 全部通过；`hesai_sim` 的 quick 回放和 TF 故障注入回放均以退出码 0 完成。详见第 3 节。

## 二、已完成的功能与对应资料

| 子项目 | 状态 | 交付内容 |
| --- | --- | --- |
| A：定位可观测性与降级状态机 | 已完成 | `INIT → RELOCALIZING → TRACKING → DEGRADED → LOST` 状态机、雷达/IMU 数据流看门狗、`/diagnostics`、odom 协方差可信度标记。|
| A 收尾：TF 连续性 | 已完成 | 有有效定位锚点后，正常发布测量 TF；无新结果时短时 IMU 外推；IMU 不可用或超时则保持最后锚点。诊断发布 `tf_mode`、`tf_hold_reason`、锚点/外推年龄。|
| F：代码缺陷清理 | 已完成 | iVox 批量添加的提前 `return` 已改为 `continue`；已删除未编译的 `frontend.cpp` 与 `loop_closure.cpp`；公共头不再泄漏 `using namespace std;`。|
| B：坐标与外参基准统一 | 已完成 | mapping 与两个 localization launch 都使用 `base_link → hesai_lidar = [0.02, 0, 0.32]`、`base_link → imu_link = [0.02, 0, 0.22]`，由此得到 YAML 中的 `T^imu_lidar = [0, 0, 0.10]`。|
| C：稳定性验证框架 | 已完成 | 外置 `hesai_sim` bag/地图身份校验、quick 指标基线回归、RSS 长测、结果产物、手动/定时 Orin CI；同时提供 TF 故障注入回放。|
| E：故障自恢复闭环 | 已完成 | 每次进入 `LOST` 立即自动重定位一次；仅在数据流从异常恢复为健康时再尝试一次；手动 `/initialpose` 可打破自动等待。|
| D1：启动就绪握手 | 已完成 | 在订阅和工作线程启动前同步加载地图、地面图、kd-tree 与可选 Scan Context；失败直接拒绝启动；诊断暴露准备耗时。|
| D2：实时化与资源治理 | 已完成 | 预处理/定位线程默认请求 FIFO 与 CPU 亲和，权限或 cpuset 不满足时降级运行并上报；500 ms 单帧处理看门狗在安全检查点丢弃超期帧并恢复状态。|
| P2-1：大场景分块地图按需加载 | 已完成 | 新增 `localization_map` 模块：共享 tile 索引、带路径校验的 v1 manifest、原子分片生成器 + CLI、有界 LRU 缓存、`GlobalMapProvider`/`TiledMapProvider` 统一接口、单 worker 异步加载。支持冷启动 Scan Context 重定位与跟踪，内存受活动范围与缓存预算约束；旧全图路径不变。|

主要设计与实施记录：

- A：[设计](superpowers/specs/2026-07-27-localization-health-observability-design.md) / [计划](superpowers/plans/2026-07-27-localization-health-observability.md)
- B：[设计](superpowers/specs/2026-07-28-extrinsic-frame-unification-design.md)
- C：[设计](superpowers/specs/2026-07-28-localization-stability-validation-design.md) / [计划](superpowers/plans/2026-07-28-localization-stability-validation.md)
- E：[设计](superpowers/specs/2026-07-29-localization-fault-self-recovery-design.md) / [计划](superpowers/plans/2026-07-29-localization-fault-self-recovery.md)
- D1：[设计](superpowers/specs/2026-07-30-localization-startup-readiness-design.md) / [计划](superpowers/plans/2026-07-30-localization-startup-readiness.md)
- D2：[设计](superpowers/specs/2026-07-30-localization-realtime-resource-governance-design.md) / [计划](superpowers/plans/2026-07-30-localization-realtime-resource-governance.md)
- TF 收尾：[设计](superpowers/specs/2026-08-03-localization-tf-continuity-design.md) / [计划](superpowers/plans/2026-08-03-localization-tf-continuity.md)
- P2-1 分块地图：[设计](superpowers/specs/2026-09-07-tiled-map-on-demand-loading-design.md) / [计划](superpowers/plans/2026-09-07-tiled-map-on-demand-loading.md)

## 三、已记录的验证证据与复跑命令

### 2026-08-04 最终记录

- `cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1`：成功。
- `cmake --install /home/robot/slam_ws/build/lio_sam_hesai`：成功。
- 已启用 CTest：28/28 通过（包括 TF 控制器、恢复闸门、IMU cache、运行时控制、稳定性 Python 契约）。
- quick 回放：`error: null`、`valid_odom_ratio: 1.0`、无 error sample；最终一次的首条 odom 为 5.00 s、最大 odom 间隔 0.400 s。
- TF 故障注入：`error: null`、`violations: []`；记录 1294 个 TF，最大活动间隔 0.1 s；在约 40.050 s 与 60.140 s 观测到两次对应故障窗口的 `stream_recovered` 自动恢复触发。

### 2026-09-09 分块地图验收记录

- 构建 `build/lio_sam_hesai-p2-tiled-map`（RelWithDebInfo）成功；排除仓库既有六项 lint 后，36 个已启用 CTest 全部通过（含 `test_tile_index`、`test_tile_manifest`、`test_tile_map_generator`、`test_tile_cache`、`test_global_map_provider`、`test_tiled_map_provider`、`test_tiled_map_localization_surface` 及 `test_tiled_map_integration`）。
- 旧全图 `quick` 复验：`map_mode: global`、`error: null`、`valid_odom_ratio: 1.0`、`first_ok_seconds: 3.0`。
- 分块 `tiled-quick`（无 `/initialpose` 冷启动）：`map_mode: tiled`、`error: null`、`error_samples: 0`、`valid_odom_ratio: 1.0`、odom 时间戳单调；`first_ok_seconds: 3.15`（自动重定位达标）；缓存 `map_active_tile_count: 4`、`map_cached_tile_count: 4`、`map_cache_mib: 7.78`，无 tile 加载失败。
- 分块 `tiled-faults`：corrupt-manifest 与 missing-center 均拒绝就绪（`first_ok_seconds` 为 null）；missing-neighbor 产生降级诊断（`max_diagnostic_level: 1`）且 `map_tile_load_error` 精确含缺失索引 `[-1,0]`，`error_samples: 0` 无崩溃。

在具备 ROS 2、外置数据与本机 DDS/UDP 权限的主机上，按下面顺序复跑：

```bash
cd /home/robot/slam_ws
cmake --build build/lio_sam_hesai -- -j1
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
cmake --install build/lio_sam_hesai

cd src/lio_sam_hesai
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh quick
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh tf-continuity
# 长测（在配置好 Orin 共享负载时执行）
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh memory
```

六项被排除的 CTest（`copyright`、`cpplint`、`flake8`、`lint_cmake`、`uncrustify`、`xmllint`）是仓库历史规范债务；其中 `xmllint` 还依赖不可离线获取的 ROS schema。它们不应被误解为本轮功能回归，但若后续专门治理 lint，应独立立项并收敛为全绿。

## 四、运行行为与部署注意事项

### TF 和恢复语义

- `map → base_link` 的回退 TF 只在已经获得有效定位锚点后存在；启动未定位阶段不会伪造 TF。
- 外推/保持只发布 TF，绝不写入 odometry、path、当前点云或定位内部状态。
- TF 使用 ROS 时间作发布节拍、传感器时间作外推段；时钟回跳会清空 IMU 缓存、重置角速度积分状态并隔离回跳前排队结果，避免跨 epoch 发布。
- LOST 的自动尝试严格为“进入 LOST 一次 + 每个异常到健康边沿一次”，不是按帧或按定时器重试。

### 实时资源默认值

`runtime` 默认启用 FIFO 请求：预处理优先级 70/CPU 4，定位优先级 80/CPU 5，单帧时限 500 ms。`pthread_setschedparam` 的 `EPERM`（缺少 `CAP_SYS_NICE`）或亲和性 `EINVAL` 不会使节点退出；节点会继续以系统允许的调度运行，并在 `/diagnostics` 与日志中报告实际生效状态。设置 CPU 核为 `-1` 表示不绑定。

部署前仍应按实车频率、安装外参、CPU 隔离策略和共享负载校准：

1. `localization_health.*` 的 fitness、帧数和数据流阈值；
2. `runtime.*` 的 FIFO 优先级、CPU 核和帧时限；
3. `localization_tf.*` 的发布率与最大外推时长；
4. RSS 长测的运行时长、峰值和增长斜率上限。

## 五、后续工作

没有活动中的 P0/P1 子项目。P2-1 大场景分块地图已完成（见第 7 节）。剩余 P2 顺序，下一项需在开始前重新确认范围并建立新的 spec：

1. **建图/重定位统一坐标与命名体系**：逐步收敛两条历史代码路径，降低维护和配置歧义。
2. **地图产物版本化与兼容校验**：为 PCD、`sc_db.bin` 和参数快照建立版本戳、元数据与加载期校验（本项目只定义分块加载所需的最小 manifest 版本与一致性检查）。

`docs/ARCHITECTURE.md` 是 7 月的静态风险评估，仍含“未完成”及旧文件存在等历史描述；开始新工作前应以当前源码、提交历史和本交接文档为准，再单独更新该架构文档。

## 六、环境、约定与入口

- 平台：Jetson Orin NX 16GB，JetPack 6（L4T R36.3 / Ubuntu 22.04 / aarch64 / 8 核），ROS 2 Humble。
- 工作区：`/home/robot/slam_ws`；包：`/home/robot/slam_ws/src/lio_sam_hesai`。
- 外置验收数据：`/home/robot/rosbag2/hesai_sim` 与 `/home/robot/map/hesai_sim2`；可用 `LIO_SAM_TESTDATA_ROOT`、`LIO_SAM_WORKSPACE` 覆盖。
- 构建入口：`colcon build --symlink-install --packages-select lio_sam_hesai` 或已有 CMake 构建目录的 `cmake --build`。
- `docs/` 与 `test/` 受 `.gitignore` 影响。需要提交其中的文档时，使用 `git add -f <path>`；正常代码和配置不应依赖强制添加。
- 当前已直接在 `main` 开发。后续是否继续直接提交 `main`，应以当次用户/团队指令为准。

关键入口：

- 建图：[launch/mapping.launch.py](../launch/mapping.launch.py) / [config/mapping.yaml](../config/mapping.yaml)
- 重定位：[launch/localization.launch.py](../launch/localization.launch.py) / [config/relocalization.yaml](../config/relocalization.yaml)
- 编排、队列、TF 与诊断：[include/slam/system.h](../include/slam/system.h) / [src/slam/system.cpp](../src/slam/system.cpp)
- 跟踪、预测和自动重定位：[src/slam/localization.cpp](../src/slam/localization.cpp)
- 健康、恢复与 TF 控制：[include/localization_health/](../include/localization_health/)
- 运行时调度与帧看门狗：[include/runtime/](../include/runtime/)
- 回放/RSS/TF 验收：[scripts/run_localization_stability.sh](../scripts/run_localization_stability.sh) / [test/integration/](../test/integration/)

## 七、分块地图（tiled map）运行说明

### 模式选择

`localization.map_mode` 取 `auto`（默认）、`global`、`tiled`：

- `auto`：`localization.map_path` 指向普通 PCD 文件时走全图；指向含 `manifest.yaml` 的目录时走分块。
- `global`：强制全图路径，`map_path` 必须是可读 PCD。
- `tiled`：强制分块目录，manifest 缺失或无效时拒绝地图就绪。

旧 YAML 未设 `map_mode` 且 `map_path` 指向 `GlobalMap.pcd` 时行为与之前完全一致，不会隐式转换或移动旧地图文件。

### 目录布局

```
tiled/
├── manifest.yaml      # schema_version: 1，含 frame_id/point_type/tile_size_m/origin/source/tiles/overview
├── overview.pcd       # 5 米分辨率全局概览（可选，缺失只警告不影响定位）
├── global/            # 全局点云 tile：<x>_<y>.pcd
└── ground/            # 地面点云 tile：<x>_<y>.pcd（可选）
```

tile 索引由显式原点决定：`tile_x = floor((x - origin_x) / tile_size_m)`。生成器与运行时共享同一索引函数，覆盖负坐标与边界点。

### 生成命令

分块目录用离线 CLI 从现有 `GlobalMap.pcd` + 可选 `GroundMap.pcd` 生成，写入临时目录、全部落盘后原子重命名，已有目标目录默认拒绝覆盖：

```bash
lio_sam_hesai_generate_tiled_map /home/robot/map/hesai_sim2 \
  --output-subdirectory tiled --tile-size-m 100 \
  --origin-x 0 --origin-y 0 --overview-resolution-m 5 --frame-id map
```

也可通过 `scripts/run_localization_stability.sh tiled-generate` 一键执行。`map_save.save_tiled_map` 默认关闭；仅在建图流程显式开启后才在保存 PCD/`sc_db.bin` 之外额外产出分块目录。

### 回滚

将 `localization.map_mode` 设为 `global`（或让 `map_path` 指向 `GlobalMap.pcd`）即可退回旧全图路径；分块目录与旧产物互不影响。

### 诊断键

`/diagnostics` 新增：`map_mode`、`map_manifest_version`、`map_active_tile_count`、`map_cached_tile_count`、`map_cache_mib`、`map_cache_hits`、`map_cache_misses`、`map_tile_load_ms`、`map_tile_load_failures`、`map_tile_load_state`、`map_tile_load_error`。

### 实测结果（2026-09-09）

- 分块冷启动自动重定位 `first_ok_seconds: 3.15`，`valid_odom_ratio: 1.0`，odom 单调，无 error sample。
- 缓存峰值 `map_active_tile_count: 4`、`map_cached_tile_count: 4`、`map_cache_mib: 7.78`，远低于默认上限（25 tiles / 1024 MiB）。
- 故障注入：中心 tile / manifest 损坏拒绝就绪；邻接 tile 缺失仅降级并精确定位缺失索引。

### 已知限制与待办

- 启动/稳定 RSS 与全图模式的逐项对比（地图准备耗时、最大帧耗时）尚未完成测量；记录后应回填本节与第 3 节。分块模式不常驻完整 `GlobalMap.pcd`、全图 XY kd-tree 或完整地面 kd-tree，但缺实测量化对比。
- tile 请求按 manifest 实际范围裁剪，超出地图边界的邻域不报“缺失 tile”，避免地图边缘误判。
- 分块注册体素尺寸固定为 0.3 m 以匹配旧全图密度；如后续调整全图密度需同步此处。
- manifest 字段扩展遵循向后兼容：未知可选字段忽略，未知 `schema_version` 拒绝加载；密码学摘要、参数快照与跨版本迁移留给 P2-3。
