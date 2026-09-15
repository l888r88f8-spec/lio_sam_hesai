# 项目状态文档 — lio_sam_hesai 定位稳定化

更新日期：2026-09-15
当前分支：`main`
当前提交：`db15459 docs(p2): record post-merge hardening evidence`
远端状态：本地 `main` 相对 `origin/main` ahead 183，尚未 push。
工作区：更新本文档前已有一项用户改动——`docs/P2_2_MIGRATION.md` 被删除；本轮不处理、不恢复该删除。

## 一、当前结论

**P2.2（建图/重定位坐标与命名收敛）及其修复、复核和合并后加固均已完成，并已进入本地 `main`。** 第 8 节记录的是 2026-09-12 的阶段性未签收状态；第 9 节随后修复了 launch 参数方向、相对地图路径、分块覆盖语义和回放判据等问题。原先阻塞签收的 `tiled-quick`、`tiled-faults` 已在修复轮与复核轮通过，不再是当前阻塞项。代码加固止于 `bbfd47d`；其后的 `db15459` 只修改本文档与本轮计划文档，验证与未重跑范围见 9.9。

原定的稳定化子项目均已合入 `main`：健康可观测性、低风险缺陷清理、外参统一、回放与 RSS 验证框架、LOST 自恢复、启动就绪握手、实时资源治理，以及定位 TF 连续性收尾。当前没有已批准但尚未开始的 P0/P1 子项目。

稳定化基线的历史完整验证记录于 2026-08-04；P2.2 修复轮、独立复核轮与合并后加固轮的最新证据记录于 2026-09-15。9.9 当轮记录的约定 CTest 门禁为 44/44、Python 测试为 173/173，并完整重跑了 `tiled-faults`；相关临时构建与结果目录现已不存在，本次文档更新没有在 `db15459` 上重跑这些检查。`quick`、`tf-continuity`、`tiled-quick` 的最近一次真实全绿回放仍来自 9.8，不能表述为已在 `bbfd47d` 上重跑。详见第 3 节与 9.8-9.9。

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
| P2-2：坐标与命名收敛 | 已完成，已进入本地 `main` | 新增公共契约层：有方向约束的 `RigidTransform3d`（`parent_T_child` 编译期语义）、`RosPoseAdapter`、不可变 `SlamConfig` + legacy 键兼容解析、`MapArtifactLayout`。重定位迁移到注入式配置并删除 `ConfigParameters` 单例；建图 `ParamServer` 收敛为纯配置持有者，全部话题改为 `pipeline_topic`/`command_topic`/`output_topic` 分组。修复轮与加固轮补齐 launch 参数、相对地图路径、分块覆盖、地图身份绑定、初始位姿订阅握手及回放判据；原分块回放阻塞已解除（见第 9 节）。|
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

P2-1 与 P2.2 均已完成并进入本地 `main`；第 8 节的分块回放阻塞已由第 9 节修复，不再列为后续工作。**P2.3 当前暂缓，不是待立即执行的下一项；未经用户明确恢复并重新确认范围，不开始设计、实现或验证。** 其余条目也需在开始前确认范围并建立新的 spec：

1. **P2.3 地图产物版本化与兼容校验（暂缓）**：原计划为 PCD、`sc_db.bin` 和参数快照建立版本戳、元数据与加载期校验（当前只定义了分块加载所需的最小 manifest 版本与一致性检查）；保留为已知方向，暂不推进。
2. **`TightCouplingKF` 产品决策**：配置校验器接受该模式，但运行期仍 `LOG(FATAL)`；需决定实现、隐藏还是在配置期明确拒绝。
3. **历史 lint 基线治理**：`flake8`、`lint_cmake`、`uncrustify`、`xmllint` 仍未全绿，应独立立项，不与功能回归混淆。

`docs/ARCHITECTURE.md` 是 7 月的静态风险评估，仍含“未完成”及旧文件存在等历史描述；开始新工作前应以当前源码、提交历史和本状态文档为准，再单独更新该架构文档。

## 六、环境、约定与入口

- 平台：Jetson Orin NX 16GB，JetPack 6（L4T R36.3 / Ubuntu 22.04 / aarch64 / 8 核），ROS 2 Humble。
- 工作区：`/home/robot/slam_ws`；包：`/home/robot/slam_ws/src/lio_sam_hesai`。
- 外置验收数据：`/home/robot/rosbag2/hesai_sim` 与 `/home/robot/map/hesai_sim2`；可用 `LIO_SAM_TESTDATA_ROOT`、`LIO_SAM_WORKSPACE` 覆盖。
- 构建入口：`colcon build --symlink-install --packages-select lio_sam_hesai` 或已有 CMake 构建目录的 `cmake --build`。
- `docs/` 与 `test/` 受 `.gitignore` 影响：已跟踪文件修改后使用普通 `git add`，只有新增且仍被忽略的文件才需要 `git add -f <path>`；正常代码和配置不应依赖强制添加。
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

## 八、P2.2 坐标与命名收敛（2026-09-12）

> 本节保留 2026-09-12 的阶段性历史记录；其中“未合入”“未签收”和分块回放阻塞均已被第 9 节的修复、复核与合并后加固取代，不代表当前状态。

设计：[spec](superpowers/specs/2026-09-10-mapping-localization-coordinate-naming-unification-design.md) / 计划：[plan](superpowers/plans/2026-09-10-mapping-localization-coordinate-naming-unification.md)

分支 `p2.2-coordinate-naming-unification`，从 `main` 的 `30d6e88` 分出，共 14 个提交（`a0fd37f` … `7e0d1da`）。**未合入 `main`**，用户要求显式批准后才合并。

### 状态：已实现到 Task 8，未签收

Task 1-8 全部完成。Task 9 的静态检查、构建、CTest、全图回放、TF 连续性回放、建图 smoke 回放均通过；两条分块回放检查失败，且已在未改动的 `main` 上复现同样的失败，因此作为既有缺口记录，P2.2 按计划保持未完成。

### 验证证据

静态检查：

- `git diff --check HEAD~8` 与 `git diff --check main..HEAD`：均无输出（退出码 0）。
- `rg 'ConfigParameters::Instance|T_[A-Za-z0-9_]+_to_[A-Za-z0-9_]+|last_odom_state_' include src`：无匹配。

构建与测试：

- `colcon build --symlink-install --packages-select lio_sam_hesai --cmake-args -DBUILD_TESTING=ON --executor sequential --parallel-workers 1`：成功（在既有 build 目录上增量 13.8 s）。
- `cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1`（全部目标）：成功，0 error。
- `ctest -E '(copyright|cpplint)'`：**42/46 通过**。失败 4 项为 `uncrustify`、`lint_cmake`、`flake8`、`xmllint`，与 `main` 一致（既有 lint 基线，见第 3 节说明）。P2.2 新增/修改的源码没有引入任何新的 lint 违规：三个 launch 文件的 flake8 E501 数量与 `main` 完全相同（4 条，全部为改动前既有行），另修掉了 `main` 上的一条 E302。

定位回放（`scripts/run_localization_stability.sh`，`hesai_sim` bag，基线 `test/integration/baselines/hesai_sim.yaml`）：

- `quick`（全图）：**退出码 0**。`map_mode: global`、`error_samples: 0`、`timestamp_monotonic: true`、`valid_odom_ratio: 1.0`、`first_ok_seconds: 2.95`、`max_non_ok_gap_seconds: 2.95`、`max_odom_gap_seconds: 0.490`、`max_reg_fitness: 1.0`。全部满足基线阈值。
- `tf-continuity`：**退出码 0**，`violations: []`；`map -> base_link` 连续，无定位 odom TF。
- `tiled-quick`：**失败** — `metric violation: max_non_ok_gap_seconds must be <= 5.533784914016724, observed 49.73233079910278`。
- `tiled-faults`：**失败** — `fault violation: missing-neighbor diagnostic must contain [-4,-5]`。

两条分块失败的 A/B 对照（在 `/tmp/lio_main_ws` 用 `main`（`30d6e88`）重新构建后运行同样的命令）：

- `main` 的 `tiled-quick`：`observed 49.72324180603027`（与本分支的 49.7239 s / 49.7239 s 相差 < 0.01 s）。
- `main` 的 `tiled-faults`：`fault violation: missing-neighbor diagnostic must contain [-4,-5]`（与本分支信息完全相同）。

结论：两条失败都是既有的分块基线缺口，不是 P2.2 回归。结构性佐证：P2.2 对 `src/localization_map/` 的改动只有 `tile_manifest.cpp` 的 3 处字面量改为同一常量 `MapArtifactLayout::kManifestName`（字符串逐字节相同）和 `tile_map_generator.cpp`（仅在生成路径运行）；`tiled_map_provider.cpp`、`tile_cache.cpp`、`tile_index.cpp` 与整个 `src/localization_health/` 未被改动。基线中 `max_non_ok_gap_seconds: 5.53` 是 2026-07-29 从**全图**模式标定的（`7e6f9f2`），而 `tiled-quick` 是 2026-09-09 才加入的（`bb648bd`），未重新标定或扩展基线。

建图 smoke 回放（`hesai_sim` bag，1.5 GB，`--clock --rate 1.0`，采样 60 s）：

- 命令：`ros2 launch lio_sam_hesai mapping.launch.py use_sim_time:=true` + `ros2 bag play <bag> --clock --rate 1.0`。
- TF：`map -> odom` = 平移 `[0.000, 0.000, 0.000]`、四元数 `[0,0,0,1]`（静态单位变换）；`odom -> base_link` 动态，两次采样分别为 `[19.132, -0.018, -0.218]`、`[26.702, -0.021, -0.182]`；`base_link -> hesai_lidar` = `[0.020, 0.000, 0.320]`；`base_link -> imu_link` = `[0.020, 0.000, 0.220]`。
- 话题：P2.2 前建图全部话题字符串均存在（`lio_sam/deskew/cloud_info`、`cloud_deskewed`、`ground_cloud_global`、`lio_sam/feature/cloud_info`、`cloud_corner`、`cloud_surface`、`lio_sam/mapping/odometry`、`odometry_incremental`、`path`、`map_global`、`map_local`、`cloud_registered`、`cloud_registered_raw`、`ground_cloud_global`、`trajectory`、`lio_sam/imu/path`、`icp_loop_closure_history_cloud`、`loop_closure_constraints`、`lio_sam/cloud_nonground`、`odometry/imu`、`odometry/imu_incremental`、`lio_loop/loop_closure_detection`）。话题表中出现的 `/lio_sam/localization/*` 与 `/initialpose` 来自 RViz 配置的订阅与 SetInitialPose 工具（`ros2 topic info` 显示 0 个发布者、1 个订阅），不是建图节点新增的发布。
- 进程退出：`bag_play_exit=0`，建图各节点无异常退出。

### 本轮发现并修复的回归（Task 6 引入）

建图 smoke 回放第一次运行时 `lio_sam_hesai_mapOptimization` 在启动约 13 s 后 `terminate called after throwing an instance of 'std::invalid_argument'`（exit code -6）。原因是 Task 6 把 `publishOdometry()` 的全局里程计改为

```cpp
OdomTLidar::FromMatrix(trans2Affine3f(transformTobeMapped).matrix().cast<double>())
```

而 `transformTobeMapped` 是 `float[6]`，`trans2Affine3f` 产生的是 float 旋转矩阵，widening 到 double 后正交性误差约 **2.08e-07**，远超 `RigidTransform3d::Validate` 的 `isApprox` 容差 **1e-12**，于是每次建图运行都会在建图开始时抛异常中止。

修复：在 `publishOdometry()` 里用同样的 RPY 角度在 **double** 下重建旋转（与改动前的 `tf2::setRPY` 路径一致），而不是回绕 float 矩阵。这同时消除了 Task 6 交接说明里标记的 ~1e-7 四元数偏差。新增回归测试 `RigidTransform.RejectsFloatPrecisionRotation` 固定这一行为。

该缺陷只被端到端 smoke 回放捕获——单元测试没有覆盖非单位位姿经过发布边界的情况。

### 遗留与注意事项

- 分块回放的阶段性阻塞已在第 9 节修复；本节只保留 2026-09-12 的历史记录。
- ~~退出自动保存仍写 `cloudCorner.pcd` / `cloudSurf.pcd` / `cloudGlobal.pcd` / `cloudGround.pcd`，而服务保存写 `CornerMap.pcd` / `SurfMap.pcd` / `GlobalMap.pcd` / `GroundMap.pcd`……只靠 Ctrl+C 保存的地图无法被定位或分块生成器加载（它们找 `GlobalMap.pcd`）。~~ **该缺陷已于 2026-09-15 按“删除而非重命名”解决**：Ctrl+C 路径、`map_save.enabled` / `savePCD` 参数和 `MapArtifactLayout::Saved*Cloud()` 全部删除，`/lio_sam/save_map` 成为唯一导出入口。当时选择不重命名磁盘文件，因此 `CornerMap.pcd` / `SurfMap.pcd` / `GlobalMap.pcd` / `GroundMap.pcd` 的名字保持不变。
- 空的 `localization.map_path` 默认值从 `<package>/data/map.pcd` 改为 `<package>/data/GlobalMap.pcd`（计划要求删除 `kGlobalMapFileName`）。`data/` 下从未存在 `map.pcd`，因此旧默认值是死代码；任何显式配置的路径不受影响。
- `docs/P2_2_MIGRATION.md` 受 `.gitignore` 的 `docs/` 规则影响但已被跟踪，修改后普通 `git add` 即可；当前工作区已将该文件删除且未暂存，本轮不决定其去留。
- 迁移文档中的 legacy→canonical 键表由 `test_unified_parameter_schema.cpp` 的 `kParameterContracts` 生成，改动该表后应重新生成。

## 九、P2.2 修复轮（2026-09-15）

按当时的本地 `docs/P2.2_fix.md` 执行；该文件未被 Git 跟踪且当前已不存在，本轮记录以 9.1-9.8 为准。本轮修复的是第 8 节记录的四类问题：launch 参数方向、相对地图路径、分块覆盖语义与故障回放、以及独立参数复核。该分支当时尚未合并，随后已并入本地 `main`（`740dab9`）；合并后的加固轮见 9.9。

### 9.1 状态区分

| 维度 | 状态 |
| --- | --- |
| 实现完成 | §0-§4 全部代码、配置、测试、脚本、文档改动完成 |
| 单元/契约测试 | 通过：48 项 CTest 中 44 项通过，失败 4 项全部是 `main` 上既有的 lint 基线 |
| 生产构建 | 通过：全新临时 `build/install/log` 目录从零构建，退出码 0，耗时 27 分 41 秒 |
| 回放验收 | 通过：`quick`、`tf-continuity`、`tiled-quick`、`tiled-faults` 四项全部退出码 0，建图 smoke 通过 |

### 9.2 干净构建与 CTest

```bash
cd /home/robot/slam_ws
colcon --log-base /tmp/p22_clean/log build --symlink-install \
  --packages-select lio_sam_hesai \
  --build-base /tmp/p22_clean/build --install-base /tmp/p22_clean/install \
  --cmake-args -DBUILD_TESTING=ON --executor sequential --parallel-workers 1
```

退出码 0。`--log-base` 是 colcon 的全局参数，必须写在 `build` 之前，否则报 `unrecognized arguments`。

- 干净树完整 CTest：44/48，失败项与增量树**完全一致**（`flake8`、`lint_cmake`、`uncrustify`、`xmllint`）。
- 约定 lint 门禁通过：

```bash
ctest --test-dir /tmp/p22_clean/build/lio_sam_hesai --output-on-failure -j1 \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

本轮新修改的文件都单独查过 flake8 / pydocstyle，没有引入新违规；三个 launch 的 E501 数量与 `main` 完全相同。

### 9.3 四项回放

全部使用干净 install（`LIO_SAM_WORKSPACE=/tmp/p22_clean`），避免误跑旧二进制。

```bash
LIO_SAM_WORKSPACE=/tmp/p22_clean LIO_SAM_KEEP_RESULTS=1 \
  scripts/run_localization_stability.sh {quick|tf-continuity|tiled-quick|tiled-faults}
```

| 模式 | 退出码 | 关键指标 |
| --- | --- | --- |
| `quick` | 0 | `map_mode: global`、`error_samples: 0`、odom 单调、`valid_odom_ratio: 1.0`、`first_ok: 2.87 s`、`max_non_ok_gap: 2.87 s`、`max_odom_gap: 0.40 s` |
| `tf-continuity` | 0 | `violations: []`、`error_samples: 0`、`valid_odom_ratio: 1.0` |
| `tiled-quick` | 0 | `map_mode: tiled`、`error_samples: 0`、`valid_odom_ratio: 1.0`、`max_non_ok_gap` 回到 ~2.9 s（修复前 49.7 s） |
| `tiled-faults` | 0 | 三个用例全部通过，见 9.4 |

结果目录在每次运行的 `results_dir=` 行；`LIO_SAM_KEEP_RESULTS=1` 时保留。

### 9.4 三类故障的验收证据

- **corrupt-manifest**：`corrupt_manifest_node_died: true`。按计划要求检查 launch 日志里节点自身的死亡，而不是假设子节点退出整个 launch 必然退出。
- **missing-center**：注入 payload 可证（`quarantine/-1_-1.pcd` 存在）、`map_tile_load_failures: 1`、诊断点名 `[-1,-1]`、且 `first_ok_seconds: null`、`valid_odom_ratio: null`——没有任何定位输出。
- **missing-neighbor**：`map_tile_load_failures: 3`、事件序列里同时出现 `global file validation failed for tile [-2,-2]` 与 `missing tiles: [-2,-2]`，且 `first_ok_seconds: 5.26`、`valid_odom_ratio: 1.0`、`error_samples: 0`——**降级但仍然正常定位**。

判定不再只看最终那条错误字符串，而是看 `map_tile_events` 记录的加载状态序列；「目标未请求」「只有启动 WARN」「索引只出现在最终汇总」「启动超时」四条假通过路径都有 mock 时序测试覆盖。

### 9.5 建图 smoke（60 秒）

干净 install，`use_sim:=true`，`map_save.directory` 覆盖到临时目录。

- `map -> odom`：平移 `[0.000, 0.000, 0.000]`，静态单位变换。
- `odom -> base_link`：动态，两次采样 `[19.439, -0.008, -0.217]` → `[27.031, -0.016, -0.177]`。
- `base_link -> hesai_lidar`：`[0.020, 0.000, 0.320]`；`base_link -> imu_link`：`[0.020, 0.000, 0.220]`。
- 22 个 P2.2 前建图话题**全部存在**；无 crash、无 `process has died`；`bag_play_exit=0`。
- 临时地图输出目录为空——没有任何写入落到配置的默认目录。

### 9.6 对第 8 节两处结论的修正

**一、"旧分支复现相同失败就证明 P2.2 无回归"这个结论过强。** 当时的 A/B 对照构建的是 `main`（`30d6e88`），而分支实际是从 `45df657` 分出的，`45df657`（`chore(p2.1): preserve P2.1 follow-up edits`）位于 `main` 之上，包含了用户的 P2.1 跟进改动。因此正确的说法是"在 `main` 上复现了相同的失败"，**不能**表述为"相对保留的 P2.1 基线已证明无回归"——真正的基线树从未参与那次对照。修复轮之后 `tiled-quick` 与 `tiled-faults` 均已通过，该问题不再影响签收，但历史记录的措辞应予更正。

**二、分块裁剪的根因描述不完整。** 第 8 节写的是"包围盒四角有 28 个空格子"，实测那 28 格确实是包围盒角上的覆盖外区域，不是数据内部的洞；`RootDirectoryOf` 与 `FromTiledRoot` 对末尾分隔符处理不一致也是同期发现的独立缺陷。

### 9.7 范围外遗留（未处理）

- ~~**退出自动保存的产物无人消费**：`map_save.enabled` 只控制这一条路径，写出的 `cloudCorner/cloudSurf/cloudGlobal/cloudGround.pcd` 没有任何回读方，且不写 `sc_db.bin`。~~ **已于 2026-09-15 关闭**：不采用“重命名磁盘文件”的方案，而是直接删除该路径；`map_save.enabled` 与 `savePCD` 现在在启动期 fail-closed 拒绝并提示改用 `/lio_sam/save_map`。验收证据见第 10 节。
- **`TightCouplingKF` 被校验器接受但运行期 `LOG(FATAL)`**：`ValidateLocalizationRuntime` 的报错信息把它列为合法值，`test_slam_config.cpp` 也断言它被接受，但 `localization.cpp` 对该模式走 `LOG(FATAL) << "Kalman filter support will be coming soon!"`。这是有意保留的未实现桩，属产品决策，本轮未改动。
- **四项 lint 基线**（`flake8`、`lint_cmake`、`uncrustify`、`xmllint`）在 `main` 上就失败，本轮保持不恶化。

### 9.8 复核轮修复（2026-09-15）

对 9.1-9.7 那批实现做独立复核后发现的 8 个问题（4 个必须修 + 4 个次要），全部已修复并复验。

**判据类（会误判通过，最危险）**

- `corrupt-manifest` 原先只匹配 `"process has died"`，而正常清理也会杀死 static TF 与地面分割节点。实测日志里定位节点是 `exit code -6`、其余是 `-2`，旧判据确实会误判。现改为精确匹配 `[lio_sam_hesai_relocalization-\d+]` + **非零退出码** + `Failed to prepare localization map`，两个信号必须同时成立。
- 保存的参考位姿原本取"第一条格式合法的 odom"，但初始化结果 `is_tracking=false` 同样会发布，所以记下的是**初始化位姿**（实测 2.52 s，而 first OK 是 2.78 s）。现在门控在"目标诊断 `is_tracking=true` 且 level OK"之后，实测记录到 2.78 s。
- `missing-neighbor` 原先分别检查"某次 tile 失败"和"整个运行有输出"，两者可以是无关的事。现在要求事件序列里出现 `ready` 状态且错误串同时含 `missing tiles:` 与故障索引——即**中心可用且产生了对应的降级状态**。

**安全类**

- fault CLI 原先直接按 `--tiled-dir` 构造 `TiledFaultFixture`，指向真实地图时 `inject()` 会 `rename()` 真实 tile 甚至 `manifest.yaml`。现在 `prepare_tiled_fault_case` 写入 `.p22-fault-fixture.json` 标记，`inject()` 强制校验来源、故障选择与 manifest 声明三者一致；harness 改为 `load_tiled_fault_fixture()`，**只能加载不能构造**，`--tiled-fault-relative` 从契约中移除。
- 参考位姿的身份绑定原先只覆盖 `DatasetIdentity`，现已扩展 `topic_counts`、`ground_map_bytes`、`sc_db_bytes`、`tiled_manifest_sha256`。**该绑定一启用就暴露了连带缺陷**：夹具用 `yaml.safe_dump` 重写 manifest，字节与真实地图不同、指纹必然对不上；新夹具本就不修改 manifest，改为逐字节复制。

**契约类**

- 非零回放 offset 时参考时间算错：保存的是"相对当前播放起点"，却当成"相对 bag 起点"使用。现为 `bag_seconds = start_offset_seconds + captured_clock - replay_clock_start`。当前 baseline offset 为 0 所以此前未暴露。
- `slam_config.cpp` 接受 `tile_size_m = 0`，而 `tile_map_generator.cpp` 运行期拒绝。已补 `RequirePositive`，配置校验与运行时契约对齐。
- `TiledFaultRun.observe_diagnostics` 原先合并全部 `DiagnosticStatus`，其他节点报同名键可能误开 prepare/IMU 闸门。现只读 `lio_sam_hesai: localization`。

**复核后的完整验收（干净树重建 27 分 37 秒，退出码 0）**

- 干净树 CTest：44/48，失败项与 `main` 的 4 个既有 lint 完全一致；约定 lint 门禁（排除 `copyright`/`cpplint`/`flake8`/`lint_cmake`/`uncrustify`/`xmllint`）零失败。
- 四项回放全部退出码 0：`quick`、`tf-continuity`、`tiled-quick`、`tiled-faults`。
- 建图 smoke（干净 install，60 秒，地图输出指向临时目录）：无 crash；`map -> odom` 单位变换；`odom -> base_link` 动态（19.74 → 28.03）；`base_link -> hesai_lidar` `[0.020, 0.000, 0.320]`；`base_link -> imu_link` `[0.020, 0.000, 0.220]`；临时地图目录为空。

### 9.9 合并后加固轮（2026-09-15）

9.8 那批复核修复已并入本地 `main`（当前基线 `740dab9`）。本轮从该提交拉出
`fix/p22-postmerge-hardening`，修复一并遗留的验收判据/校验缺口后重新验收。计划与设计见
`docs/superpowers/plans/2026-09-15-p22-postmerge-hardening.md` 与
`docs/superpowers/specs/2026-09-15-p22-postmerge-hardening-design.md`。
计划文件中的复选框未在实施后回填，仅作过程说明；完成状态与证据以本节为准。

**历史分支状态**：`fix/p22-postmerge-hardening` 的代码修复止于 `bbfd47d`（基于 `740dab9`）。
**当前集成状态**：`9ddbe69` 至 `bbfd47d` 以及证据提交 `db15459` 均已进入本地 `main`；
当前 HEAD 为 `db15459`。本地 `main` 尚未 push，相对 `origin/main` ahead 183。

**修复内容**（提交 `9ddbe69` → `bbfd47d`）

1. `9ddbe69` docs：本轮计划与设计文档。
2. `144f374` 配置/回放闸门：删除 `map_save.tiled_map.tile_size_m` 的重复
   `RequireNonNegative` 死代码并修正缩进，`test_slam_config.cpp` 新增
   `MappingRejectsNonPositiveOrNonFiniteTileSize` 锁定"有限且 > 0"契约；fault case 的
   center 缺失统一为 `DatasetError`；tracking 闸门改为非粘性，WARN/ERROR 或
   `is_tracking != true` 会复位，不会把失去跟踪后的位姿记成参考。
3. `20d4399` 地图身份：参考绑定从字节数升级为具名 artifact 的 SHA-256
   （`GlobalMap.pcd`/`GroundMap.pcd`/`sc_db.bin`，tiled 另含 `tiled/manifest.yaml`）；
   旧格式引用缺少 `artifacts` 键时 fail-closed 并要求重录；`schema_version` 仍为 1。
4. `37e98d2` 参考位姿发布：`/initialpose` 只在存在 matched subscriber 时发布，否则下次
   tick 重试，窗口内始终无订阅者则抛 `ReplayError`（不阻塞 executor）。
5. `ae9226d` lint：清掉 `06a2427` 引入的 2 处 E303（仅删空行）。
6. `64224f4` 判据解耦：record-reference 不再套用 quick 的性能阈值，但 `quick` 与
   `tiled-quick` 的性能门禁保持原样。
7. `bd87434` tile index 诊断比较只归一空白（`[-1, -1]` vs `[-1,-1]`），符号与数字仍须精确匹配。
8. `bbfd47d` 校验边界：显式 `null` offset 与数值溢出统一转 `DatasetError`，非迭代
   center 不再泄漏 `TypeError`；提取等价 subscriber helper，并补齐生产查询、
   `tiled-quick` 接线和回放边界测试。仅动 Python，无 C++ 改动。

**最终验证（在 `bbfd47d` 上执行；其代码与当前 HEAD `db15459` 相同）**

- 干净 C++ 构建：成功、退出码 0。colcon 前端在配置阶段后挂起，改用同一生成树
  `cmake --build ... --parallel 1` 完成编译、`cmake --install` 完成安装；`ae9226d` 之后
  无 C++ 源码改动，故该生成树的二进制对 HEAD 有效。
- CTest：共发现 50 项；约定门禁排除 `copyright`/`cpplint`/`flake8`/`lint_cmake`/
  `uncrustify`/`xmllint` 后，44/44 通过。另单独跑 `test_slam_config` 与两处
  `*TileSize*` 过滤器，均通过。本轮未把未执行的完整 50 项结果写成通过证据。
- Python：7 个文件 173/173 通过（`test_localization_regression` 72、
  `test_tiled_map_integration` 28、`test_localization_tf_continuity` 22、其余 4 个文件 51）。
- 新增行 flake8：0 违规（BASE `740dab9` 对照，新增/改动 1062 行；12 处既有违规未动）。
- `bash scripts/run_localization_stability.sh tiled-faults`：在 `bd87434` 上从头
  （含 reference 捕获）跑完，退出码 0，reference + 三个 fault case 全部执行：
  `corrupt-manifest` `corrupt_manifest_node_died: true`；`missing-center`
  `first_ok_seconds: null`、`valid_odom_ratio: null`、`map_tile_load_failures: 1`；
  `missing-neighbor` `first_ok_seconds: 8.19`、`valid_odom_ratio: 1.0`、
  `map_tile_load_failures: 3`。`bd87434 → bbfd47d` 只改 Python 验证边界并提取等价
  helper，最终 44/173 全绿即该脚本判据的复验。
- 证据原在 `.p22_build/tiled-faults-e2e-results/` 与 `.p22_build/final-verification/`；
  截至本次更新，两目录、临时 worktree 与当轮 `/tmp` 构建树均已不存在，因此这些历史数值当前无法从本机产物复核。

**本轮未做**：未 push；未做版权头或全仓 lint 治理（仅保证新增行 0 违规、既有基线不恶化）；
`tiled-quick`/`quick`/`tf-continuity` 的真实回放本轮**未重跑**（完整 `tiled-faults` 已跑），
其最近一次全绿记录仍属 9.8 轮。

**仍存的非本轮项**：P2.3、`TightCouplingKF` 运行期 `LOG(FATAL)` 桩、四项 lint 基线——
本轮未清零，也不宣称清零。9.7 的退出自动保存磁盘产物已按第 10 节关闭，不再属于遗留项。

## 十、退出自动保存删除（2026-09-15）

按 [计划](superpowers/plans/2026-09-15-remove-shutdown-auto-save.md) 执行，代码基线 `c9aa44a`（计划提交 `docs: plan shutdown auto-save removal` 与计划修订提交 `docs: revise shutdown auto-save removal plan` 都只改本计划文件，不含实现改动）。目标：Ctrl+C 不再写任何地图文件、不改动导出目录，`/lio_sam/save_map` 成为唯一导出入口，删除的参数在启动期 fail-closed 拒绝。

### 10.1 改动

- 删除 `src/mapOptimization.cpp` 的 `save_map_enabled_` 成员、构造赋值，以及 `visualizeGlobalMapThread()` 退出后的整段导出代码（`rm -r` / `mkdir` / `trajectory`、`transformations`、`cloudCorner`、`cloudSurf`、`cloudGlobal`、`cloudGround` 共 6 个 PCD 写入与 Z 偏移导出）。
- 删除仅供该路径使用的 `MapArtifactLayout::kSaved*CloudName` 常量与 `Saved*Cloud()` 访问器（`include/common/map_artifact_layout.h`、`src/common/map_artifact_layout.cpp`）。
- 删除 `MapSaveConfig::enabled`、`map_save.enabled` 读取（`savePCD` 别名随之失效）、`config/mapping.yaml` 的 `enabled` leaf，以及 `test_unified_parameter_schema.cpp` 的 `{"savePCD", "map_save.enabled", ...}` 契约行。
- 新增 `RejectRemovedMappingParameters()`：在 `LoadSlamConfig` 的任何 `Load*` 之前读取原始 override map，命中 `map_save.enabled` 或 `savePCD` 时抛出 `was removed with Ctrl+C automatic map saving; use /lio_sam/save_map instead`。
- 新增契约测试：`SlamConfig.RejectsRemovedShutdownAutoSaveParameters`、`UnifiedParameterSchema.ShutdownAutoSaveImplementationIsAbsent`、`UnifiedParameterSchema.ShutdownAutoSaveContract`（YAML/launch 反向契约 + guard 字面量精确计数）。
- 独立 code review 后的测试加固：`UnifiedParameterSchema.ProjectSurfaceRejectsLegacyExternalNames` 把这两个已删除参数并入 `kRemovedExternalNames`，在全部 8 个生产表面（`include/lio_sam/utility.hpp`、`src/imageProjection.cpp`、`src/slam/system.cpp`、`config/mapping.yaml`、`config/relocalization.yaml` 与三个 launch 文件）做反向扫描；`SlamConfig.RejectsRemovedShutdownAutoSaveParameters` 对 `false` 与 `true` 两种取值都要求拒绝，并点名 `/lio_sam/save_map`。
- 未新增脚本；生产路径零匹配与 guard 文件两处字面量各恰好 1 次由上述静态契约测试与 Task C 的一次性 `rg` allowlist 扫描加注入自检验证。

### 10.2 验证证据

- 基线构建与改动后全量构建：`cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1`，两次均退出码 0。
- 约定 CTest 门禁（排除 `copyright`/`cpplint`/`flake8`/`lint_cmake`/`uncrustify`/`xmllint`）：基线 44/44 通过，改动后 44/44 通过；完整输出见 `run/baseline-ctest.txt` 与 `run/final-ctest.txt`。
- 旧行为关闭路径基线：`/home/robot/shutdown_auto_save_evidence/old/` 的 `c9aa44a` 二进制配 `enabled: true`，在与新版完全相同的 fixture/等待/SIGINT harness 下：节点在信号前存活（`node_alive_before_signal=0`），日志含恰一次 `Map Optimization Started.` 与 `signal_handler(SIGINT/SIGTERM)`、恰一次 `Saving map to pcd files ...`，随后第一次空点云写入抛 `pcl::IOException`（`Input point cloud has no data`）并打印 `[ros2run]: Aborted`，wrapper **非零退出**（`old_exit` 本次观测值为 `250`，不作为契约；`wrapper_still_running=0`）；`/home/robot/shutdown_auto_save_smoke` 被 `mkdir` 创建但为空，`trajectory`/`transformations`/`cloudCorner`/`cloudSurf`/`cloudGlobal`/`cloudGround` 六个 PCD 全部不存在，日志无 `Saving map to pcd files completed`；该空目录归档为 `old-export/`（`old_export_is_dir=0`、`old_export_empty=0`，**不是** `cloudGlobal.pcd`）。
- 新二进制正向 smoke：与旧版相同的 fixture/等待/SIGINT harness（`final-mapping.yaml`），`smoke_dir_absent_before_signal=0`、`node_alive_before_signal=0`、`wrapper_still_running=0`，日志含恰一次 `Map Optimization Started.` 与 `signal_handler(SIGINT/SIGTERM)`，`new_exit=0`（`new_exit_zero=0`），导出目录在信号前与退出后始终不存在（`smoke_dir_absent_after_exit=0`），`Saving map to pcd files`、`pcl::IOException`、`Input point cloud has no data`、`[ros2run]: Aborted` 计数均为 0。
- 新二进制 fail-closed smoke：`enabled: true` 的 `legacy-enabled.yaml` 启动后 8 秒内自行退出（`still_running=0`），**退出码非零**（`fail_closed_exit` 本次观测值为 `250`，不作为契约），日志含 `map_save.enabled was removed with Ctrl+C automatic map saving; use /lio_sam/save_map instead`，零写入（导出目录不存在）。
- `/lio_sam/save_map` 服务接口 smoke：普通 `ros2 service list` / `ros2 service type` 所依赖的 daemon XML-RPC socket 被沙箱以 `PermissionError` 阻断（该路 `ros2 service list` 输出为空）；随后用无 daemon 的 typed list `ros2 service list --no-daemon --spin-time 5 -t` 成功发现 `/lio_sam/save_map [lio_sam_hesai/srv/SaveMap]`，节点收到 SIGINT 后退出码 0（`service_exit_zero=0`）。**该 smoke 只通过无 daemon typed list 验证了服务名称与类型**，未单独跑通 `ros2 service type`，也未调用服务导出。
- 静态 allowlist 扫描（Task C 的一次性 `rg`，非脚本）：生产路径 `include/ src/ config/ launch/` 的三路扫描 A/B/C 均无匹配（`exit_A=1`、`exit_B=1`、`exit_C=1`）；guard 文件 `src/common/slam_config.cpp` 的 `map_save.enabled` 与 `savePCD` 各恰好 1 次（`run/guard-literals.txt` 仅 1 行，`rg -o | sort | uniq -c` 两行计数均为 1）；注入自检在证据副本上恰好命中 2 行（`injected_scan_exit=0`），仓库 `config/mapping.yaml` 扫描无匹配（`repository_scan_exit=1`）且与备份逐字节相同（`repository_cmp_exit=0`）。
- 证据文件位置：`/home/robot/shutdown_auto_save_evidence/`（`old/`、`new/`、`run/`、`old-export/`）；smoke 目标目录 `/home/robot/shutdown_auto_save_smoke` 在全部 smoke 结束后不存在。

### 10.3 未做

- 未 push。
- 未做全量 lint 治理，四项既有 lint 基线（`flake8`、`lint_cmake`、`uncrustify`、`xmllint`）未清零。
- 未做真实 bag 导出：`/lio_sam/save_map` 只做了服务接口 smoke（注册与类型），没有在真实 bag 回放中调用服务导出地图。
