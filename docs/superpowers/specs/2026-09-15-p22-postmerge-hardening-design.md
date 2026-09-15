# P2.2 合并后加固设计

**日期：** 2026-09-15

**状态：** 已确认（按批准顺序实施）

**分支：** `fix/p22-postmerge-hardening`，基线 `740dab9`（本地 `main` 已包含 P2.2 修复轮与复核轮）

**实施计划：** `docs/superpowers/plans/2026-09-15-p22-postmerge-hardening.md`

## 1. 目标

在 P2.2 并入本地 `main`（`740dab9`）之后，修掉复核遗留的 6 类正确性/契约问题，并以
"全部单测 + 串行干净构建 + 44 项约定 CTest + `tiled-faults`"重新签收。本轮不新增
功能、不改变 ROS 话题与 TF 拓扑、不改变磁盘产物格式与 `schema_version`。

## 2. 范围

### 2.1 包含

1. `src/common/slam_config.cpp` 重复 `tile_size_m` 校验清理与配置守护测试。
2. `prepare_tiled_fault_case(..., center_index=None)` 的 `TypeError` → `DatasetError`。
3. `MetricCollector` tracking 闸门去粘性。
4. 地图身份从字节数升级为具名 artifact 的 SHA-256（含引用时间/矩阵校验收紧）。
5. `/initialpose` 发布前的 matched subscriber 闸门（发布回调返回 `bool`，不阻塞 executor）。
6. 全部单测、串行干净构建、44 项约定 CTest、`tiled-faults` 重新验收。
7. 本轮新增 flake8 E303 清零；`docs/PROJECT_STATUS.md` 更新为"本地 `main` 已合并、修复重新验收"。

### 2.2 不包含

- 版权整改、全仓库 lint 清理；`E501`/`E402`/`E203` 等既有基线保持不恶化。
- P2.3（地图产物版本与兼容协议）。
- 磁盘产物改名、`TightCouplingKF` 未实现桩、退出自动保存产物无人消费等 §9.7 遗留。
- 改变 `DatasetIdentity.db3_bytes`：本轮只处理"地图身份"；bag 身份已由
  `duration_ns`/`message_count`/`topic_counts` 约束，哈希 1.5 GB 级 `*.db3` 不在
  本轮预算内。

## 3. 现状与缺陷

### 3.1 配置校验重复且不可测

`src/common/slam_config.cpp:765-773` 先对 `map_save.tiled_map.tile_size_m` 调用
`RequirePositive`，随后又对同一字段调用 `RequireNonNegative`（死代码）；中间夹着
缩进错位的 `RequirePositive(system.mapping_process_interval, ...)`。运行期契约见
`src/localization_map/tile_map_generator.cpp:90-92`：必须有限且 `> 0`。
`test/test_slam_config.cpp` 没有该字段的守护测试，清理动作本身无回归保护。

### 3.2 `corrupt-manifest` 的中心契约泄漏 `TypeError`

`test/integration/stability_common.py:406-411` 只在 `fault_case != "corrupt-manifest"`
时要求中心；`corrupt-manifest` + `center_index=None` 会在 430 行
`for value in center_index` 抛 `TypeError: 'NoneType' object is not iterable`，
而不是文档化的 `DatasetError`。shell harness（`scripts/run_localization_stability.sh`）
对所有 case 都传中心，因此该缺陷只在直接 API 调用时暴露。

### 3.3 tracking 闸门粘性

`test/integration/localization_regression.py:99` 的 `_tracking_confirmed` 一旦置
`True` 永不复位（191-197 行只负责打开）。初始化结果 `is_tracking=false` 与后续
WARN/ERROR 都不能关闭闸门，`odom()`（134-140 行）可能把失去跟踪之后的位姿记成
"稳定参考位姿"。该路径当前没有任何单测。

### 3.4 地图身份是字节数，引用校验过弱

`test/integration/stability_common.py:775-814` 的引用身份混用
`map_bytes`（`GlobalMap.pcd` 大小）、`ground_map_bytes`、`sc_db_bytes` 与
`tiled_manifest_sha256`：同尺寸不同内容的两张地图会被当成同一身份，摘要缺失时还
返回 `None`（两边都 `None` 即通过）。引用绑定只在调用方记得传 fingerprint 时才
生效（`localization_regression.py:543,858`）。`load_tiled_fault_reference`
（869-901 行）对 `bag_seconds`/`offset_seconds`/`warmup_seconds` 只做 `float()`，
NaN、负值、`offset > bag` 都能通过；`_require_matrix4`（716-724 行）只查形状，
NaN/Inf、缩放、镜像、错误底行都能通过；字段缺失时抛 `KeyError` 而非 `DatasetError`。

### 3.5 参考位姿可能发给空气

`test/integration/localization_regression.py:764-769,820-827,666-689`：
`TiledFaultRun.tick` 到点直接调用 `publish()` 并立即置 `published=True`，从不检查
`/initialpose` 是否有 matched subscriber。没有订阅者时中间件直接丢弃消息，回放
仍然报告 "published"，故障注入实际上没有发生却会通过。

### 3.6 本轮新增的 flake8 E303

`test/test_tiled_map_integration.py:339`（2 个空行）与 `392-394`（3 个空行）是
`06a24272` 新增测试引入的 `E303 too many blank lines`；仓库 lint 基线（`E501`、
`E402`、`E203`）此前不含 `E303`。

## 4. 设计决策

### 4.1 配置校验：单一入口 + 守护测试

- 契约：`map_save.tiled_map.tile_size_m` 必须有限且 `> 0`，由 `LoadMappingConfig`
  中唯一一处 `RequirePositive` 保证。删除重复的 `RequireNonNegative`，修正
  `system.mapping_process_interval` 的缩进。
- 测试：`test_slam_config.cpp` 新增 `MappingRejectsNonPositiveOrNonFiniteTileSize`，
  覆盖 `0.0`、`-1.0`、`NaN` 拒绝与合法值接受。清理是纯重构（删的是死代码），
  因此该测试先以特征化方式锁定现状，再用"把 `RequirePositive` 换成 `RequireFinite`"
  的变异证明它确实会失败。

### 4.2 fault fixture 一律要求中心

- 契约：`prepare_tiled_fault_case` 对任何 fault case（含 `corrupt-manifest`）都要求
  `center_index`；`None` → `DatasetError`，且在创建目标目录之前抛出。
- `corrupt-manifest` 的中心仍参与 `declared` 校验：保证传入的是 manifest 声明过的
  tile，而不是任意坐标。CLI/shell 无需改动（已对所有 case 传中心）。

### 4.3 tracking 闸门去粘性

- 契约：每个"可解码"的目标诊断（level 可解析、时间有限）都重写闸门：
  `level == 0 && is_tracking == "true"` → 打开，其余（WARN/ERROR、`is_tracking`
  缺失/`false`/其他值）→ 关闭。level 不可解析或时间为 NaN 的样本仍按现状整体忽略。
- `captured_pose()` 继续返回第一个满足闸门的合法 odom 样本。健康运行中 OK 诊断
  持续携带 `is_tracking=true`（`src/slam/system.cpp:923`），闸门保持打开，无行为回归。

### 4.4 地图身份：具名 artifact 的 SHA-256

- 唯一身份结构：`DatasetIdentity.map_artifacts: Mapping[str, str]`，键是相对地图根
  目录的规范路径，而不是本机路径或字节数：
  - `GlobalMap.pcd`、`GroundMap.pcd`、`sc_db.bin`（复用 `_REQUIRED_MAP_FILES`）；
  - `tiled/manifest.yaml`（仅当传入 `tiled_dir`）。
- `validate_dataset` 计算摘要（1 MiB 分块流式读取）；`_dataset_identity_payload`
  输出 `artifacts` 映射；摘要缺失/不可读 → `DatasetError`，不再返回 `None`。
- 删除 `map_fingerprint()` 与 `map_fingerprint_value` 参数：引用绑定成为
  `DatasetIdentity` 的固定部分，不再依赖调用方是否记得传指纹。
- `schema_version` 保持 `1`，不写迁移代码：旧引用没有 `artifacts` 键，
  `recorded.get("artifacts") != expected` 直接触发
  `"...recorded against a different dataset (...); recapture it"`，自然 fail-closed。
- 兼容性：`scripts/run_localization_stability.sh` 的 `tiled-faults` 每次运行都会
  `--record-reference` 重新录制，无需额外迁移；fixture 的 `manifest.yaml` 是逐字节
  复制（`prepare_tiled_fault_case` 已如此），而 `case_root` 用软链接指向真实地图
  产物，摘要按内容比较，两者一致。

### 4.5 引用位姿与时间字段校验收紧

`make_tiled_fault_reference` 与 `load_tiled_fault_reference` 使用同一套校验：

- `bag_seconds`/`offset_seconds` 必须有限且 `>= 0`；`warmup_seconds` 必须有限且
  `> 0`；并要求 `offset_seconds <= bag_seconds`（否则 `reference_offset` 会退化为 0，
  参考位姿被提前发布）。
- `map_T_base` 必须是刚体：4x4、全部有限、底行 `[0,0,0,1]`、旋转正交、`det = +1`
  （容差 `1e-6`）。
- 非数值、布尔、字符串、缺失字段一律 `DatasetError`，不再泄漏
  `ValueError`/`KeyError`/`TypeError`。

### 4.6 `/initialpose`：matched subscriber 闸门

- 发布回调返回 `bool`：`initial_pose_publisher.get_subscription_count() > 0` 时
  `publish()` 并返回 `True`，否则返回 `False`。回调内不 `sleep`、不调用阻塞式
  `wait_for_all_acked()`，因此不会卡住 executor。
- `TiledFaultRun.tick` 只在回调显式返回 `True` 时置 `published`；否则返回
  `"waiting-for-subscriber"`，由下一个 tick（`_spin_bag_window` 每轮最多 0.1 s）
  重试。等待发生在 tick 之间，`/clock` 回调继续被 executor 处理。
- 参考窗口（`reference_offset + tolerance`）内始终没有 matched subscriber →
  `ReplayError` 明确报"没有 `/initialpose` 订阅者匹配"，而不是含混的"错过参考时刻"；
  运行结束时仍未发布，保留既有 `ReplayError`。

## 5. 验收标准

| 项 | 命令 | 预期 |
| --- | --- | --- |
| 单测 | `PYTHONPATH=test/integration python3 test/test_localization_regression.py` 等 4 个改动文件 + `test_slam_config` | 全部 OK |
| 串行干净构建 | `colcon --log-base /tmp/p22_hardening/log build --symlink-install --packages-select lio_sam_hesai --build-base /tmp/p22_hardening/build --install-base /tmp/p22_hardening/install --cmake-args -DBUILD_TESTING=ON --executor sequential --parallel-workers 1` | 退出码 0，约 28 分钟 |
| 44 项约定 CTest | `ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai -j1 --output-on-failure -E '^(copyright\|cpplint\|flake8\|lint_cmake\|uncrustify\|xmllint)$'` | 44/44 通过（50 − 6 = 44） |
| `tiled-faults` | `LIO_SAM_WORKSPACE=/tmp/p22_hardening scripts/run_localization_stability.sh tiled-faults` | 退出码 0，三用例通过 |
| E303 | `python3 -m flake8 --select=E303 test/` | 无输出，退出码 0 |

完整 `ctest`（不加 `-E`）预期 46/50：失败项恰为既有的 `flake8`、`lint_cmake`、
`uncrustify`、`xmllint`，与本轮改动无关。

## 6. 风险与缓解

- **摘要成本**：测试地图约 5.5 MiB，生产地图为一次性顺序读取，发生在回放启动宽限内；
  分块读取避免整文件驻留内存。
- **订阅者发现时延**：`get_subscription_count()` 是本地非阻塞查询；最坏 0.1 s 的
  重试间隔远小于 1.0 s 参考容差。
- **失败模式**：所有新校验 fail-closed（不确定即拒绝），不会产生新的假通过；唯一的
  行为变化是旧 reference 必须重录。
