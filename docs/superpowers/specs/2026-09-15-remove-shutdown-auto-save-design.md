# 删除 Ctrl+C 自动保存路径设计

日期：2026-09-15
状态：待用户审阅

## 背景

建图节点当前在 `visualizeGlobalMapThread()` 因 ROS 关闭而退出循环后，根据
`map_save.enabled`（legacy：`savePCD`）执行一次自动导出。该路径写出
`cloudCorner.pcd`、`cloudSurf.pcd`、`cloudGlobal.pcd`、`cloudGround.pcd`，不生成
`sc_db.bin`，产物名称也与定位和分块生成器消费的地图契约不同。因此这套产物没有
回读方，而且 Ctrl+C 会触发耗时、隐式且可能覆盖目录的文件操作。

显式服务 `/lio_sam/save_map` 已能导出规范的 `CornerMap.pcd`、`SurfMap.pcd`、
`GlobalMap.pcd`、`GroundMap.pcd`、可选 `sc_db.bin` 和分块地图，应成为唯一地图导出入口。

## 目标

- Ctrl+C 只负责停止 ROS、结束工作线程和退出进程，不再导出地图或修改导出目录。
- 保留 `/lio_sam/save_map` 的现有行为和其余 `map_save.*` 配置。
- 完整移除只服务于退出自动保存的代码、参数和磁盘文件名。
- 对仍使用 `map_save.enabled` 或 `savePCD` 的旧配置启动失败，并给出迁移到显式保存服务的明确错误。

## 非目标

- 不修改 `/lio_sam/save_map` 的请求、响应、目录覆盖语义或产物内容。
- 不重命名规范地图产物，不实施 P2.3 版本化。
- 不处理 `TightCouplingKF` 或历史 lint 基线。

## 设计

### 1. 退出行为

删除 `visualizeGlobalMapThread()` 主循环之后的整段自动保存逻辑。线程观察到
`rclcpp::ok()` 为 false 后直接返回。保留循环运行期间的全局地图可视化发布。

删除运行时成员 `save_map_enabled_` 及构造函数中根据 `mapping_mode` 覆盖它的逻辑。
`mapping_mode=false` 时仍禁用建图保存服务，但日志只描述仍然存在的行为，不再声称关闭
退出自动保存。

### 2. 配置契约

从 `MapSaveConfig`、共享配置加载器和 `config/mapping.yaml` 删除
`map_save.enabled`，从 legacy 映射表删除 `savePCD`。

配置加载开始时检查原始参数 overrides。若 mapping 节点收到
`map_save.enabled` 或 `savePCD`，抛出配置错误，错误信息说明 Ctrl+C 自动保存已经删除，
并指向 `/lio_sam/save_map`。两个名称都采用 fail-closed，避免旧配置被静默忽略。

### 3. 地图产物布局

从 `MapArtifactLayout` 删除以下仅由退出路径使用的常量和访问器：

- `kSavedCornerCloudName` / `SavedCornerCloud()`
- `kSavedSurfaceCloudName` / `SavedSurfaceCloud()`
- `kSavedGlobalCloudName` / `SavedGlobalCloud()`
- `kSavedGroundCloudName` / `SavedGroundCloud()`

规范产物以及服务、定位、Scan Context 和分块地图使用的布局接口保持不变。

### 4. 文档与迁移

更新 `docs/PROJECT_STATUS.md`：将“退出自动保存产物收敛”标记为已解决，明确地图只能
通过 `/lio_sam/save_map` 显式保存，并记录两个被移除参数的启动失败行为。

## 错误处理

- 旧参数在节点完成启动前被拒绝，不允许以 no-op 形式继续运行。
- 错误信息同时包含被移除的参数名、行为变更和替代服务。
- 显式保存服务的现有错误处理不在本轮改变。

## 测试与验收

1. 配置单元测试分别证明 `map_save.enabled` 与 `savePCD` 会被明确拒绝，且错误指向
   `/lio_sam/save_map`。
2. 参数契约测试证明配置 YAML、共享 schema 与生产代码不再暴露两个旧参数。
3. `MapArtifactLayout` 测试和静态扫描证明四个 `Saved*Cloud` 接口及四个 `cloud*.pcd`
   文件名已删除。
4. 构建 `lio_sam_hesai` 并运行约定 CTest 门禁。
5. 在具备 ROS 2 和 `hesai_sim` 数据的环境中运行建图退出 smoke：指定空的临时导出目录，
   发送 Ctrl+C 后确认进程正常退出且目录未被创建或写入。该 smoke 的实际执行状态必须
   如实记录，不能用单元测试替代。

## 回滚

如必须恢复退出自动保存，应回滚本变更，而不是重新加入 no-op 参数。显式保存服务不会
因本轮改变，回滚不涉及地图格式迁移。
