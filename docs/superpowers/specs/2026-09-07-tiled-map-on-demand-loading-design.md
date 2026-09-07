# 大场景分块地图按需加载设计

更新日期：2026-09-07

## 1. 目标

为 `lio_sam_hesai` 增加可用于生产部署的分块地图模式，使定位节点的
常驻地图内存由“随整张地图增长”变为“受活动范围和缓存预算约束”。新模式必须
支持冷启动 Scan Context 自动重定位、正常跟踪、地面高度查询和现有可视化，且
不得破坏当前 `GlobalMap.pcd + GroundMap.pcd + sc_db.bin` 地图目录与配置。

本设计对应交接路线中的 P2-1。P2-2 的建图/重定位坐标与命名统一、P2-3 的
完整地图产物版本与兼容协议分别立项；本项目只定义分块加载所必需的最小 manifest
版本和一致性检查。

## 2. 当前问题

仓库已有 `SplitMap` 和 `Localization::LoadLocalMap()` 的 tile 分支，但不能作为
受支持能力使用：

- `Localization::use_tile_map_` 固定为 `false`，没有参数入口；
- tile 目录固定为源码树中的 `data/tile_map/`；
- `PrepareMap()` 无论模式都会加载完整全局地图并建立全图 XY kd-tree；
- `LoadCandidateLocalMap()` 明确拒绝 tile 模式，因此自动重定位不可用；
- tile 索引文本没有格式版本、坐标原点、点类型或产物一致性信息；
- 同步读取、静态局部变量和无预算缓存不能满足实时性和内存上限要求；
- 地面地图和全局可视化仍依赖完整 PCD 常驻。

因此，简单打开已有布尔值既不能降低启动内存，也会破坏自动重定位。

## 3. 范围

### 3.1 本项目包含

1. 向后兼容的 `auto/global/tiled` 地图模式。
2. 可重复生成的分块地图目录及最小 manifest schema。
3. 全局点云与地面点云的离线切片。
4. 后台读取、预取、缓存预算、LRU 淘汰和活动 tile 固定。
5. 冷启动自动重定位和手动初始位姿的候选局部地图加载。
6. 跟踪期间局部地图的安全切换。
7. 低分辨率全局 overview 与活动局部地图发布。
8. 分块加载诊断、单元测试和真实 `hesai_sim2` 回放验收。

### 3.2 本项目不包含

- 删除旧全图加载路径或迁移现有地图目录；
- 统一建图和重定位的全部坐标、话题与参数名称；
- 为所有地图产物设计最终签名、升级和跨版本迁移协议；
- 自定义压缩点云格式、内存映射 PCD 或远程对象存储；
- 根据云端服务动态下载 tile；
- 修改定位算法、匹配器数学模型或健康状态机语义。

## 4. 兼容行为与配置

`localization.map_mode` 接受以下值：

- `auto`：默认值。`localization.map_path` 指向普通文件时使用全图模式；指向
  包含 `manifest.yaml` 的目录时使用分块模式。
- `global`：强制使用现有全图路径；`map_path` 必须是可读取的 PCD 文件。
- `tiled`：强制使用分块目录；manifest 不存在或无效时拒绝地图就绪。

旧 YAML 未设置 `map_mode` 且 `map_path` 指向 `GlobalMap.pcd`，行为必须与当前
版本一致。不得隐式转换、移动或删除旧地图文件。

新增默认配置：

```yaml
localization:
  map_mode: "auto"
  tiled_map:
    active_radius_tiles: 1
    prefetch_radius_tiles: 2
    max_cached_tiles: 25
    max_cache_mib: 1024
    load_timeout_ms: 1000

map_save:
  save_tiled_map: false
  tiled_map:
    tile_size_m: 100.0
    output_subdirectory: "tiled"
    overview_resolution_m: 5.0
```

`map_save.save_tiled_map` 默认关闭。只有显式启用后，保存地图流程才额外生成分块
目录；现有 PCD 和 Scan Context 产物继续生成。

配置约束：

- `active_radius_tiles >= 0`；默认 1，对应 3×3 活动区；
- `prefetch_radius_tiles >= active_radius_tiles`；默认 2，对应 5×5 预取区；
- `max_cached_tiles` 必须至少容纳完整活动区；默认 25；
- `max_cache_mib` 必须大于零，并在启动及每次缓存提交前校验；
- `tile_size_m`、`overview_resolution_m` 和 `load_timeout_ms` 必须大于零。

## 5. 分块地图目录

分块目录结构为：

```text
tiled/
├── manifest.yaml
├── overview.pcd
├── global/
│   ├── -1_0.pcd
│   ├── 0_0.pcd
│   └── 1_0.pcd
└── ground/
    ├── -1_0.pcd
    ├── 0_0.pcd
    └── 1_0.pcd
```

`manifest.yaml` 的最小 schema 包含：

```yaml
schema_version: 1
frame_id: "map"
point_type: "XYZI"
tile_size_m: 100.0
origin_xy: [0.0, 0.0]
source:
  global_map_name: "GlobalMap.pcd"
  global_map_bytes: 2855470
  ground_map_name: "GroundMap.pcd"
  ground_map_bytes: 1367788
tiles:
  global:
    - index: [-1, 0]
      path: "global/-1_0.pcd"
      point_count: 12345
      bytes: 234567
  ground:
    - index: [-1, 0]
      path: "ground/-1_0.pcd"
      point_count: 2345
      bytes: 45678
overview:
  path: "overview.pcd"
  resolution_m: 5.0
```

`source` 只记录生成输入的文件名和大小，不作为运行时读取路径。本项目的 manifest
校验 tile/overview 文件存在性、相对路径不逃逸分块地图根目录、数值范围、重复
索引、声明大小和点数。最终密码学摘要、参数快照和跨版本迁移留给 P2-3，但字段
扩展必须遵循向后兼容规则：未知可选字段忽略，未知 `schema_version` 拒绝加载。

tile 索引由显式原点决定：

```text
tile_x = floor((x - origin_x) / tile_size_m)
tile_y = floor((y - origin_y) / tile_size_m)
```

生成器和运行时必须共享同一个经过单元测试的索引函数，尤其覆盖负坐标和边界点，
不得继续各自复制半格偏移公式。

生成过程写入临时目录，所有 PCD 与 manifest 成功落盘后再原子重命名为目标目录。
已有目标目录默认拒绝覆盖，避免中断时留下可被误加载的半成品地图。

## 6. 运行时结构

### 6.1 地图提供者边界

定位模块通过统一的局部地图提供者使用地图：

- `GlobalMapProvider` 封装现有全图加载、裁剪、候选地图和地面高度查询；
- `TiledMapProvider` 封装 manifest、后台读取、缓存、候选地图和 overview；
- `Localization` 只请求某个位姿附近的不可变地图快照，不拼接路径或直接管理
  tile 生命周期。

最小接口应覆盖准备、候选地图请求、跟踪地图请求、地面高度数据源、overview、
诊断快照和关闭。两个实现的返回类型和失败语义保持一致，为后续 P2-2 收敛两条
历史路径提供清晰边界，但本项目不重写匹配器。

### 6.2 缓存对象

缓存以一个 XY 网格坐标为条目；条目可以同时持有该坐标的 global 点云和可选
ground 点云。因此 `max_cached_tiles: 25` 表示最多 25 个网格坐标，而不是两类
文件各 25 个。每个缓存条目包含：

- manifest 索引；
- global 和可选 ground 只读点云共享指针；
- 实际内存估算；
- 最近访问序号；
- `loading/ready/failed` 状态和最近错误；
- 活动引用与在途请求引用。

缓存同时满足 `max_cached_tiles` 与 `max_cache_mib`。LRU 只能淘汰不属于活动集、
没有在途请求且 matcher 不再持有的条目。若完整活动集本身超过预算，地图提供者
返回明确的预算错误，不通过反复淘汰造成抖动。

### 6.3 后台加载

单个受控 worker 负责磁盘读取、PCD 验证和配置要求的降采样。相同 tile 的并发
请求合并为一个任务。任务按优先级处理：

1. 初始化或 Scan Context 候选所需中心和活动 tile；
2. 当前跟踪活动区新进入的 tile；
3. 运动方向和预取半径内的 tile。

定位线程不执行磁盘读取。worker 等到本次请求内的 tile 全部进入
`ready/failed/not-present` 终态后，才发布不可变快照；仍在读取的半成品集合不能
发布。中心 global tile 失败会使整个请求失败，非中心 global tile 失败可发布带
缺失清单的降级快照，ground tile 失败则使用第 7.4 节的回退。关闭节点时停止接收
新任务、取消预取并有界等待 worker 退出，不遗留线程或临时文件。

## 7. 定位数据流

### 7.1 启动准备

全图模式保持当前同步加载行为。分块模式的 `PrepareMap()` 只完成：

1. 解析并验证 manifest；
2. 建立 global/ground tile 元数据索引；
3. 加载 Scan Context DB；
4. 可选加载 `overview.pcd`；
5. 启动 tile loader worker。

分块模式不得创建完整 `global_map_cloud_ptr_`、完整全图 XY kd-tree 或完整地面图
kd-tree。由于初始位姿未知，启动就绪不要求预先选择活动 tile。

### 7.2 自动重定位

Scan Context 返回有序候选位姿后，对每个候选执行：

1. 将候选位置转换为中心 tile；
2. 请求以该 tile 为中心的活动区 global tile；
3. 在 `load_timeout_ms` 内等待高优先级请求完成；
4. 合并为仅属于该候选的只读局部地图并执行现有精配准；
5. 候选失败时释放固定引用，缓存条目仍可按 LRU 复用；
6. 候选成功时将相同 tile 集提升为跟踪活动集。

候选 tile 缺失、损坏或超时只淘汰该候选，继续评估后续候选。所有候选都不可用
时维持现有重定位失败语义。手动 `/initialpose` 走同一候选局部地图接口，不建立
第二套加载逻辑。

### 7.3 正常跟踪

活动半径 1 表示当前中心周围的 3×3 tile。预取半径 2 表示中心周围 5×5 tile；
已知速度方向时优先读取前方 tile，但不得减少完整活动区覆盖。

中心 tile 变化时，provider 在后台加载新的活动差集。旧地图继续供 matcher 使用，
直到新活动集合的所有请求都进入终态；随后在定位帧安全点一次性提交新快照并
调用现有 matcher 局部地图更新。快照可以明确标记非中心 tile 缺失并触发降级，
但不得包含仍在加载的半成品，也不得在帧处理中等待预取 I/O。

当前地图仍覆盖车辆位置但新邻居未及时完成时，保持旧地图并上报加载延迟。中心
tile 不可用且旧地图已经不覆盖当前位置时，报告不可用，由现有健康协调逻辑进入
`DEGRADED/LOST`，不能继续发布高置信定位结果。

### 7.4 地面高度

分块模式使用活动区对应的 ground tile 建立局部 XY 查询结构。ground tile 缺失时
可回退到同区域 global tile 的地面估计逻辑；中心区域两类数据都不可用时报告
高度约束不可用。不得为地面查询重新加载完整 `GroundMap.pcd`。

## 8. 可视化

地图生成器从 `GlobalMap.pcd` 生成默认 5 米分辨率的 `overview.pcd`。

- 全图模式的 `lio_sam/localization/global_map` 行为不变；
- 分块模式的 `lio_sam/localization/global_map` 发布 overview；
- 新增 `lio_sam/localization/local_map`，发布当前 matcher 实际使用的活动地图；
- overview 缺失只产生警告和诊断，不阻止定位就绪；
- 可视化关闭时不加载 overview，以进一步降低内存。

接口和 frame ID 保持不变，下游无需因地图模式切换话题。

## 9. 诊断与日志

现有 `/diagnostics` 增加以下键：

- `map_mode`：`global` 或 `tiled`；
- `map_manifest_version`；
- `map_active_tile_count`；
- `map_cached_tile_count`；
- `map_cache_mib`；
- `map_cache_hits`、`map_cache_misses`；
- `map_tile_load_ms`：最近一次完整请求耗时；
- `map_tile_load_failures`；
- `map_tile_load_state` 和最近失败原因。

重复的外围 tile 缺失或预取失败应限频记录，中心 tile、manifest、预算和候选加载
失败必须保留可定位到路径与索引的明确错误。诊断不得暴露完整用户目录之外的敏感
内容。

## 10. 错误处理

- `tiled` 模式下 manifest 缺失、schema 不支持或关键字段无效：拒绝地图就绪；
- `auto` 模式指向目录但 manifest 无效：拒绝启动，不静默退回同目录中的全图；
- 中心 tile 未在 manifest 中：该地图请求失败；
- manifest 声明 tile 存在但文件缺失、点数或大小不符：该 tile 标记失败；
- 中心 global tile 失败：地图请求失败；非中心 global tile 失败：请求可提交降级
  快照并列出缺失索引；ground tile 失败：按第 7.4 节回退；
- 新活动集加载超时：保留旧快照；初始化请求超时则本次初始化失败；
- 缓存预算无法容纳活动集：请求失败并报告实际需求与配置上限；
- overview 缺失或损坏：禁用全局可视化，不影响定位；
- worker 异常：隔离到请求结果并保持节点可控退出，不从后台线程终止进程。

## 11. 测试与验收

### 11.1 单元及契约测试

1. tile 索引覆盖原点、边界、正负坐标和大坐标。
2. 分片前后点数守恒，每个点只属于一个 tile，输出顺序与 manifest 可重复。
3. manifest 覆盖合法输入、未知版本、重复索引、路径逃逸、缺文件、大小及点数不符。
4. 缓存覆盖命中、请求合并、LRU、活动固定、双预算和预算不足。
5. 后台加载覆盖优先级、超时、失败隔离、关闭和无在途线程泄漏。
6. 地图提供者覆盖全图兼容、auto 选择、候选请求、跟踪切换和 overview 缺失。
7. fake loader 证明跟踪帧不直接执行磁盘读取，只有完整快照可以提交。
8. 配置契约证明旧 YAML 不修改即可继续使用全图模式。

### 11.2 集成验收

使用 `/home/robot/map/hesai_sim2` 和现有 bag：

1. 重新执行全图模式 180 秒 quick 回放，现有基线继续通过；
2. 生成 `hesai_sim2/tiled`，验证 manifest、点数和目录身份；
3. 在没有 `/initialpose` 的条件下以分块模式完成 Scan Context 自动重定位；
4. 执行分块模式 180 秒 quick 回放，要求 `error: null`、无 error sample、
   `valid_odom_ratio >= 0.9` 且 odom 时间戳单调；
5. 验证分块模式没有完整 GlobalMap/全图 kd-tree 常驻，活动及缓存数量不越界；
6. 对比全图与分块模式的启动 RSS、稳定 RSS、地图准备耗时和最大帧处理时间；
7. 删除外围 tile、删除中心 tile、损坏 manifest 并验证各自失败语义；
8. 回放跨 tile 轨迹，验证旧地图保持、完整快照切换和定位输出连续性。

分块模式只有在功能指标不低于现有 quick 基线、内存受配置约束且没有新增超期帧
时才可标记完成。实测结果和参数调整记录到交接文档。

## 12. 交付顺序

1. 固化共享 tile 索引和 manifest 契约。
2. 实现安全、可重复的地图分片与 overview 生成。
3. 实现有界异步缓存及其单元测试。
4. 引入地图提供者边界并保持全图回归通过。
5. 接通分块自动重定位、正常跟踪、地面查询和可视化。
6. 增加诊断、故障注入与真实数据验收。
7. 更新配置、运维说明和交接文档。

每一步保持全图模式可构建、可测试、可回滚；不得以关闭现有稳定性检查作为交付
条件。
