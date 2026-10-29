# 在线 DUFOMap 与地图保存

在线 DUFOMap 在 `mapOptimization` 内以后台 worker 运行，直接调用编译进来的
C++20 官方 DUFOMap/UFOMap 适配器，不启动 Python 子进程。worker 独占算法状态；
实时 SLAM 的匹配、关键帧、回环和原始关键帧点云不受影响。生产保存不再有
离线 DUFOMap 导出 stage，也不再生成 `dufomap_export/` 数据集。旧的
`scripts/dufomap_native`、安装器和离线重放脚本仅作为 legacy 手动工具保留。

在线清理是否真正改善某一段用户 bag 的动态点效果，仍取决于场景、标定和
`dufomap.*` 参数；这里只描述集成与保存契约，不承诺实车数据质量已经改善。

## 配置

```yaml
dufomap:
  enable: true
  resolution: 0.1
  d_s: 0.2
  d_p: 1
  num_threads: 2
  min_range: 0.2
  max_range: 100.0
  cache_directory: /home/robot/slam_ws/keyframe_cache
  temporal_guard:
    enable: true
    coarse_resolution: 0.4
    min_keyframe_support: 2
static_map_filter:
  enable: false
```

- `enable` 是建图会话的在线开关，不是每次保存的开关。关闭时保存和 RViz
  都走原始 keyframe 地图。
- `dufomap.enable=true` 与 `static_map_filter.enable=true` 互斥，配置校验会
  在启动时拒绝组合，避免把 DUFOMap 所需的原始坐标提前合并成体素质心。
- `num_threads` 只能为 0（自动 2–8）或 >=2；官方单线程射线 miss 分支仍是
  上游 TODO。`d_p` 范围 0–4。保存等待使用有界 barrier 超时；worker 生命周期
  不设固定时长，避免建图约一小时后自行停止。
- `temporal_guard` 增加一个仅用于补删的辅助粗网格：fine 标签仍优先；
  coarse 判定为 free 且该粗体素的不同完整关键帧支持不足
  `min_keyframe_support` 时才删除。`coarse_resolution` 必须有限且大于 fine
  `resolution`。C++ 默认关闭以兼容不维护完整缓存的调用方，mapping.yaml 默认
  开启。新增关键帧只增量读取新扫描；位姿 generation 改变时重新计算 support，
  防止微小位姿变化跨越体素边界后误删。小修正复用 fine/coarse 射线证据，
  大模型 epoch 则同时重建射线证据和 support。
  任一当前关键帧缺少可读的完整扫描时，粗网格补删保守关闭，fine 不变；
  路径损坏会记录独立诊断但不会使 worker 永久失败。
  保存还会校验 support revision，拒绝混用更新前后的缓存路径与保护统计。
  RViz 可继续显示已完成分类的历史前缀；纯追加关键帧不会清空这份结果，
  避免历史分类速度低于关键帧追加速度时长期停止更新。修改已覆盖关键帧的
  扫描或位姿仍会使对应显示失效。保存使用独立的严格快照，等待最新完整
  support revision，不直接复用落后的显示前缀。
- `cache_directory` 是持久缓存根路径。每次建图创建独立的
  `session_<pid>_<stamp>` 目录，退出后保留，不自动删除，因此多次建图会持续
  增长；需要时由用户自行清理旧 session。外部设置 `dufomap_export.*` 旧参数
  会直接被拒绝。

## 在线流程

1. 保存关键帧时把 corner/surface 点云复制给 worker 作为分类目标；SLAM 自己
   的原始关键帧点云保留给优化、回环和地面导出。
2. 每个已配准扫描把完整 deskewed 点云作为射线证据入队。关键帧自身的
   `anchor_T_scan` 为相对锚点的精确变换；非关键帧扫描使用
   `inv(最新KF_map_pose) * CurrentLidarMapPose`，不再用原始 IMU 差替换扫描
   匹配结果。
3. 被 `mapping_process_interval` 跳过的扫描没有新的扫描匹配结果。它只在
   最近一次已配准帧的锚点、`anchor_T_scan` 和 IMU initial guess 都可靠时，
   用 `anchor_T_processed * inv(processed_guess) * current_guess` 外推；矩阵
   非有限、旋转不合法或增量过大都会被丢弃，而不是写入错误坐标。
4. 回环/GPS 修正关键帧 pose 后，每个实际 pose 变化都推进保存用的 pose
   generation 并请求重新分类，避免旧变换配新地图点。只有相对上次“有意义
   重建”的固定参考 pose，其平移范数加
   `(max_range + 最大扫描锚点平移距离) * 旋转矩阵差的谱范数上界`
   的端点位移界超过 `min(resolution, d_s) / 4` 时，worker 才推进
   evidence/model epoch 并重新积分缓存的全部完整扫描；`d_s=0` 时容差为 0，
   `max_range=0` 时任何非零旋转都触发重建。小修正不会重置参考 pose，因此
   多次微修正会累计，但不会因每次微小数值变化而重放全历史。原始关键帧仍然
   供 SLAM 优化和回环使用。
5. worker 自己的会话缓存位于 `<session>/online`；每轮最多转换/积分 8 个
   扫描，避免一次保存等待整个积压队列。队列满、跳过、拒绝和 worker
   错误有限频记录，节点保持可观测。小修正期间在途积分可能使用略早于最新
   pose 的锚点。保留模型与最新 pose 分别受固定参考容差约束，两者间的近似
   偏差最多为两倍容差（当前参数下为 5 cm，不超过半个体素和半个命中膨胀
   距离）；分类和导出仍使用请求捕获的
   精确 pose generation。该近似不改变输出点云分辨率。

### 完整关键帧扫描缓存

`dufomap.enable=true` 时，每个新关键帧还会把完整的 deskewed
`CloudInfo.cloud_export_scan` 写入独立的
`<session>/keyframe_scans/<index>.bin`。该缓存：

- 与地面的 RingPlane `scans/*.pcd`、Patchwork++ `ground_scans/*.pcd` 完全分离，
  不复用、不覆盖，也绝不作为缺失缓存的回退；
- 文件内含 magic、版本、关键帧 index、点数，并按精确文件大小校验；重命名、
  截断、损坏或含非有限点的文件会被拒绝；
- 每个关键帧一个文件，先写临时文件再原子 rename；只依赖 keyframe index，
  因此地面模式、证据队列丢帧都不会驱逐它；
- 在释放 mapping 主锁后写入；保存会先等待 pending 写入，再等待 barrier，
  最后在 worker 内串行执行一次完整扫描导出。写入失败会保留明确错误，
  不会退回特征点云做 GlobalMap。

## 保存契约

保存服务 `/lio_sam/save_map_options` 只有 `destination`、`resolution`、
`ground_resolution`、`save_ground_map`、`save_tiled_map`，没有
`use_dufomap`。在线清理由 `dufomap.enable` 决定。

一次保存的执行顺序：

1. 保存互斥量保证同一时刻只有一个保存查询 worker。在映射锁内冻结
   keyframe 点云/最终 poses，并在同一临界区捕获
   `DufomapSnapshotRequest`（keyframe 数量、pose generation、evidence
   sequence）。
2. 释放映射锁后先等待完整的 `keyframe_scans` pending 写入，再等待该请求的
   精确 generation。worker 失败、等待超时或 generation 在冻结后前进都使
   保存失败，不能把新标签套到旧 poses 上。保存超时或版本失配只拒绝本次
   请求，不会停止后续在线分类。
3. 在任何旧 artifact 删除前，批量检查快照覆盖所有冻结关键帧，且每个
   corner/surface 标签长度与对应点云完全一致。标签缺失或长度不一致会失败，避免错误地写出空地图；正常分类产生的
   空角点/面点文件允许保存。
4. 同样在旧 artifact 删除前，worker 按本次 generation 的最终关键帧 pose
   读取每个完整扫描、变换到 map 坐标系并通过同一个 temporal-guard helper
   分类，只返回 label=0 的点。RViz 特征标签、完整扫描导出和 fallback 特征
   标签共用该 helper，避免策略分叉。缺失/损坏缓存、分类失败或导出期间
   generation 前进只让本次保存失败，不写入 worker 的永久错误，也不影响在线
   分类。完整扫描结果与 corner/surface 标签配对返回，使用同一代位姿和同一批
   已积分的空间证据，避免不同地图使用不同时间的判断。
5. 只有上述检查通过后才清理并写入轨迹、corner/surface、global、地面和
   可选 tiled 产物。会话扫描缓存和在线 worker 缓存不因保存成功或失败删除。

`GlobalMap.pcd` 的语义随 `dufomap.enable` 切换：

- 开启：`GlobalMap.pcd` 是上面完整扫描过滤后的 map-frame 点云，先应用保存
  Z 偏移，再按 `resolution` 下采样；不包含 corner/surface 特征拼接。
- 关闭：保持 legacy 的 corner+surface 拼接路径。
- `CornerMap.pcd` / `SurfMap.pcd` 始终来自 feature label 流程；地面、Scan
  Context、轨迹、transformations 和 tiled 产物继续使用同一冻结快照。

地面导出完全独立：原 ground 分割、补点、下采样、Z 偏移/flatten 逻辑不变，
独立的 `GroundMap.pcd` 不套用 DUFOMap 标签。完整扫描中的自然地面回波仍参与
射线证据计算，并在生成完整 `GlobalMap.pcd` 时和其他点一起查询、过滤。
RingPlane/Patchwork++ 所需的关键帧输入缓存保留。

RViz 默认显示清理后的 `/lio_sam/mapping/map_global`（`mapping_global`）。
原始配准点云 `/lio_sam/mapping/cloud_registered` 对应
`mapping_cloud_registered_raw`，默认关闭，避免动态原始点覆盖清理图；它本身
不是清理结果。清理后的特征地图按原有发布周期更新，回环后要等 generation 重建完成
才会切换。新关键帧尚未分类时继续发布已分类的前缀；标签缺失或版本失配时
保留上一张清理图，不发布原始图冒充清理结果。

升级后需要重启建图节点，新的建图会话才会生成 `keyframe_scans/` 缓存。
旧会话和旧 `dufomap_export/scans/*.pcd` 不会自动转换成这套完整关键帧缓存。
没有对应缓存时保存会明确失败，不会以特征地图冒充完整地图。

## 服务示例

```bash
ros2 service call /lio_sam/save_map_options lio_sam_hesai/srv/SaveMapOptions \
  "{destination: '/tmp/lio_sam_clean_map', resolution: 0.0, ground_resolution: 0.0, save_ground_map: true, save_tiled_map: false}"
```

服务字段变化后必须重新构建节点、RViz 插件和所有外部调用端，不能混用旧生成
的接口类型。RViz 保存对话框只保留 ground/tiled 复选框；在线 DUFOMap 状态由
mapping 节点的 `dufomap.enable` 配置体现。
