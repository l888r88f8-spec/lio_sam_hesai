# 地面分割执行模式

本项目的扫描线地面分割算法命名为 **RingPlane（扫描线平面分割）**：先利用相邻扫描线寻找地面种子，再拟合局部平面并检查几何连通性。其配置段为 `ring_plane`，另一种算法为官方 Patchwork++。旧建图配置 `ground_segmentation.*` 和原扁平参数作为兼容别名保留；同一参数以多个名称配置且值不一致时，会按项目原有规则明确报错。定位配置沿用原有参数命名。

两个算法各设置一个字符串参数，只接受 `"off"`、`"online"`、`"export"`。不支持 `both`。`"off"` 请加引号，避免 YAML 将其解析为布尔值。参数在节点启动时读取，修改后重启整套 mapping launch。

```yaml
ring_plane:
  mode: "online"
patchworkpp:
  mode: "export"
```

此例实时阶段使用 RingPlane，保存 GroundMap 时用官方 C++ Patchwork++ 从缓存关键帧重新分割。默认配置仍是 RingPlane `"online"`、Patchwork++ `"off"`。

`mode` 只决定分割算法的执行阶段。现有 `ring_plane.leaf_size`、`flatten_z`、`flatten_value`、`grid_patch` 仍用于相应的公共地面后处理；选择 Patchwork++ 不会绕过这些设置。

| RingPlane | Patchwork++ | 在线分割 | 导出时重新分割 |
|---|---|---|---|
| online | off | RingPlane | 无，保存已有在线地面 |
| online | export | RingPlane | Patchwork++ |
| export | online | Patchwork++ | RingPlane |
| online | online | RingPlane | 无，保存已有在线地面 |
| export | export | 无 | RingPlane |
| off | online | Patchwork++ | 无，保存已有在线地面 |
| off | export | 无 | Patchwork++ |
| export | off | 无 | RingPlane |
| off | off | 无 | 无 |

优先级在在线和导出阶段分别计算。同一阶段只运行胜出的算法，不重复计算或融合。没有在线算法时，地面输出为空，非地面输出保留原投影扫描；LOAM 特征输入仍保留完整投影点云。两个算法都关闭时不生成 GroundMap，仍可保存特征地图，不会伪造空地面文件。

## 点云与缓存

- RingPlane 保留相邻 ring 种子、RANSAC、平面距离、连通性检查和地面输出 ring 范围。在线与导出复用同一核心。导出缓存携带原投影格索引及网格行列数，不通过去畸变后的 XYZ 猜测 ring；两节点投影尺寸不一致时报错。
- Patchwork++ 使用单帧雷达局部坐标的完整 XYZI；保留投影抽稀前的有效测量，可做去畸变。它不使用角点/面点特征云或拼接后的 GlobalMap 作为扫描输入。
- 所有建图会话始终缓存选中关键帧的完整 deskewed 扫描；开启任一有效导出算法时，再按算法需要缓存投影格索引或完整地面扫描。原在线扫描特征和去畸变参考帧保持原流程。
- 导出先对每个关键帧分割，再按同一份最终优化位姿快照拼接地面。`map_save.ground_grid.enable: true` 时，最终 GroundMap 以 `ground_resolution` 为 XY 格距，每格最多输出一个保留坡度的格心点，再应用保存偏移和显式压平选项；关闭时保留原有补点及三维下采样流程。单帧分割本身保留原始测量点。详见 [GroundMap 网格导出](ground-map-grid.md)。
- `save_ground_map: false` 时跳过导出地面分割；只选择算法不会强制保存 GroundMap。保存服务遵循此规则。
- 缓存缺失、损坏或算法失败会报告错误，不会静默切换算法。导出分割失败时不替换旧最终地图文件。

离线分割减少实时阶段的分割工作，但按需准备、传输和写入缓存仍有开销。仅缓存关键帧，不缓存每帧到磁盘。旧会话缺少所选算法所需的缓存时，需要重新采集或回放原始 bag。

扫描缓存目录使用 `dufomap.cache_directory`，每次建图在根路径下创建独立 session，具体路径见节点日志；留空使用系统临时目录。所有会话退出时都保留缓存，不自动删除，因此会随建图次数持续增长。在线 `dufomap.enable` 只决定是否启动清理 worker，不影响地面导出所需的扫描/投影缓存。导出前的扫描或算法失败保护旧最终地图；最终 PCD 写盘仍沿用现有流程，并非所有文件的原子事务。

## Patchwork++ 参数与来源

使用官方 C++ 核心固定版本 `3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`，不依赖 Python 包。许可证和源码来源见 `third_party/patchworkpp/`。

`patchworkpp.sensor_height` 是雷达原点距地高度；配置中的 `0.68` 米参考现有保存偏移，请按实际安装标定。`min_range`、`max_range` 是算法自己的范围门限；`uprightness_thr` 越大越严格；`th_seeds` 与 `th_dist` 控制种子和局部平面容差。

RNR 默认关闭，因为不同雷达强度量程不同。启用 `enable_rnr` 前需校准 `rnr_intensity_threshold`；该模块要求有限强度值。几何分割无需 ring 或逐点时间。

DUFOMap 仍只清理 corner/surface。地面分割和 DUFOMap 动态清理是独立流程，GroundMap 不会成为 DUFOMap 分类目标。
