# 子项目 A 收尾：定位 TF 连续性设计

日期：2026-08-03
状态：已实施并验证

## 目标

在已经获得过一次有效定位后，即使重定位节点暂时没有新的
`LocalizationResult`，也持续发布 `map -> base_link`。输入只依赖现有的雷达和
IMU；不要求 bag 或运行环境提供外部位姿、轮速计或 GNSS。

该设计补齐子项目 A 的 TF 降级契约，并与子项目 E 的自动重定位闭环配合：E
决定何时请求恢复，TF 连续模块只保证恢复期间下游仍能查询变换，并明确其可信度。

## 范围

做：

- 成功定位后保存一个线程安全的、仅用于 TF 外推的导航状态锚点。
- 在没有新定位结果但 IMU 仍健康时，基于锚点和已有 IMU 缓存进行短时外推。
- 没有可用 IMU 或外推超过安全时限时，按当前 ROS 时间重发最后锚定变换，绝不
  伪造运动。
- 在 `/diagnostics` 中区分测量、IMU 外推和保持三种 TF 发布模式。
- 用现有纯雷达 + IMU bag 验证雷达断流、双流断流和子项目 E 的恢复行为。

不做：

- 不引入外部位姿、轮速计或 GNSS。
- 不把外推/保持结果写回 `Localization`、路径或 odometry；它们不是新的配准结果。
- 不改变子项目 E 的 "进入 LOST 一次、数据流恢复再一次" 重试闸门。
- 不在 IMU 已过期时无界积分或声明位姿准确。

## 当前缺口

`System::Run()` 在没有新 `LocalizationResult` 时只调用
`PublishLocalizationHealth()`；`PublishTF()` 位于
`ProcessLocalizationResultCache()` 的新结果路径。因此诊断可转为 `LOST`，但 TF
停止发布。

## 架构与数据边界

```
Localization worker
  已提交的 NavStateData + map->lidar 变换
                         │  线程安全快照
                         ▼
System 定时器 ─ IMUDataSearcher.GetDataSegment() ─► Localization TF 预测器
                                                        │
                  measured / extrapolated / held ◄─────┘
                                                        │
                                                        ▼
                                           map -> base_link TF + diagnostics
```

### 定位模块：只读预测器

`Localization` 新增一个仅用于输出的线程安全快照，包含最后一次**已提交**的
`NavStateData`、其传感器时间戳和地图到雷达的转换所需标定。它只在初始化或正常
跟踪结果真正提交后更新；超时丢帧、失败配准和自动重定位候选都不得覆盖它。

新增只读预测接口接收一个按时间排序的 `IMUData` 段：

- 复制锚点，而不读取或修改配准线程正在使用的 `PreIntegration`；
- 以锚点的 gyro/accel bias 创建临时预积分器，积分该 IMU 段并预测导航状态；
- 复用既有 map-IMU 到 map-lidar 外参转换，返回预测的 `map -> lidar` 位姿及其
  传感器时间戳；
- 输入不连续、少于两个 IMU 样本、时间戳不前进或预测失败时返回空结果。

预测器不持有 ROS 发布器，不改变 `last_nav_state_`、匹配器、预积分器或重定位状态。

### System：发布选择器

每次 `System::Run()`：

1. 有新定位结果时，保留现有 measured TF、path 和 odometry 发布路径。
2. 没有新结果且存在 TF 锚点时，以锚点时间和 IMU 缓存最新时间取 IMU 段。
3. IMU 段有效且锚点年龄不超过 `localization_tf.max_extrapolation_seconds`
   （默认 `0.5` 秒）时，发布预测的 `map -> base_link`，模式为 `extrapolated`。
4. 没有有效 IMU 段、IMU 已断流或锚点年龄超过该上限时，发布最后锚定的
   `map -> base_link`，以当前 ROS 时间戳标记，模式为 `held`。
5. 初次成功定位前没有地图坐标锚点，模式为 `no_anchor`，不发布伪造的
   `map -> base_link`。

发布频率由 `localization_tf.publish_rate_hz` 控制，默认 `20 Hz`。选择器在
`System` 定时器内节流，避免 10 ms 健康定时器无条件以 100 Hz 广播重复 TF。

外推或保持只发送 TF；不发布 path、点云或 odometry，避免下游将非测量结果误判为
新的定位更新。下一次有效 `LocalizationResult` 立即恢复 measured 模式并更新锚点。

### 诊断与安全降级

既有健康 level、协方差和子项目 E 的 `auto_relocalization_trigger` 保持不变。健康
诊断额外包含：

- `tf_mode`：`measured`、`extrapolated`、`held` 或 `no_anchor`；
- `tf_anchor_age_ms`：锚点相对当前 ROS 时间的年龄；
- `tf_extrapolation_age_ms`：本次 IMU 外推跨度，保持模式为 `0`；
- `tf_hold_reason`：`imu_unavailable`、`extrapolation_limit` 或 `no_anchor`。

进入 `LOST` 不会降低 TF 发布频率。`held` 显式标记变换已陈旧；既有状态机仍负责
在 `LOST` 时发布 ERROR 与高协方差。它只维护 TF 图连通性，不表示车辆静止或定位
有效。

## 参数

新增 `localization_tf` 参数组：

- `publish_rate_hz: 20.0`，必须为正数；
- `max_extrapolation_seconds: 0.5`，必须为正数，且不得大于
  `localization_health.stream_timeout`；
- `enable_imu_extrapolation: true`。关闭时仍按频率发布 `held` TF，不积分 IMU。

启动期验证所有数值；错误配置直接拒绝启动。现有话题名、frame 名和 TF 外参不变。

## 与子项目 E 的关系

E 的 `RecoveryAttemptGate` 在进入 `LOST` 时投递一次自动重定位请求，并仅在之后
观测到数据流异常再恢复时投递一次。TF 连续模块不触发、不抑制也不确认这些请求；
它通过 `tf_mode` 与既有 `auto_relocalization_trigger` 让现场能同时观察两者。

## 测试与验收

### 自动化

1. 为发布选择逻辑添加纯单元测试：无锚点不发布、有效 IMU 段选
   `extrapolated`、缺 IMU/超限选 `held`、新 measured 结果覆盖旧锚点、发布节流。
2. 为 `Localization` 预测快照添加测试：只使用已提交状态和提供的 IMU 段；预测不
   改变配准预积分器或锚点；bias 与锚点一致。
3. 为 System 表面/集成测试断言：无新结果路径仍能调用 TF 选择器，但不会发布新的
   odometry 或 path；诊断包含上述 TF 键。

### rosbag2/hesai_sim 回放

无需外部位姿话题。使用已有 `/lidar_points` 和 `/imu`：

1. 先等一次有效定位，再人为屏蔽雷达而保留 IMU。断言 `map -> base_link` 以至少
   `publish_rate_hz` 的速率持续、时间戳单调，诊断变为 `extrapolated`/`LOST`；E 的
   首次 LOST 请求最多一次。
2. 随后恢复雷达和 IMU。断言 E 仅在恢复边沿追加一次请求，下一次有效定位恢复
   `measured` 模式。
3. 同时屏蔽雷达和 IMU。断言 TF 仍按频率以 `held` 模式出现，diagnostics 为 ERROR，
   不产生新的 odometry/path 或虚假平移。

没有真值位姿时，验收连续性、时间戳、模式、重试计数和恢复路径；不以该 bag 评价
IMU 外推的绝对轨迹精度。

## 验收标准

- 首次有效定位后，任何健康状态下均保持 `map -> base_link` 可查询。
- 只在 IMU 段完整且外推跨度不超过 0.5 秒时使用外推；否则保持最后锚定变换。
- 外推或保持不会改变定位算法状态，也不会伪装为 odometry/path 更新。
- 原有 E 重试策略与手动 `/initialpose` 语义不变。
- 新单元/集成测试、完整包测试和上述三段回放均通过。
