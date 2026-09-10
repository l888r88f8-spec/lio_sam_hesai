# P2.2 建图/重定位坐标语义与命名收敛设计

**日期：** 2026-09-10

**状态：** 已确认

## 1. 目标

在不改变建图、重定位算法行为和现有 TF 拓扑的前提下，收敛两条历史代码路径的
坐标语义、位姿方向命名、配置模型、ROS 话题分类和地图产物名称来源，降低后续修改
把矩阵乘反、给消息填写错误 frame 或让同一参数在两处产生不同含义的风险。

P2.2 完成后：

- 所有跨模块刚体变换使用 `parent_T_child` 语义；
- 建图与重定位共享同一份基础 frame、传感器话题和标定配置类型；
- ROS 消息与 TF 由公共适配器填写 frame 和 pose，不再在算法代码中重复拼装；
- 当前 ROS 话题字符串、地图文件名和两种模式的 TF 拓扑保持兼容；
- 旧参数键保留一个迁移周期，并在使用时打印一次性弃用告警；
- 重定位不得重新依赖或发布 odom TF。

## 2. 背景与问题

此前已完成外部 ROS 参数 snake_case 分组和静态外参安装量统一，但内部仍存在两套
历史实现：

- 建图节点继承 `ParamServer`，直接使用大量 camelCase 数据成员；
- 重定位由全局单例 `ConfigParameters` 提供配置；
- `LocalizationResult::map_pose` 实际表示 `map_T_lidar`；
- `System::T_base_to_lidar_` 实际缓存 `lidar_T_base`，而初始位姿路径中的同名局部变量
  又表示 `base_T_lidar`；
- 重定位的内部预积分状态使用 `odom` 命名，但它不是 ROS `odom` frame；
- 建图管线和可视化话题大量硬编码，`sensor_topic.odom_topic` 实际承载内部/输出
  里程计，`output_topic.mapping_odom_topic` 又被重定位用于全局定位结果；
- 地图文件名 `GlobalMap.pcd`、`GroundMap.pcd`、`sc_db.bin`、`tiled/manifest.yaml`
  分散为字符串字面量，另有 `kGlobalMapFileName = "map.pcd"` 的不一致默认值。

重定位曾经引入 `map -> odom -> base_link` 和外部 odom 闭环，之后因定位稳定性问题在
`fe61c5e` 中移除。P2.2 不恢复该路径。

## 3. 范围

### 3.1 包含

1. 建立有方向约束的公共刚体变换类型和统一命名规则。
2. 将跨模块位姿、定位结果、TF 与 ROS 消息边界迁移到强语义类型。
3. 建立不可变 `SlamConfig`，包含共享配置和建图/重定位专属配置。
4. 让建图与重定位使用同一参数加载与校验基础设施。
5. 分类并参数化传感器输入、内部管线、命令输入和对外输出话题。
6. 集中定义地图产物布局，消除文件名字面量和默认路径不一致。
7. 提供一个迁移周期的旧参数键兼容读取与弃用诊断。
8. 用静态契约、单元测试和 rosbag 回放证明行为未改变。

### 3.2 不包含

- 不改变建图与重定位的匹配、优化、预积分或回环算法。
- 不改变任何已配置 ROS 话题的实际字符串。
- 不改变地图磁盘文件名、tile manifest schema 或 Scan Context 二进制格式。
- 不改变静态外参数值。
- 不将重定位改为 `map -> odom -> base_link`。
- 不解决 P2.3 的地图版本、摘要、生成参数快照和跨版本迁移。
- 不顺带治理许可证或与本项目无关的 lint。

## 4. 坐标与 TF 契约

### 4.1 统一记法

`parent_T_child` 表示 child frame 在 parent frame 中的位姿，也表示把 child 坐标中的
点转换到 parent 坐标。组合只允许：

```text
A_T_B * B_T_C = A_T_C
inverse(A_T_B) = B_T_A
```

跨模块接口和持久状态禁止使用 `*_to_*` 或无 frame 语义的 `pose`、`map_pose`、
`odom_pose` 名称。算法函数内部短生命周期的雅可比、增量矩阵和求解器原生类型不在
此限制内。

### 4.2 强类型

新增零额外运行时状态的模板包装：

```cpp
template <typename ParentFrame, typename ChildFrame>
class RigidTransform3d;
```

它内部保存 `Eigen::Isometry3d`，只暴露方向安全的 `Inverse()`、`operator*`、
`Rotation()`、`Translation()` 和只读 `Matrix()`。frame tag 至少包括：

- `MapFrame`
- `OdomFrame`
- `BaseFrame`
- `ImuFrame`
- `LidarFrame`
- `LocalIntegrationFrame`

`LocalIntegrationFrame` 专用于重定位内部的相对预积分，避免把它误认为 ROS odom。
常用别名包括 `MapTLidar`、`MapTBase`、`OdomTLidar`、`OdomTBase`、
`BaseTLidar`、`LidarTBase`、`ImuTLidar` 和 `LocalTImu`。

### 4.3 保持现有 TF 拓扑

建图继续使用：

```text
map --静态单位变换--> odom --动态--> base_link
                                  |--> imu
                                  `--> lidar
```

重定位继续使用：

```text
map --动态--> base_link
              |--> imu
              `--> lidar
```

重定位的 `System` 只允许查找 base/lidar 静态外参并发布 `map -> base_link`；参数表、
代码和测试均不得重新加入 `frames.odometry`、外部 odom 订阅或 `map -> odom` 发布。

### 4.4 边界数据

- `LocalizationResult::map_pose` 改为 `map_T_lidar`，类型为 `MapTLidar`。
- `LocalizationResult::odom_pose` 改为 `local_T_imu`，仅用于内部预积分结果；不得进入
  ROS frame 或 TF。
- `TfPoseSample::map_base_pose` 改为 `map_T_base`，类型为 `MapTBase`。
- 初始位姿消息明确解释为 `map_T_base`，通过 `base_T_lidar` 转为 `map_T_lidar`。
- 重定位输出明确通过 `map_T_lidar * lidar_T_base = map_T_base` 生成。
- 建图优化内部保持既有数值表示，但离开优化器时转换成 `OdomTLidar`；TF 发布前再
  转为 `OdomTBase`。
- 标定 canonical 语义继续为 `imu_T_lidar`。建图需要的逆向旋转和杆臂只允许由公共
  `LidarImuCalibration` 派生。

## 5. 配置模型

### 5.1 类型结构

新增不可变配置聚合：

```text
SlamConfig
|-- CommonConfig
|   |-- FrameIds
|   |-- SensorTopics
|   |-- CalibrationConfig
|   `-- LidarImuConfig
|-- TopicConfig
|   |-- PipelineTopics
|   |-- CommandTopics
|   `-- OutputTopics
|-- MappingConfig
`-- LocalizationConfig
```

`System` 加载一次配置，并以 `shared_ptr<const SlamConfig>` 显式注入
`PreProcessing`、`Localization` 等对象。P2.2 结束时删除 `ConfigParameters`
单例。建图侧保留 `ParamServer` 作为 ROS Node 基类，但删除其平铺 camelCase 配置
成员，改为持有不可变 `SlamConfig`；这样无需同时重写四个节点的继承结构。

### 5.2 加载规则

公共加载器负责声明、读取、类型检查、范围检查和弃用处理：

1. canonical 参数存在时使用 canonical 值；
2. 只存在 legacy 参数时读取 legacy 值并对该键打印一次 WARN；
3. canonical 与 legacy 同时存在且值相同，使用 canonical 并打印一次 WARN；
4. 两者同时存在但值不同，拒绝启动并列出冲突键和值；
5. frame ID 为空、话题为空、外参尺寸错误、旋转非有限或非正交时拒绝启动。

兼容读取在 P2.2 发布周期默认开启；计划在 P2.3 验收后的独立兼容清理里删除，不能
在没有迁移通知和现场配置审计的情况下自动移除。

## 6. 话题命名与兼容

参数按职责分为四类：

| 分组 | 含义 | 示例 |
|---|---|---|
| `sensor_topic.*` | 设备或上游传感器输入 | `lidar_topic`, `imu_topic`, `gps_topic` |
| `pipeline_topic.*` | 包内节点间协议 | `cloud_info`, `feature_cloud_info`, `mapping_odometry`, `imu_odometry_incremental` |
| `command_topic.*` | 外部控制输入 | `initial_pose`, `loop_closure_hint` |
| `output_topic.*` | 面向下游的结果 | `odometry`, `path`, `global_map`, `local_map`, `registered_cloud`, `ground_map` |

既有 `sensor_topic.lidar_topic`、`sensor_topic.imu_topic`、`frames.*` 和
`calibration.*` 保持不变。以下误分类键迁移：

- `sensor_topic.odom_topic` -> `output_topic.odometry` 与
  `pipeline_topic.imu_odometry_incremental` 两个显式键；legacy 前缀仅在兼容读取中
  派生旧的 `_incremental` 名称。
- `output_topic.mapping_odom_topic` -> 重定位的 `output_topic.odometry`。
- 建图硬编码的 `lio_sam/mapping/odometry` -> `pipeline_topic.mapping_odometry`。
- `/initialpose` -> `command_topic.initial_pose`。

其余硬编码话题逐项进入对应分组。两份仓库 YAML 给出与当前代码完全相同的字符串，
因此默认运行时图不变。P2.2 不通过同一节点双发两份大点云；兼容发生在参数键层，
已有话题消费者无需修改。

每条 `Odometry`、`Path` 和点云的 header/child frame 契约集中登记。建图现有话题如果
携带 lidar 位姿，就在 `pipeline_topic` 中明确命名，不伪装成对外 base 位姿；当前
消息内容与 frame 值保持不变，语义修正后的新输出不在 P2.2 中额外引入。

## 7. 地图产物命名

新增 `MapArtifactLayout`，集中定义并解析现有布局：

```text
<map_root>/GlobalMap.pcd
<map_root>/GroundMap.pcd
<map_root>/sc_db.bin
<map_root>/trajectory.pcd
<map_root>/transformations.pcd
<map_root>/CornerMap.pcd
<map_root>/SurfMap.pcd
<map_root>/<tile_subdirectory>/manifest.yaml
```

P2.2 不重命名磁盘文件。保存器、全图加载器、分块生成器、Scan Context 加载器和测试
必须使用同一布局对象，不再自行拼接文件名。删除或修正与该布局冲突的
`kGlobalMapFileName = "map.pcd"` 默认。manifest 内 `frame_id` 继续取 `frames.map`；
`tile_subdirectory` 默认保持 `tiled`，并继续接受现有
`map_save.tiled_map.output_subdirectory` 配置；格式版本及摘要仍由 P2.3 负责。

## 8. 公共 ROS 适配器

新增 `RosPoseAdapter`，唯一负责：

- 强类型变换与 `geometry_msgs::msg::Pose` 的互转；
- 构造 `nav_msgs::msg::Odometry`；
- 构造 `geometry_msgs::msg::TransformStamped`；
- 为 `Path`、点云和 marker 填写配置中的 frame ID；
- 验证时间戳、平移和四元数均为有限值，四元数归一化。

错误 frame 组合在编译期失败；来自 ROS 的非有限位姿、零范数四元数和错误
`initialpose.header.frame_id` 在运行时拒绝，并输出节流告警。算法状态不得被无效消息
部分更新。

## 9. 数据流

重定位结果路径保持：

```text
matcher MapTLidar
  -> LocalizationResult.map_T_lidar
  -> System: MapTLidar * LidarTBase
  -> MapTBase
  -> TF / odometry / path
```

建图发布路径保持：

```text
mapOptimization legacy optimizer state
  -> OdomTLidar boundary conversion
  -> TransformFusion + IMU increment
  -> OdomTLidar
  -> OdomTLidar * LidarTBase
  -> OdomTBase TF
```

该改造只改变类型、命名和消息构造位置，不改变乘法顺序、滤波参数、优化结果或发布
频率。

## 10. 测试策略

按 RED -> GREEN 分阶段实施：

1. **现状锁定测试**：解析 launch 和发布点，锁定两种 TF 拓扑、静态外参、现有
   话题字符串和 frame 值。
2. **强类型单元测试**：验证合法组合、求逆、Eigen 往返和编译期禁止错误组合。
3. **配置测试**：验证 shared/mode-specific 配置、canonical/legacy 优先级、冲突
   拒绝、一次性告警和数值校验。
4. **ROS 适配器测试**：验证 pose、odometry、TF、path 和非法输入。
5. **地图布局测试**：保存和加载路径来自同一布局，默认全图、地面图、SC DB 与 tiled
   manifest 均能解析。
6. **源码表面测试**：禁止新增 `*_to_*` 跨模块成员、裸字符串话题、全局
   `ConfigParameters::Instance()` 和已弃用参数键。
7. **构建与 CTest**：串行构建包并运行全部非许可证历史 lint 测试。
8. **回放验收**：分别运行现有全图和分块地图定位回放，要求稳定性 baseline、TF
   连续性、有效 odometry 比例和 diagnostics 不退化；运行建图短回放确认 TF 树和
   原有话题仍存在。

## 11. 实施顺序

1. 锁定现状契约，避免重构期间无意改变 TF 和话题。
2. 引入强类型变换与 ROS 适配器，先迁移重定位边界。
3. 引入 `SlamConfig` 与兼容加载器，迁移重定位并删除 singleton。
4. 迁移建图 `ParamServer` 配置成员和内部话题。
5. 迁移建图位姿发布边界，但保持优化器内部表示。
6. 引入地图产物布局并替换全部文件名字面量。
7. 更新 YAML、launch、RViz、诊断和操作文档。
8. 完成全量测试与建图/定位回放验收。

## 12. 验收标准

- 建图和重定位 TF 拓扑、数值、频率及静态外参相对 P2.2 前不变。
- 重定位源码、参数和运行图中不存在 ROS odom frame 依赖。
- 所有跨模块 pose/transform 均能从类型或名称判断 parent 与 child。
- 两条路径共享 `SlamConfig`、标定解析器、地图布局和 ROS pose 适配器。
- `ConfigParameters` singleton 和 `ParamServer` 平铺 camelCase 配置成员被移除。
- 默认配置产生的 ROS 话题字符串与 P2.2 前一致。
- legacy 参数单独使用时可启动并告警；与 canonical 冲突时确定性失败。
- 地图产物仍可被 P2.1 全图和分块路径加载，不改变磁盘格式。
- 全部定向单测、包构建、CTest、定位回放和建图 smoke test 通过。
