# lio_sam_hesai 架构梳理与车规级稳定性评估

- 目标平台：Jetson Orin NX 16GB（JetPack 6 / L4T R36.3，Ubuntu 22.04，aarch64，8 核）
- 运行环境：ROS 2 Humble，PCL / OpenCV / Eigen / GTSAM 4.x / g2o / glog
- 传感器：Hesai XT16 机械式激光雷达（16 线）+ 6/9 轴 IMU
- 代码规模：约 18k 行 C++17，5 个可执行文件，1 个内部静态库
- 文档定位：先对现有架构做完整梳理，再从"车规级稳定可用"角度给出评估与改造路线，不修改代码。

> 本文中的所有 `文件:行号` 引用均可点击跳转，便于对照源码阅读。

---

## 1. 系统总览

项目由 **funny_lidar_slam** 的重定位框架与 **LIO-SAM** 的建图框架融合而成，形成两条**相互独立、通过磁盘地图产物解耦**的流水线：

1. 建图（Mapping）：4 个 ROS 节点组成经典 LIO-SAM 图，输出全局点云地图 `GlobalMap.pcd`、地面地图 `GroundMap.pcd` 以及 Scan Context 检索库 `sc_db.bin`。
2. 重定位（Localization）：单进程 `System` 节点加载离线地图，基于 Scan Context 自动重定位 + 紧耦合点面配准，直接发布 `map -> base_link` TF 与全局定位里程计。

两条流水线共享 `config/` 下的传感器/标定/坐标系参数，但**运行时不同时启动**：`system.mapping_mode` 决定角色（[config/mapping.yaml:24](../config/mapping.yaml#L24)、[config/relocalization.yaml:4](../config/relocalization.yaml#L4)）。

关键设计取向：
- 建图侧沿用 LIO-SAM 的多节点 + iSAM2 因子图 + GTSAM 回环。
- 重定位侧是 funny_lidar_slam 血统的单节点多线程 + g2o 紧耦合优化 + iVox 增量体素地图。
- 两侧代码基因不同，命名风格、坐标约定、数据结构都不一致，这是后续维护的最大认知负担（见第 8 节）。

### 1.1 顶层数据流

```
                        ┌───────────────── 建图流水线 (mapping.launch.py) ─────────────────┐
  /lidar_points ─┐      │  imageProjection → featureExtraction → mapOptimization           │
  /imu ──────────┼──────┤        ↑ deskew          ↑ 曲率特征        ↑ iSAM2+iVox+回环       │
                 │      │   imuPreintegration  ←──── /lio_sam/mapping/odometry_incremental  │
                 │      └──────────────────────────────┬───────────────────────────────────┘
                 │                                      │ SaveMap 服务 (CTRL+C 或手动)
                 │                                      ▼
                 │                        GlobalMap.pcd + GroundMap.pcd + sc_db.bin
                 │                                      │
                 │      ┌──────────── 重定位流水线 (localization.launch.py) ────────────────┐
                 └──────┤  System(单节点) : PreProcessing 线程 → Localization 线程          │
                        │  Scan Context 自动重定位 → iVox 点面紧耦合跟踪 → map->base_link TF │
                        └───────────────────────────────────────────────────────────────────┘
```

- 建图节点之间通过 ROS 话题 + 自定义 `CloudInfo.msg` 传递（进程间，DDS）。
- 重定位内部通过共享内存队列 + 条件变量传递（进程内，线程间）。
- 两条流水线之间只通过**磁盘上的 PCD/二进制地图文件**耦合，这是一个干净的解耦边界。

### 1.2 构建产物（[CMakeLists.txt](../CMakeLists.txt)）

- `lio_sam_hesai_imageProjection` —— 点云投影/去畸变/地面分割（建图 & 可选地面输出）
- `lio_sam_hesai_featureExtraction` —— LOAM 边缘/平面特征提取
- `lio_sam_hesai_imuPreintegration` —— GTSAM IMU 预积分里程计 + TF 融合
- `lio_sam_hesai_mapOptimization` —— iSAM2 后端、回环、地图/地面/SC 库保存
- `lio_sam_hesai_relocalization` —— 重定位单节点（链接内部库 `lio_sam_hesai_relocalization_lib`）

> 注意：`src/slam/frontend.cpp` 与 `src/slam/loop_closure.cpp` **存在于磁盘但未加入任何编译目标**（见 [CMakeLists.txt:126-151](../CMakeLists.txt#L126)）。它们是 funny_lidar_slam 的建图前端遗留代码，当前重定位使用的是 `Localization` 而非 `FrontEnd`。这属于"死代码"，见第 8 节。

---

## 2. 建图流水线（Mapping）

四节点均继承 `ParamServer`（[include/lio_sam/utility.hpp:69](../include/lio_sam/utility.hpp#L69)），共享一套参数加载与 IMU 坐标转换工具。QoS 定义在 [utility.hpp:566-621](../include/lio_sam/utility.hpp#L566)：点云 5 深 Reliable、IMU 2000 深 Reliable、里程计 1 深 Reliable。

### 2.1 imageProjection —— 点云投影与运动补偿

- ROS 角色（[src/imageProjection.cpp:355-373](../src/imageProjection.cpp#L355)）
  - 订阅：`lidar_topic`（点云）、`imu_topic`（IMU）、`odom_topic_incremental`（IMU 预积分里程计）。
  - 发布：`lio_sam/deskew/cloud_info`（`CloudInfo`）、去畸变点云、地面点云（全局系）、非地面点云。
- 职责
  - 多雷达点类型转换（Velodyne/Ouster/Livox/Hesai，[imageProjection.cpp:13-57,139](../src/imageProjection.cpp#L13)），统一到带 `ring`+`time` 的通用格式。
  - 基于 IMU 姿态 + 里程计平移做**逐点去畸变（deskew）**（`imuDeskewInfo`/`odomDeskewInfo`/`deskewPoint`，[imageProjection.cpp:742,801,929](../src/imageProjection.cpp#L742)）。
  - 距离图投影，输出行索引/列索引/深度供特征提取使用。
  - 地面分割（SAC 平面拟合 + ring 角度种子 + 可选高程栅格补点 `ground_grid_patcher`）。
- 线程模型：3 个互斥回调组（imu/odom/cloud），`imuLock`/`odoLock` 保护两个输入 deque；主处理在 `cloudHandler` 内串行完成（[imageProjection.cpp:471](../src/imageProjection.cpp#L471)）。
- 缓冲边界：`cloudQueue` 至少积压 2 帧才处理（[imageProjection.cpp:537-546](../src/imageProjection.cpp#L537)），`imuQueue`/`odomQueue` 按时间戳裁剪，无硬上限但随处理推进被 `pop_front`。

### 2.2 featureExtraction —— LOAM 特征提取

- 订阅 `cloud_info`，按曲率阈值（`edge_threshold`/`surface_threshold`）区分边缘点与平面点，做 ring 内标记与体素下采样，回填 `CloudInfo` 后发布。规模小（285 行），无独立线程，纯回调内计算。

### 2.3 imuPreintegration —— GTSAM IMU 预积分里程计

- 两个类：`IMUPreintegration`（预积分因子图）与 `TransformFusion`（融合激光里程计 + IMU 里程计发 TF）。
- 因子图：`ImuFactor` + `BetweenFactor`(bias) + `PriorFactor`(pose)，iSAM2 增量优化（[imuPreintegration.cpp:426-443](../src/imuPreintegration.cpp#L426)）。
- **固定滞后重置**：`key == 100` 时用边缘协方差重建图，防止 iSAM2 无界增长（[imuPreintegration.cpp:376-403](../src/imuPreintegration.cpp#L376)）——这是稳定性的关键设计。
- **失效检测与自恢复**：速度 > 30 m/s 或 bias 过大时 `failureDetection` 触发 `resetParams()`，重新初始化（[imuPreintegration.cpp:455-459](../src/imuPreintegration.cpp#L455)）。
- 坐标约定：`imuConverter` 已把 IMU 数据旋到雷达轴，故 `imu2Lidar` 仅用杠杆臂、旋转取单位阵（[imuPreintegration.cpp:239-244](../src/imuPreintegration.cpp#L239)）。

### 2.4 mapOptimization —— 后端优化与地图产物（2715 行，核心）

- 后端：GTSAM iSAM2 位姿图，关键帧插入按平移/旋转阈值，局部子地图 scan-to-map 点面配准。
- **加速**：使用 iVox 增量体素地图替代原生 kd-tree（[CMakeLists.txt:115](../CMakeLists.txt#L115) 链接 `ivox_map`）。
- 回环：`loopClosureThread` 独立线程（[mapOptimization.cpp:1287,2701](../src/mapOptimization.cpp#L1287)），距离/外部触发 + ICP 校验（`fitness_threshold`）。
- 可视化：`visualizeGlobalMapThread` 独立线程（[mapOptimization.cpp:906,2704](../src/mapOptimization.cpp#L906)）。
- 地图保存：`SaveMap.srv`（resolution / ground_resolution / destination），同时可生成 Scan Context 库（`generate_scan_context_db`）、地面地图、以及可选的 tile 切分地图（`SplitMap`）。
- 动态/静态滤波：局部几何一致性（KNN+平面）与体素持久性滤波，默认均关闭（[config/mapping.yaml:120-130](../config/mapping.yaml#L120)）。

---

## 3. 重定位流水线（Localization）

单进程 `System` 节点（[src/relocalization_node.cpp](../src/relocalization_node.cpp)），使用 **MultiThreadedExecutor** + 独立工作线程。

### 3.1 System —— 编排层（[include/slam/system.h](../include/slam/system.h)）

- ROS 接口
  - 订阅：`lidar_topic`、`imu_topic`、`/initialpose`（RViz 手动初值，独立回调组）。
  - 发布：定位里程计 `mapping_odom_topic`、定位路径、全局地图点云、当前帧点云。
  - TF：直接广播 `map -> base_link`（[system.h:56](../include/slam/system.h#L56)），不再发 `odom`。
- 三条线程：ROS executor（回调）、`PreProcessing` 线程、`Localization` 线程（[system.h:102-103](../include/slam/system.h#L102)）。
- **有界队列**（车规级的关键正向设计，[system.h:43-46](../include/slam/system.h#L43)）：
  - `kMaxRawCloudQueueSize = 3`（原始点云）
  - `kMaxCloudClusterQueueSize = 2`（预处理结果）
  - `kMaxLocalizationResultQueueSize = 5`（定位结果）
  - `kMaxLocalizationPathSize = 2000`（可视化路径）
  - 每个队列有专属互斥锁 + 条件变量，超限丢最旧帧。
- IMU 缓存：`IMUDataSearcher`，容量 `data_searcher_buffer_size`（默认 5000，[config/relocalization.yaml:60](../config/relocalization.yaml#L60)），环形 deque、线程安全（[include/common/data_searcher.h](../include/common/data_searcher.h)）。

### 3.2 PreProcessing —— 预处理线程（[include/slam/preprocessing.h](../include/slam/preprocessing.h)）

- 消费 `raw_cloud_deque_`，做点类型转换、逐点相对时间计算、IMU 去畸变（`LidarDistortionCorrector`）、LOAM 特征提取与投影，产出 `PointcloudCluster`（含 raw/ordered/corner/planar 云 + 该帧 IMU 段），压入 `cloud_cluster_deque_`。

### 3.3 Localization —— 跟踪线程（[src/slam/localization.cpp](../src/slam/localization.cpp)，1350 行）

控制流（[localization.cpp:365-458](../src/slam/localization.cpp#L365)）：
1. `Run()` 启动时加载全局地图并做 0.3m 体素下采样、构建 XY 平面 kd-tree；单独加载 `GroundMap.pcd` 用于地面高度估计；若开启自动重定位则加载 `sc_db.bin`。
2. 主循环等待 `cloud_cluster_deque_`（条件变量），未初始化时走 `Init()`。
3. 初始化成功后进入紧耦合跟踪：加载局部地图 → `matcher_->Match` → `Optimize`（IMU 预积分 + 点面残差 g2o 优化）→ 缓存 `map->lidar` 结果 → System 转成 `map->base_link` 发 TF。

### 3.4 Scan Context 自动重定位

- 离线：`mapOptimization` 为每个关键帧生成旋转不变描述子（ring×sector 高度栅格）+ 6D 位姿写入 `sc_db.bin`（[docs/.../auto-relocalization-scan-context.md](superpowers/plans/2026-07-20-auto-relocalization-scan-context.md)）。
- 在线（`TryAutoRelocalize`，[localization.cpp:641](../src/slam/localization.cpp#L641)）：
  - 用当前帧描述子 ring-key kd-tree 粗筛 top-N 候选（`sc_top_n`）。
  - 降维：yaw 来自 SC 列移，roll/pitch 由 IMU 姿态锁定，z 由 `EstimateMapGroundZ` 约束——检索只解 (x,y)。
  - **串行早退**（[serial-early-exit 设计](superpowers/specs/2026-07-24-serial-early-exit-relocalization-design.md)）：按 SC 分数升序逐个精配准；fitness < `sc_accept_fitness` 立即接受；否则取所有候选中 < `sc_fitness_thres` 的最优。
  - **多帧一致性确认**（[localization.cpp:255-291](../src/slam/localization.cpp#L255)）：连续 `sc_consistency_frames` 帧位姿平移/航向一致才切入跟踪，抗误匹配。
  - 接受策略函数 `ShouldEarlyAcceptRelocalization` 单独成头文件并有单测（[acceptance_policy.h](../include/relocalization/acceptance_policy.h)）。

### 3.5 配准与紧耦合优化

- 可插拔配准接口 `RegistrationInterface`（[registration_interface.h](../include/registration/registration_interface.h)），默认 `PointToPlane_IVOX`，另有 `IcpOptimized`/`PointToPlane_KdTree`/`LoamFull_KdTree`/`IncrementalNDT`。
- 紧耦合融合 `TightCouplingOptimization`：g2o 图把 IMU 预积分因子、位姿先验、点面残差联合优化（[src/optimization/g2o/](../src/optimization/g2o/)）。
- **线程安全修复**：早期矩配器持有可变 `static` 局部变量，自动重定位并行时会数据竞争；已改为成员变量并有回归测试（[phase1 计划 Task 1](superpowers/plans/2026-07-22-phase1-relocalization-efficiency.md)、[test_relocalization_efficiency_surface.cpp](../test/test_relocalization_efficiency_surface.cpp)）。

---

## 4. 共享库与公共组件

- **iVox 增量体素地图**（[include/ivox_map/ivox_map.h](../include/ivox_map/ivox_map.h)、[src/ivox_map/ivox_map.cpp](../src/ivox_map/ivox_map.cpp)）
  - `unordered_map<voxel-key, list-iterator>` + `list` 组成 **LRU 缓存**，`capacity_` 默认 1,000,000 体素，超限淘汰最久未用（[ivox_map.cpp:114-135](../src/ivox_map/ivox_map.cpp#L114)）——内存有界。
  - 支持 NEARBY6/18/26 邻域最近邻查询，供点面配准使用。
  - **注意**：`IVoxMap` 自身**非线程安全**；重定位并行阶段靠"每线程独立 matcher/独立 iVox"规避，而非内部加锁。
- **IMU 预积分（g2o 侧）**（[include/imu/pre_integration.h](../include/imu/pre_integration.h)）：维护 ΔR/Δv/Δp 及对 bias 的雅可比，供紧耦合边使用；`data_synchronizer` 做雷达-IMU 时间对齐。
- **雷达模型与去畸变**（[src/lidar/](../src/lidar/)）：`LidarModel` 描述线数/角分辨率/FOV；`LidarDistortionCorrector` 用 IMU 段做逐点运动补偿。
- **公共工具**（[include/common/](../include/common/)）
  - `DataSearcher`：线程安全、按时间戳检索的有界环形缓存（第 3.1 节）。
  - `bounded_async.h`：`RunBoundedAsync` 把 N 个任务限制在 `min(N, 硬件线程数)` 个 worker 内并行，避免 `std::async` 无界起线程（[bounded_async.h](../include/common/bounded_async.h)），有单测。
  - `LidarImuCalibration`：从 `translation/rotation/rpy` 构造 4×4 外参，**入参长度校验**，非法数组抛异常而非 UB（[lidar_imu_calibration.h:23-34](../include/common/lidar_imu_calibration.h#L23)）。
  - `keyframe.h`/`frame.h`/`pointcloud_cluster.h`：关键帧与点云簇数据结构。

### 4.1 统一参数架构

近期完成了一次**硬切换式参数重构**（[unified-ros-parameters 设计](superpowers/specs/2026-07-24-unified-ros-parameters-design.md)）：把混杂的 camelCase 顶层参数统一为按功能分组的 snake_case（`system`/`sensor_topic`/`frames`/`lidar`/`imu`/`calibration`/`localization`/`frontend`/...）。两个 YAML 共享硬件/标定/坐标系键，模式相关键各自独立。有专门的 schema 测试拒绝任何遗留参数名（[test_unified_parameter_schema.cpp](../test/test_unified_parameter_schema.cpp)）。

**标定方向约定**（易错点，已被测试锁定）：`calibration.rotation/translation` 定义为 `T^{imu}_{lidar}`。重定位直接使用；建图侧取其转置得到 `R^{lidar}_{imu}`，并在雷达轴下表达杠杆臂。[test_lidar_imu_calibration.cpp](../test/test_lidar_imu_calibration.cpp) 用非单位阵旋转锁定该方向。

---

## 5. 坐标系与 TF 树

- 建图模式（[launch/mapping.launch.py](../launch/mapping.launch.py)）：`map -(static)-> odom -(动态,由里程计)-> base_link`，另有 `base_link -> hesai_lidar`、`base_link -> imu_link` 静态 TF。
- 重定位模式（[launch/localization.launch.py](../launch/localization.launch.py)）：`System` 直接发 `map -> base_link`（动态），launch 发 `base_link -> hesai_lidar / imu_link` 静态 TF；可用 `publish_static_tf:=false` 交给机器人 URDF。
- 内部位姿约定：`Localization` 内部维护 `map->lidar`（沿用 funny_lidar_slam），`System` 输出前转 `map->base_link`（[sensor_data_type.h:60-69](../include/common/sensor_data_type.h#L60)）。
- **风险点**：两个 launch 的静态外参数值不一致（建图 `0,0,0.405` / 重定位 `0.02,0,0.32`），且与 `calibration.translation`（`0,0,0.10`）不是同一语义量。上车前必须以实测安装量统一，否则建图与重定位处于不同基准。

---

## 6. 车规级稳定性评估

按"可用性 / 确定性 / 故障恢复 / 资源边界 / 可观测性"五个维度评估。总体判断：**该项目在算法与内存边界上已有相当好的工程基础，但距离车规级（ISO 26262 / 功能安全思维）仍有明确差距**，主要集中在故障恢复的完备性、实时确定性、以及输出可信度自评上。

### 6.1 已具备的正向基础（可直接受益）

- 队列全部有界（第 3.1 节），IMU/数据缓存为环形有界结构，iVox 为 LRU 有界——**无明显无界内存增长**（建图侧关键帧累积除外，见 6.3）。
- iSAM2 固定滞后重置 + 失效检测自恢复（第 2.3 节），预积分不会无界膨胀。
- 并发经过专门加固：消除可变 `static`、`RunBoundedAsync` 限制并发度、独立 matcher 规避竞争。
- 参数硬切换 + schema 测试 + 标定方向测试，配置错误在启动期即报错而非静默漂移。
- Scan Context 多帧一致性 + fitness 阈值，抗单帧误匹配。
- 重定位节点可选 GDB 包裹并打印崩溃栈（[localization.launch.py:24-41](../launch/localization.launch.py#L24)），便于现场取证。

### 6.2 关键风险（车规级必须闭环）

按严重度（阻塞上车 / 需缓解 / 建议改进）分级。

#### A. 输出可信度与降级——【阻塞】
- 重定位持续输出 `map->base_link` TF，但**跟踪发散/配准失败时没有统一的"定位质量"信号**对外暴露。下游导航无法区分"高置信定位"与"已漂移仍在发 TF"。
- 缺少**降级状态机**（INIT → RELOCALIZING → TRACKING → DEGRADED → LOST）。当前 `has_init_` 只是布尔量。
- 建议：定义定位健康度（fitness、内点率、IMU-雷达残差、协方差迹），以 `diagnostic_msgs/DiagnosticStatus` 发布；LOST 时停止发布 TF 或在里程计协方差中标注，交由上层安全逻辑处置。

#### B. 实时确定性——【需缓解】
- 关键路径大量 `pcl::io::loadPCDFile`、体素滤波、kd-tree 构建在**运行线程内同步执行**（如 `Run()` 启动时全图加载与下采样 [localization.cpp:375-390](../src/slam/localization.cpp#L375)）；大地图下首帧延迟不可控。
- `std::async` / 线程分离用于回环与并行候选，**无实时优先级、无 CPU 亲和性、无 WCET 约束**；Orin NX 上与其他任务争抢时抖动不可预测。
- `mapping_process_interval` 等节流为软周期，非硬实时调度。
- 建议：地图加载与 kd-tree 构建移入启动阶段并显式"就绪"握手；对雷达回调/优化线程设置 `SCHED_FIFO` 与 `taskset` 亲和；给单帧处理设看门狗，超时丢帧并计数。

#### C. 故障恢复完备性——【需缓解】
- 重定位 `LOST` 后**没有自动重进入自动重定位**的闭环（一致性失败会 reset 计数，但整体恢复策略未成体系）。
- IMU/雷达**数据断流、时间戳跳变、时钟回退**缺乏系统性检测与处置（建图侧有 deskew 前置检查，但重定位侧对长时间无 IMU 仅"跳帧"[localization.cpp:460-464](../src/slam/localization.cpp#L460)）。
- 传感器坐标/外参在线自检缺失（仅启动期校验）。
- 建议：数据流看门狗（雷达 10Hz、IMU ≥100Hz 的到达率监控）；时钟单调性检查；LOST 超阈值自动回退到 Scan Context 重定位。

#### D. 资源边界（Orin NX 16GB 特有）——【需缓解】
- 建图侧关键帧点云在内存中累积，长时间/大场景建图会持续增长；Orin NX 16GB 且与相机/规划共用内存时需评估上限（`mapOptimization` 保留各关键帧 corner/surf 云）。
- iVox `capacity_=1,000,000` 体素在 0.2m 分辨率下理论占用可观，需结合实际地图规模压测。
- 全局地图一次性载入内存并复制出 XY 版本 + kd-tree（[localization.cpp:383-390](../src/slam/localization.cpp#L383)），500,000 m² 场景需实测峰值内存。
- 建议：建图设关键帧/内存上限或落盘策略；重定位大地图用 tile 分块（`SplitMap` 已具雏形，[split_map.cpp](../src/slam/split_map.cpp)，但代码注释标注 "Not recommended"，[localization.h:135](../include/slam/localization.h#L135)）；上车前做长时稳态内存压测（RSS 曲线）。

#### E. 已知代码级缺陷——【建议改进】
- `IVoxMap::AddPoints(world_points, nearest_points)` 重载中 [ivox_map.cpp:89](../src/ivox_map/ivox_map.cpp#L89) 用 `return` 而非 `continue`，会提前退出整批添加；该重载当前标 `[[maybe_unused]]` 未被调用，属潜在地雷。
- 死代码 `frontend.cpp` / `loop_closure.cpp` 未编译（第 1.2 节），易误导维护者。
- `using namespace std;` 出现在头文件 [utility.hpp:63](../include/lio_sam/utility.hpp#L63)，污染所有包含者的命名空间。
- 建图与重定位两套坐标/命名体系并存，认知负担高、易引入方向性 bug。

### 6.3 测试覆盖现状

现有 11 个 gtest（[test/](../test/)）集中在：参数 schema、标定契约、地面 ring 策略、Scan Context、接受策略、bounded_async、ground map source。**均为单元/契约测试，没有端到端管线集成测试、没有回放数据集回归、没有长时稳定性/内存压测。** 这是车规级最大的验证缺口。

---

## 7. Jetson Orin NX 16GB 部署建议

1. 编译：`colcon build --symlink-install`，`Release` + `-O2`（[CMakeLists.txt:5-9](../CMakeLists.txt#L5) 已默认 Release）；针对 Ampere（Orin）可加 `-march=armv8.2-a+...`，谨慎评估数值一致性。
2. 功耗/频率：上车固定 `nvpmodel` 功耗模式 + `jetson_clocks` 锁频，避免 DVFS 抖动影响实时性。
3. 线程与核：8 核建议划分——雷达回调/优化线程绑定专用核（`SCHED_FIFO`），OpenMP/`RunBoundedAsync` 限制在剩余核，避免与 DDS、相机、规划争抢。`number_of_cores` 当前建图默认 8（[config/mapping.yaml:25](../config/mapping.yaml#L25)），应给系统留余量。
4. DDS/QoS：点云为 Reliable 5 深（[utility.hpp:604-614](../include/lio_sam/utility.hpp#L604)，注释保留了 BEST_EFFORT 选项）。高帧率大点云 + Reliable 在丢包链路会累积延迟；车载建议评估传感器话题改 BestEffort，并调优 CycloneDDS/FastDDS 共享内存传输。
5. 内存：16GB 与相机/规划/DDS 共享，务必做长时 RSS 压测（见 6.2 D），预留 OOM 余量，必要时设 cgroup 限额。
6. 存储：地图 PCD 与 `sc_db.bin` 放本地 NVMe，避免运行时 I/O 抖动进入关键路径。

---

## 8. 架构改进路线（面向"车规级稳定可用"）

按优先级（P0 上车前必须 / P1 尽快 / P2 演进）组织。**本节仅为建议，未改动代码。**

### P0 —— 上车前必须闭环
1. **定位健康度与降级状态机**：显式状态机 + `DiagnosticStatus` + 里程计协方差如实反映置信度；LOST 时对下游明确不可用（对应 6.2 A/C）。
2. **数据流看门狗**：雷达/IMU 到达率、时间戳单调性、跳变检测；异常进入降级并计数（6.2 C）。
3. **外参/坐标基准统一**：统一两个 launch 的静态 TF 与 `calibration.*` 语义，以实测安装量为准，并补一条端到端断言（6.2、第 5 节）。
4. **长时稳定性/内存压测**：真实数据回放跑数小时，采集 RSS、时延分布、丢帧率曲线（6.3）。

### P1 —— 尽快改进
5. **实时化**：地图加载/构建移出关键路径，线程实时优先级 + CPU 亲和 + 单帧看门狗（6.2 B）。
6. **端到端集成测试 + 回放回归**：固定小数据集，断言轨迹误差/重定位成功率阈值，纳入 CI（6.3）。
7. **失败自恢复闭环**：LOST → 自动 Scan Context 重定位重入（6.2 C）。
8. **修复已知缺陷**：`ivox_map.cpp:89` 的 `return`/`continue`、清理死代码、移除头文件 `using namespace std;`（6.2 E）。

### P2 —— 架构演进
9. **大场景分块地图**：把 `SplitMap` tile 机制转正，支持按需加载，去掉全图常驻（6.2 D）。
10. **统一坐标/命名体系**：逐步收敛建图与重定位两套基因，降低长期维护风险（6.2 E）。
11. **参数/地图版本化**：地图产物（PCD + sc_db + 参数快照）打版本戳并在加载时校验兼容性。

---

## 9. 快速索引

- 建图入口：[launch/mapping.launch.py](../launch/mapping.launch.py)、参数 [config/mapping.yaml](../config/mapping.yaml)
- 重定位入口：[launch/localization.launch.py](../launch/localization.launch.py)、参数 [config/relocalization.yaml](../config/relocalization.yaml)
- 编排/线程/队列：[include/slam/system.h](../include/slam/system.h)
- 跟踪与自动重定位：[src/slam/localization.cpp](../src/slam/localization.cpp)
- iVox 地图：[src/ivox_map/ivox_map.cpp](../src/ivox_map/ivox_map.cpp)
- 参数契约与标定：[include/common/lidar_imu_calibration.h](../include/common/lidar_imu_calibration.h)、[test/test_unified_parameter_schema.cpp](../test/test_unified_parameter_schema.cpp)
- 现有设计文档：[docs/superpowers/specs/](superpowers/specs/)、[docs/superpowers/plans/](superpowers/plans/)

> 本文档基于对当前代码库（分支 HEAD）的静态阅读整理，未运行构建或实机验证。第 6-8 节的风险与建议应结合实机回放数据与压测结果进一步确认。
