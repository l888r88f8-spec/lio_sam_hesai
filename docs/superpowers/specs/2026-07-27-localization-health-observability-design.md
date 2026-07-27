# 定位可观测性与降级状态机设计

## 目标

为 `lio_sam_hesai` 重定位节点增加**定位质量可观测性**与**降级状态机**，让下游能区分"高置信定位"与"已漂移仍在发 TF"。这是 [docs/ARCHITECTURE.md](../../ARCHITECTURE.md) 第 6.2 节 P0#1（健康度+状态机）与 P0#2（数据流看门狗）的合并子项目，是车规级稳定可用的基础一环。

运行平台：Jetson Orin NX 16GB / JetPack 6 (L4T R36.3, Ubuntu 22.04) / ROS 2 Humble。

## 范围

**做**：健康度评估、五态降级状态机、数据流看门狗、`DiagnosticArray` 发布、里程计协方差如实反映不确定度。全部为增量，不改动定位算法主干（`Localization` / g2o 优化 / iVox）。

**不做**（明确排除，留给后续子项目）：
- 主动故障自恢复动作（LOST 后主动重进入 Scan Context 重定位）——仅预留状态钩子，属子项目 E。
- 实时线程优先级/CPU 亲和/地图加载移出关键路径——属子项目 D。
- 端到端集成测试、bag 回放回归、内存压测、CI——属子项目 C。本子项目只做纯单元测试。
- 建图侧（mapping）不改动；本子项目只作用于重定位节点。

## 核心决策（已与用户确认）

1. **TF 降级契约**：任何状态都继续广播 `map -> base_link`（LOST 时用 IMU 外推），通过 `DiagnosticArray` + 高协方差标注"不可信"，不停发 TF。保持 TF 连续性，由上层安全逻辑决定切换。
2. **健康信号**：配准 fitness + 数据流看门狗（雷达/IMU 到达率、时间戳单调性、断流）+ 一致性帧进度。
3. **输出接口**：标准 `diagnostic_msgs/DiagnosticArray` 发到 `/diagnostics` + 里程计 `pose.covariance` 写入真实不确定度。
4. **验证方式**：纯单元测试（gtest 驱动合成信号序列），不依赖数据集，与现有测试风格一致。
5. **架构落点**：独立的、不依赖 ROS/PCL 的纯 C++ 组件（方案一），`System` 只做薄翻译层。

## 架构与组件

新增一个纯 C++ 模块，三个可独立单测的单元；`System` 在既有线程结构上驱动。

```
include/localization_health/
  health_types.h               # 共享枚举与结构体（零依赖）
  data_stream_watchdog.h       # 单元1：数据流健康
  localization_state_machine.h # 单元2：状态机 + 健康评分
src/localization_health/
  data_stream_watchdog.cpp
  localization_state_machine.cpp
```

### health_types.h（纯值类型，零依赖）

- `enum class LocalizationState { INIT, RELOCALIZING, TRACKING, DEGRADED, LOST }`
- `enum class HealthLevel { OK, WARN, ERROR }`
- `struct StreamHealth`：`double lidar_rate_hz; double imu_rate_hz; bool lidar_stalled; bool imu_stalled; bool timestamp_monotonic; bool timestamp_jump; bool rates_known;`
- `struct HealthInputs`：`bool has_imu_init; bool map_ready; bool has_init; bool is_tracking; float reg_fitness; int consistency_hit; int consistency_required; StreamHealth stream; uint64_t now_us; uint64_t last_result_age_us;`
- `struct HealthReport`：`LocalizationState state; HealthLevel level; double health_score; double s_fitness; double s_stream; double s_consistency; std::string reason; double covariance_scale;`

### DataStreamWatchdog（依赖：仅标准库）

- 职责：把雷达/IMU 到达时间戳序列转成 `StreamHealth`。
- 接口：
  - `explicit DataStreamWatchdog(Config)`
  - `void RecordLidar(uint64_t stamp_us)`
  - `void RecordImu(uint64_t stamp_us)`
  - `StreamHealth Evaluate(uint64_t now_us) const`
- 内部：各保留一个时间戳滑动窗口（固定容量 deque）。到达率 = 窗口内样本数 / 时间跨度。断流 = `now - last_stamp > timeout`。非单调 = 新戳 < 上一戳。跳变 = 相邻间隔超出额定周期的可配倍数。窗口未填满 → `rates_known=false`。
- 线程安全：**内部持一把互斥锁**（`RecordLidar`/`RecordImu`/`Evaluate` 可能被不同回调组并发调用）。这是本设计唯一的并发点。

### LocalizationStateMachine（依赖：仅标准库 + health_types.h）

- 职责：吃 `HealthInputs`，产出 `HealthReport`，维护当前状态与迟滞计数器。
- 接口：
  - `explicit LocalizationStateMachine(Config)`
  - `HealthReport Update(const HealthInputs&)`
  - `LocalizationState state() const`
  - `void Reset()`（预留显式复位）
- 内部：当前状态 + 各迁移的连续命中计数器（迟滞防抖动）。纯函数式（除内部状态外无副作用），单线程访问（仅 10ms 定时器调用），无需锁。

### System 集成层（改现有文件，不新增职责单元）

- 持有 `DataStreamWatchdog` 与 `LocalizationStateMachine` 成员。
- `ImuMsgCallBack` / `LidarMsgCallBack`：调 `watchdog.RecordImu/RecordLidar`。
- 结果处理路径：读 `LocalizationResult` 新增的 fitness/一致性字段。
- 10ms 系统定时器：组装 `HealthInputs` → `state_machine.Update()` → 翻译成 `DiagnosticArray` 发布 + 写 odom `pose.covariance`。
- 新增一个 `rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>`。

## 状态迁移

阈值与迟滞帧数均可配；下列为默认（保守）值。

- `INIT → RELOCALIZING`：`has_imu_init && map_ready`。
- `RELOCALIZING → TRACKING`：`has_init`（`Init()` 成功：fitness < `sc_fitness_thres` 且一致性达标）。
- `TRACKING → DEGRADED`：连续 `degrade_enter_frames`(3) 帧出现 {fitness > `degraded_fitness_thres`} 或 {stream warn} 或 {一致性参考丢失}。
- `DEGRADED → TRACKING`：连续 `recover_frames`(5) 帧全部信号正常（回滞，阈值比进入更严）。
- `DEGRADED → LOST`：连续 `lost_enter_frames`(10) 帧 fitness > `lost_fitness_thres`，**或** 雷达断流 > `stream_timeout`(0.5s)，**或** 连续跳帧 > `max_skipped_frames`，**或** `last_result_age > result_timeout`（定位线程挂死）。
- `LOST → RELOCALIZING`：数据流恢复即回退（A 阶段仅做状态标记，不实施主动 re-init；动作留子项目 E）。
- `* → INIT`：仅显式复位（暂不触发，预留）。

## 健康评分与协方差

三个归一化分量（均 clamp 到 [0,1]）：
- `s_fitness = clamp(1 - fitness / lost_fitness_thres, 0, 1)`；fitness 为 inf/NaN 时取 0。
- `s_stream = min(lidar_rate/额定, imu_rate/额定)`；断流取 0；`rates_known=false` 时取 1（不据此判负，避免冷启动误报）。
- `s_consistency = hit / required`（重定位阶段）；跟踪阶段恒 1。

**主分采用短板制**：`health_score = min(s_fitness, s_stream)`，仅在 `RELOCALIZING` 阶段额外 `min` 上 `s_consistency`。避免跟踪期被恒 1 分量掩盖问题。

**状态 → level 映射**：TRACKING→OK；DEGRADED→WARN；LOST→ERROR；INIT/RELOCALIZING→WARN。

**协方差映射**：`covariance_scale = (state==LOST) ? cov_max_scale : clamp(base / max(health_score, eps), 1, cov_max_scale)`。写入 odom `pose.covariance` 6×6 对角（xyz + rpy），基础值与上限可配。TRACKING 用基础协方差；DEGRADED/LOST 放大；LOST 钳到上限。

## 数据流

```
IMU 回调  ─时间戳─┐
Lidar 回调 ─时间戳─┤→ DataStreamWatchdog ─StreamHealth─┐
                                                       ▼
LocalizationResult(扩展 fitness/一致性) ─────────► LocalizationStateMachine.Update()
                                                       │
                        10ms 定时器 薄翻译层 ◄──────── HealthReport
                          ├─► /diagnostics (DiagnosticArray)
                          └─► odom.pose.covariance
```

## 改动清单

- `include/common/sensor_data_type.h`：`LocalizationResult` 增 `float reg_fitness{inf}; int consistency_hit{0}; int consistency_required{0}; bool is_tracking{false};`。默认值保证旧路径不受影响。
- `src/slam/localization.cpp`：两处 `CacheResultToSystem`（init 与 tracking）填入 fitness（复用 `GetFitnessScore`）与一致性计数（复用 `consistency_hit_count_`）。不动算法。
- `include/slam/system.h` / `src/slam/system.cpp`：新增两个成员、DiagnosticArray 发布器、回调喂时间戳、定时器组装并发布。
- `config/relocalization.yaml`：新增 `localization_health` 参数组（阈值、迟滞帧数、额定频率、断流/结果超时、基础/上限协方差）。
- `include/slam/config_parameters.h` / `src/slam/system.cpp`：解析新参数组。
- `CMakeLists.txt`：`localization_health` 源加入 `relocalization_lib`；新增两个 gtest 目标。
- `package.xml`：增 `diagnostic_msgs` 依赖。

## 错误处理

- fitness inf/NaN：视为最差，计入 LOST 判据。
- 时间戳跳变/回退（bag 循环、时钟回跳）：watchdog 标 warn 并重置到达率窗口，避免污染频率估计。
- 冷启动窗口未满：到达率 `rates_known=false`，不据此判 LOST（只靠 fitness/一致性），避免启动误报。
- 定位线程挂死（长时间无 `LocalizationResult`）：定时器侧记录最近结果时间，`last_result_age > result_timeout` 计入 LOST。覆盖"数据流正常但定位卡死"的隐蔽失效。
- DiagnosticArray 无订阅者时照发（轻量），不做订阅者短路，保证黑盒记录仪能录到。

## 测试（纯单元，gtest）

- `test/test_data_stream_watchdog.cpp`：合成时间戳序列断言到达率、断流、跳变、非单调、冷启动 unknown。
- `test/test_localization_state_machine.cpp`：合成 `HealthInputs` 序列断言每条迁移（含迟滞：进入需连续 N 帧、离开用回滞阈值）、健康分短板制、状态→level 映射、协方差放大与 LOST 钳位；边界含 fitness=inf、一致性未达标、断流超时、结果超时、时钟回退。

## 验证

1. 两个新 gtest 目标全绿。
2. 重定位可执行文件与 `relocalization_lib` 编译通过。
3. `ros2 param list` 暴露新的 `localization_health.*` 参数；缺参/错类型在启动期报错。
4. 现有全部单元测试仍通过（回归）。

## 兼容性

- `LocalizationResult` 新字段带默认值，旧写入路径与建图侧不受影响。
- 不改动任何现有话题/TF 语义；`/diagnostics` 与 odom 协方差为纯增量输出。
- 新参数组可给出安全默认值，未配置时不改变现有行为（除开始发布诊断与协方差）。
