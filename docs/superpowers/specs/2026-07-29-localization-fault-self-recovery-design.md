# 定位故障自恢复闭环设计（子项目 E）

日期：2026-07-29
状态：已实施并验证

## 背景与目标

子项目 A 已提供定位健康状态机与雷达/IMU 数据流看门狗，`LOST`
仅表示诊断状态，尚未主动重新进入 Scan Context 自动重定位流程。子项目 E
在不重启节点的前提下补齐该故障自恢复闭环。

已确认的运行契约：

1. 每次**进入** `LOST`，立即自动尝试一次重定位。
2. 本次尝试未令系统离开 `LOST` 时，不得按帧或按定时器重复尝试。
3. 后续仅当数据流从异常恢复为健康时，立即再尝试一次；每个恢复边沿最多一次。
4. 自动重定位成功后，沿用既有结果与健康状态机路径恢复到
   `RELOCALIZING` / `TRACKING`。
5. 用户的 `/initialpose` 手动初始化始终可用，并可打破自动重试等待。

## 范围

做：

- 新增无 ROS/PCL 依赖的重试闸门，判定 LOST 首次尝试和流恢复重试。
- 通过 `System` 将闸门动作线程安全地投递给 `Localization`。
- 让定位线程以一次性自动初始化请求复用现有 Scan Context、候选验证和
  `Init()` 注册流程。
- 添加可重复的单元测试，并在诊断/日志中标出触发原因。

不做：

- 不重启 ROS 节点，不更改地图、Scan Context 检索或配准算法。
- 不修改 TF 连续性策略、线程调度、CPU 亲和或队列治理（子项目 D）。
- 不增加基于时间的周期性重试；重试只由 LOST 进入和流恢复边沿触发。
- 不将旧配准 fitness 作为“数据流恢复”的前置条件。

## 架构

```
DataStreamWatchdog ─┐
                    ├─> LocalizationStateMachine ─> HealthReport
LocalizationResult ─┘                                  │
                                                         ▼
                                    LocalizationRecoveryCoordinator
                                               │ lost_entry / stream_recovered
                                               ▼
                           Localization::RequestAutomaticRelocalization()
                                               ▼
                                 Localization thread, next lidar cluster
                                               ▼
                              existing Init() / Scan Context / registration
                                               ▼
                               existing LocalizationResult + health updates
```

`LocalizationRecoveryCoordinator` 是纯 C++ 编排组件，内部组合状态机与
`RecoveryAttemptGate`；`System` 只提供健康输入并将请求回调连接到
`Localization`。`Localization` 不读取健康状态或看门狗数据，保持算法模块与健康策略解耦。

## 重试闸门

新增 `localization_health::RecoveryAttemptGate`：

- 输入：当前 `LocalizationState`、`StreamHealth`，以及状态机参数中的额定频率
  与最小速率比例。
- 输出：`none`、`lost_entry` 或 `stream_recovered` 三种动作原因。
- 内部仅记录上一个状态和“自上次尝试后是否已见过流异常”。

判定规则：

1. `previous_state != LOST && state == LOST`：输出 `lost_entry`，并开始等待
   下一次流异常/恢复循环。
2. 仍处于 `LOST` 时，任一流不健康观测会置位“已见异常”。
3. 已见异常后，首个包含新观测且流健康的评估输出 `stream_recovered`，随即
   清除“已见异常”；持续健康的后续评估不输出动作。
4. 任何非 `LOST` 状态清除闸门内部状态；下一次进入 `LOST` 再次获得一次
   `lost_entry` 动作。

若在流中断期间产生的首次请求尚未被定位线程消费，后续恢复动作可以投递同一
请求；定位侧的原子待处理标志会合并它们，因此仍最多执行一次实际初始化。

### 数据流健康

“流恢复”只使用现有 `DataStreamWatchdog` 的 `StreamHealth`：

- `lidar_stalled == false` 且 `imu_stalled == false`；
- `timestamp_monotonic == true` 且 `timestamp_jump == false`；
- 若 `rates_known == true`，雷达和 IMU 的速率均不低于既有
  `min_stream_rate_ratio × nominal_rate`；
- 触发恢复动作的这次评估必须带有 `has_new_observation == true`。

`rates_known == false` 不阻塞冷启动后的恢复，因为看门狗的现有健康语义同样不
将其视为故障；但仍要求新的数据流观测。

## System 与定位线程接口

`System::PublishLocalizationHealth()` 调用协调器；协调器按“状态机 `Update()`、
闸门状态转换、可选请求回调”的固定顺序执行：

- 健康功能与 `localization.auto_relocalization` 都开启时，将非空动作投递给
  `Localization::RequestAutomaticRelocalization()`；
- 记录 INFO 日志，携带 `lost_entry` 或 `stream_recovered` 原因；
- 在 `/diagnostics` 增加最近一次自动重定位触发原因，便于现场确认闭环行为。

`Localization::RequestAutomaticRelocalization()` 只设置一个原子 pending 位，
不会在 ROS 定时器线程执行 PCL、地图加载或配准。定位线程在获得下一簇点云时：

1. 消费 pending 位，清除本轮候选一致性进度并重置预积分边界；
2. 将当前自动初始化尝试标记为一次性；
3. 复用 `Init()` 进行 Scan Context 候选检索、候选验证、局部地图加载和初始配准；
4. Scan Context 候选尚在收集多帧一致性时，保留本次自动尝试并继续处理下一簇点云；
5. 成功时保持现有 `LocalizationResult` 输出路径；只有真实失败才进入“等待下一自动请求”，
   不会在每个后续点云帧继续调用 `Init()`。

手动 `SetInitPose()` 清除该等待状态，因此操作者始终可以马上用 `/initialpose`
接管。普通冷启动的现有初始化行为不因该一次性等待机制而改变。

## 状态机契约

闸门不直接改写 `LocalizationStateMachine` 状态。自动初始化仍产生既有的初始化
结果；当成功结果和健康数据进入状态机时，现有 `LOST -> RELOCALIZING -> TRACKING`
路径决定诊断等级与协方差。失败期间状态保持 `LOST`，所以数据流恢复边沿可再次
驱动请求。

这避免了由协调器伪造成功结果、绕过配准验收，或把陈旧 fitness 误认为流恢复。

## 验证

新增 `RecoveryAttemptGate` gtest，至少覆盖：

1. 第一次进入 LOST 立刻发一次 `lost_entry`，重复评估不重复发。
2. 尝试失败后，连续健康数据流不重试。
3. 在 LOST 中观测到断流/速率不足/时间戳异常后，首个合格的新流观测恰发一次
   `stream_recovered`；持续健康不重复发。
4. 离开 LOST 后清理状态，下一次进入 LOST 再发一次首次尝试。
5. `rates_known`、速率阈值、时间戳回退/跳变和两路 stall 的边界条件。

还应通过协调器组合测试（LOST 入口、恢复边沿、worker 消费与手动接管）、现有包构建和健康状态机测试；在 `rosbag2/hesai_sim` 回放中检查
`/diagnostics` 的触发原因、自动重定位日志以及恢复后的 odometry。

## 验收标准

- 每次 LOST 入口最多投递一次首次自动重定位请求。
- 一次失败后，未经历数据流异常/恢复边沿时请求计数不增加。
- 每个合格恢复边沿仅增加一次请求计数，并在下一可处理点云中执行一次初始化。
- 手动 `/initialpose` 在自动等待期间仍能触发初始化。
- 自动重定位成功后恢复既有诊断和 odometry 结果路径，且所有新增单测通过。
