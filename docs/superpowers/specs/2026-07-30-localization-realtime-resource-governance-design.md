# 定位实时化与资源治理设计（子项目 D2）

日期：2026-07-30
状态：已确认，待实施

## 目标

让预处理和定位/优化工作线程在 Jetson Orin NX 上默认请求实时调度与专用 CPU，
同时在无权限或 cpuset 受限环境中保持节点可用、明确暴露实际生效状态。对超过
500 ms 的单帧处理做协作式丢弃，避免积压或提交不可信的定位输出。

## 非目标

- 不修改 ROS/DDS executor 的调度策略或 CPU 亲和性。
- 不以异步取消、信号或线程终止抢占 matcher、g2o 优化或 IMU 预积分。
- 不改变既有有界队列的容量或丢弃最旧帧策略。

## 线程运行时控制

新增独立的 `runtime::ThreadRuntimeController`。`System` 创建预处理和定位线程时，
在线程自身入口先应用控制器，再进入 `PreProcessing::Run()` 或
`Localization::Run()`；因此工作循环不会在请求的策略之前执行。

新增 `runtime` 参数组，默认值如下：

- `enable_realtime_scheduling: true`
- `preprocessing_fifo_priority: 70`
- `localization_fifo_priority: 80`
- `preprocessing_cpu_core: 4`
- `localization_cpu_core: 5`
- `frame_processing_timeout_ms: 500`

核心 `4` 与 `5` 分别保留给预处理和定位/优化，未绑定的核心继续服务 ROS/DDS
回调与操作系统。FIFO 优先级必须为 1--99；CPU 核必须为 `-1`（不绑定）或在 Linux
CPU mask 可表示范围内；超时必须大于零。违反这些配置约束会使节点在启动时拒绝运行。

控制器分别调用 `pthread_setschedparam(..., SCHED_FIFO, ...)` 与
`pthread_setaffinity_np(...)`。失败（包括缺少 `CAP_SYS_NICE` 的 `EPERM`，或 cpuset
不允许目标核心的 `EINVAL`）不是启动错误：线程继续使用系统允许的调度和亲和性，并
记录每项操作的请求值、是否生效及 errno。两项操作相互独立，一项失败不能阻止另一项
尝试。

## 帧处理看门狗

新增独立、可用显式单调时间单测的 `runtime::FrameProcessingWatchdog`。预处理与定位
各持有一个实例；它记录当前帧开始时间、最近完成耗时、最近帧是否超时、累计超时数和
累计丢帧数。`System` 的 10 ms 诊断定时器读取线程安全快照。

预处理从原始点云队列取到一帧后开始计时。若在把 `PointcloudCluster` 入定位队列前
发现耗时超过阈值，则释放该簇、不入队，并将该帧记为超时丢弃。

定位从簇队列取到一帧后开始计时，并在局部地图准备、匹配和优化后的安全检查点检查
超时。超时帧不缓存 `LocalizationResult`、不更新当前雷达云、也不更新
`last_pose_`、`last_nav_state_` 或 `last_odom_state_`。为消除 matcher 与 IMU
预积分的中间状态，定位线程重建 matcher、重置两套预积分，并要求下一帧重新建立
局部地图。不会强制中止正在执行的 matcher 或优化；超过 500 ms 的阶段自然返回后才
在检查点执行丢弃和恢复。

## 诊断

现有 `/diagnostics` 保持原有健康字段，并为 `preprocessing` 与 `localization`
分别发布：

- 请求的 FIFO 优先级、目标 CPU 核；
- FIFO 与亲和性是否实际生效及失败 errno；
- 最近帧处理毫秒数、最近帧是否超时；
- 累计超时数与累计丢帧数。

因此部署人员可区分“实时请求被降级但功能正常”与“帧处理已超期并主动丢弃”。

## 验收与测试

- 纯 C++ 单测覆盖运行时配置验证、FIFO/亲和失败降级状态、看门狗的精确 500 ms
  边界、超时/丢弃计数和正常帧恢复。
- 参数表面测试覆盖 YAML 默认值、`ConfigParameters` 解析、线程入口控制器调用和
  所有新增诊断键，不需要实时权限。
- 在无 `CAP_SYS_NICE` 的环境，节点启动并在诊断中报告 FIFO 降级；不因 `EPERM`
  退出。
- 单线程完整构建、启用的 package CTest、安装与 `hesai_sim` 短回放均通过；回放
  结果无错误样本，里程计有效比例为 1.0。
