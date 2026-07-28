# 子项目 C：稳定性验证框架设计

## 目标

把 `lio_sam_hesai` 的重定位从“能手工回放”提升为可重复验证的黑盒回归：同一份 Hesai 仿真数据、同一份地图和同一参数下，能自动判定重定位是否成功、输出是否持续健康，以及内存是否有不可接受的增长。

固定输入为：

- rosbag：`/home/robot/rosbag2/hesai_sim`（756.378 秒，`/imu` 与 `/lidar_points`）。
- 地图：`/home/robot/map/hesai_sim2/GlobalMap.pcd`，同目录的 `GroundMap.pcd` 和 `sc_db.bin`。
- 启动入口：`localization_with_nonground.launch.py`，使用仿真时间、关闭 RViz、发布静态 TF。

该 bag 没有带时间戳的真值轨迹；地图目录中的 `trajectory.pcd` 不是可与消息逐帧匹配的 ground truth。因此本子项目采用经人工确认的**指标基线回归**，不伪造 ATE 断言。

## 范围

做：

- 一个可在本机和 CI 调用的回放测试运行器。
- 采集并断言首次有效 odom、有效 odom 比例、定位健康状态、关键诊断值和进程退出状态。
- 一个独立的 RSS 采样器，输出峰值、预热后的增长斜率和原始 CSV。
- 用版本化指标基线锁定 `hesai_sim` 的接受标准。
- 在 GitHub Actions 中提供手动/定时的自托管 Orin 回归任务。

不做：

- 修改定位算法、阈值或健康状态机行为。
- 将 1.5 GiB rosbag、地图或其派生数据提交到 Git。
- 在没有真值数据时宣称绝对轨迹精度；以后获得真值后再增加 ATE/RPE。
- 把数小时压测放入每次 PR 的普通单元测试。

## 架构

```
外置 bag + map
     │
     ▼
run_localization_regression.py
  ├─ 创建仅供本次运行使用的参数文件（替换 map 路径）
  ├─ 启动 localization_with_nonground.launch.py
  ├─ ros2 bag play --clock（固定 offset/duration/rate）
  ├─ rclpy 订阅 odom、/diagnostics、/clock
  ├─ 记录进程日志、指标 JSON、失败原因
  └─ 与 test/integration/baselines/hesai_sim.yaml 比较
     │
     ├── 快速回归：固定短片段，CI 可执行
     └── measure_localization_rss.py：循环回放，采样 /proc/<pid>/status
                                               │
                                               ▼
                                        RSS CSV + 摘要 JSON
```

运行器是 Python 3 + `rclpy` 的测试工具，而不是 gtest：它需要管理 ROS 进程、播放 bag 和异步订阅。纯 C++ 单元测试仍继续由 CTest 覆盖。

## 测试契约

### 数据集身份与可移植性

运行器通过 `--bag`、`--map` 或环境变量接受外部数据的绝对路径；默认值对应上述本机路径。启动前必须验证：

1. `metadata.yaml` 的话题名、消息数和时长与基线记录一致；
2. bag 的 `db3` 文件、`GlobalMap.pcd`、`GroundMap.pcd`、`sc_db.bin` 都存在；
3. 地图 PCD 的点数和文件大小与基线记录一致。

为了不依赖开发机的绝对路径，运行器复制标准 `relocalization.yaml` 到临时目录，并只替换 `map_path`、`ground_map_path`、`sc_db_path` 为本次输入的绝对路径。替换前会验证每个键恰好出现一次；无法安全生成参数文件即失败。

### 快速基线回归

`test/integration/baselines/hesai_sim.yaml` 是唯一的接受标准来源，包含数据集身份、回放 offset/duration/rate、指标窗口和数值边界。首次实现时，使用当前已认可的 `main` 在 Orin 上运行 `--record-baseline` 生成候选值；候选值需人工审阅后才写入该文件。

运行时收集：

- 进程和 rosbag 播放器的正常退出；任一崩溃、超时或日志中的致命错误立即失败。
- 从首个 `/clock` 到首条有限、归一化四元数有效的 `/lio_sam/localization/odometry` 的时间。
- 回放窗口内的有效 odom 数、有效 odom 比例和连续空档最大时长。
- 名为 `lio_sam_hesai: localization` 的 `/diagnostics` 状态序列；要求在基线规定的截止时间前出现 `OK`，并限制 `ERROR` 样本数与最长连续非 `OK` 时长。
- 诊断键 `health_score`、`reg_fitness`、`covariance_scale`、`lidar_rate_hz`、`imu_rate_hz` 的有限性、范围，以及 `timestamp_monotonic=true`。

比较规则分为两类：离散健康指标使用最小/最大边界，时间和比率使用相对于基线的容差。输出完整 `observed.json`，发生回归时同时输出实际值、边界和节点日志路径，便于定位而非只报“失败”。

基线生成与普通校验严格分离：只有显式 `--record-baseline` 可以写候选 JSON；普通测试绝不能改写任何已跟踪的基线。

### 长时 RSS 压测

`measure_localization_rss.py` 复用相同的启动和播放控制，但以 loop 回放运行由参数指定的持续时间。它通过 `/proc/<relocalization-pid>/status` 的 `VmRSS` 每 5 秒采样，记录：

- 预热期后的 RSS 峰值；
- 预热区间之后以线性最小二乘估计的 RSS 增长斜率（MiB/hour）；
- 采样总数、重启/崩溃次数和完整 CSV。

预热时长、总时长、峰值上限和斜率上限均保存在同一基线文件中。基线必须包含一个 Orin NX 16GB 共享内存运行配置的硬上限；默认开发机只生成报告，不可把其不同硬件数值写成 Orin 接受标准。压测失败应保留 CSV、摘要 JSON 和节点日志作为 CI artifact。

## CI 分层

1. **每次 PR**：继续运行构建和现有 CTest；不下载或播放大数据。
2. **手动与每日定时回归**：`.github/workflows/localization-stability.yml` 在标记为 `self-hosted`、`linux`、`arm64`、`orin` 的 runner 上执行快速 bag 回归。runner 预先挂载测试数据，并通过 `LIO_SAM_TESTDATA_ROOT` 提供路径。
3. **每周或手动长测**：同一自托管 runner 执行 RSS loop 压测并上传 CSV/JSON/日志。它不阻塞普通 PR，但超阈值会使该工作流失败。

工作流在 CI 中把缺少数据、地图或 ROS 工具视为配置错误并失败，避免出现“跳过即绿色”。本机用户未设置数据路径时，命令可以明确打印 `SKIPPED: external test data unavailable` 并返回跳过状态；这只适用于本机开发，不能被 CI 使用。

## 文件与接口

- `test/integration/localization_regression.py`：进程编排、ROS 订阅、指标采集、基线比较与 `--record-baseline`。
- `test/integration/localization_rss.py`：复用运行控制并采样 RSS。
- `test/integration/baselines/hesai_sim.yaml`：版本化数据身份、回放窗口、快速回归边界和 Orin 内存边界。
- `test/integration/README.md`：本机准备数据、生成候选基线、执行快速/长时测试、解释输出与更新基线的审批规则。
- `scripts/run_localization_stability.sh`：稳定的命令行入口，供开发者和 CI 调用；负责检查工具、解析数据根目录并选择 quick/memory 模式。
- `.github/workflows/localization-stability.yml`：自托管的手动/定时工作流。

`CMakeLists.txt` 只注册不依赖外部数据的接口/基线 schema 测试。真正播放 bag 的测试由脚本和自托管 CI 调用，避免常规 CTest 因缺少 1.5 GiB 数据而不稳定。

## 错误处理与安全性

- 每次执行使用独立 ROS domain ID、临时参数目录和进程组；任一失败、超时或 Ctrl-C 都向整个进程组发送终止信号并等待回收。
- 回放前等待节点完成地图加载；地图加载超时与回放期超时分开报告。
- 订阅回调只保存需要的标量和滑动窗口，不保存点云或完整 ROS 消息，避免测试自身影响内存结果。
- 所有产物写入用户指定或临时的结果目录；不会修改 bag、地图、标准配置或已跟踪基线。
- 历史趋势由 CI artifacts 保存；仓库只保存小型基线与结构化摘要，避免二进制数据膨胀。

## 验证

1. 不带外部数据时，脚本给出明确的本机跳过理由；在 CI 模式则明确失败。
2. 指向错误 bag、缺失地图、话题不匹配、节点崩溃、没有 odom、没有 `OK` 诊断、越界指标分别有确定的失败信息。
3. 在本机 `hesai_sim` 与 `hesai_sim2` 地图上生成一次候选基线并完成一次快速校验。
4. 在 Orin 上完成一次配置时长的 RSS 测试，验证 CSV、JSON 与阈值判定。
5. 常规 `colcon build` 与 CTest 保持通过；GitHub Actions YAML 通过语法检查。

## 后续扩展

当数据集提供按时间戳对齐的地面真值时，在不改变当前运行器的前提下增加轨迹匹配和 ATE/RPE 计算；新指标进入同一基线文件，但不会替代现有的可用性、健康度和内存契约。
