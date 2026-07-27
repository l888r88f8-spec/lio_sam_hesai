# 定位可观测性与降级状态机 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 `lio_sam_hesai` 重定位节点增加定位质量可观测性与五态降级状态机，通过 `DiagnosticArray` 与里程计协方差如实暴露定位置信度，同时始终保持 `map->base_link` TF 连续。

**Architecture:** 新增一个不依赖 ROS/PCL 的纯 C++ 模块 `localization_health`，含两个可独立单测的单元（数据流看门狗、状态机+健康评分）和一个零依赖的类型头。`System` 作为薄翻译层：在 IMU/雷达回调喂时间戳、在 10ms 定时器里评估并发布诊断与协方差。定位算法主干（`Localization`/g2o/iVox）不改动，仅让 `LocalizationResult` 多携带 fitness 与一致性字段。

**Tech Stack:** C++17, ROS 2 Humble, `diagnostic_msgs`, `nav_msgs`, GoogleTest (ament_add_gtest), colcon/ament, glog。

**Spec:** [docs/superpowers/specs/2026-07-27-localization-health-observability-design.md](../specs/2026-07-27-localization-health-observability-design.md)

**分支:** `feat/localization-health-observability`（已含 spec 提交）

---

## 文件结构

- Create `include/localization_health/health_types.h` — 共享枚举与值类型（`LocalizationState`/`HealthLevel`/`StreamHealth`/`HealthInputs`/`HealthReport`）。零依赖。
- Create `include/localization_health/data_stream_watchdog.h` + `src/localization_health/data_stream_watchdog.cpp` — 单元1：时间戳序列 → `StreamHealth`。内部互斥锁。
- Create `include/localization_health/localization_state_machine.h` + `src/localization_health/localization_state_machine.cpp` — 单元2：`HealthInputs` → `HealthReport`，迟滞状态机 + 健康评分 + 协方差缩放。
- Create `test/test_data_stream_watchdog.cpp` — 单元1 纯单测。
- Create `test/test_localization_state_machine.cpp` — 单元2 纯单测。
- Modify `include/common/sensor_data_type.h` — `LocalizationResult` 增 4 个字段。
- Modify `src/slam/localization.cpp` — 两处结果缓存填 fitness/一致性。
- Modify `include/slam/config_parameters.h` — 增健康参数成员。
- Modify `include/slam/system.h` + `src/slam/system.cpp` — 成员、发布器、回调喂数据、定时器评估发布、参数解析。
- Modify `config/relocalization.yaml` — 新增 `localization_health` 参数组。
- Modify `CMakeLists.txt` — 源文件入库 + 两个 gtest 目标。
- Modify `package.xml` — 增 `diagnostic_msgs` 依赖。

---

### Task 1: 共享类型头 health_types.h

**Files:**
- Create: `include/localization_health/health_types.h`

- [ ] **Step 1: 创建类型头（完整内容，勿省略）**

```cpp
#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_HEALTH_TYPES_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_HEALTH_TYPES_H

#include <cstdint>
#include <string>

namespace localization_health {

enum class LocalizationState {
  kInit,
  kRelocalizing,
  kTracking,
  kDegraded,
  kLost,
};

enum class HealthLevel {
  kOk,
  kWarn,
  kError,
};

// 数据流健康：由 DataStreamWatchdog 产出。
struct StreamHealth {
  double lidar_rate_hz = 0.0;
  double imu_rate_hz = 0.0;
  bool lidar_stalled = false;
  bool imu_stalled = false;
  bool timestamp_monotonic = true;
  bool timestamp_jump = false;
  bool rates_known = false;  // 窗口未填满时为 false
};

// 一次状态机评估的全部输入。
struct HealthInputs {
  bool has_imu_init = false;
  bool map_ready = false;
  bool has_init = false;
  bool is_tracking = false;
  float reg_fitness = 0.0f;       // 越小越好；inf/NaN 视为最差
  int consistency_hit = 0;
  int consistency_required = 0;
  StreamHealth stream{};
  std::uint64_t now_us = 0u;
  std::uint64_t last_result_age_us = 0u;  // 距最近一次定位结果的时长
};

// 状态机输出。
struct HealthReport {
  LocalizationState state = LocalizationState::kInit;
  HealthLevel level = HealthLevel::kWarn;
  double health_score = 0.0;       // [0,1]
  double s_fitness = 0.0;
  double s_stream = 0.0;
  double s_consistency = 0.0;
  double covariance_scale = 1.0;   // 乘到基础协方差上
  std::string reason;              // 人类可读
};

inline const char* ToString(LocalizationState state) {
  switch (state) {
    case LocalizationState::kInit: return "INIT";
    case LocalizationState::kRelocalizing: return "RELOCALIZING";
    case LocalizationState::kTracking: return "TRACKING";
    case LocalizationState::kDegraded: return "DEGRADED";
    case LocalizationState::kLost: return "LOST";
  }
  return "UNKNOWN";
}

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_HEALTH_TYPES_H
```

- [ ] **Step 2: 提交**

```bash
git add include/localization_health/health_types.h
git commit -m "feat(health): add localization health shared types"
```

---

### Task 2: DataStreamWatchdog（单元1，TDD）

**Files:**
- Create: `test/test_data_stream_watchdog.cpp`
- Create: `include/localization_health/data_stream_watchdog.h`
- Create: `src/localization_health/data_stream_watchdog.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

创建 `test/test_data_stream_watchdog.cpp`：

```cpp
#include "localization_health/data_stream_watchdog.h"

#include <gtest/gtest.h>

using localization_health::DataStreamWatchdog;

namespace {
constexpr std::uint64_t kSecond = 1000000ull;  // us
}

TEST(DataStreamWatchdog, ColdStartRatesUnknown) {
  DataStreamWatchdog::Config cfg;
  DataStreamWatchdog wd(cfg);
  wd.RecordLidar(kSecond);
  const auto h = wd.Evaluate(kSecond);
  EXPECT_FALSE(h.rates_known);      // 样本不足
  EXPECT_FALSE(h.lidar_stalled);    // 刚到过数据
}

TEST(DataStreamWatchdog, EstimatesLidarRateFromWindow) {
  DataStreamWatchdog::Config cfg;
  cfg.min_samples = 3;
  DataStreamWatchdog wd(cfg);
  // 10 Hz：每 0.1s 一帧
  for (int i = 0; i < 5; ++i) {
    wd.RecordLidar(static_cast<std::uint64_t>(i) * kSecond / 10ull);
  }
  const auto h = wd.Evaluate(4ull * kSecond / 10ull);
  EXPECT_TRUE(h.rates_known);
  EXPECT_NEAR(h.lidar_rate_hz, 10.0, 1.0);
}

TEST(DataStreamWatchdog, DetectsStallAfterTimeout) {
  DataStreamWatchdog::Config cfg;
  cfg.lidar_stall_timeout_us = kSecond / 2;  // 0.5s
  DataStreamWatchdog wd(cfg);
  wd.RecordLidar(kSecond);
  const auto h = wd.Evaluate(kSecond + kSecond);  // 1s 后无新数据
  EXPECT_TRUE(h.lidar_stalled);
}

TEST(DataStreamWatchdog, FlagsNonMonotonicTimestamp) {
  DataStreamWatchdog::Config cfg;
  DataStreamWatchdog wd(cfg);
  wd.RecordLidar(10ull * kSecond);
  wd.RecordLidar(9ull * kSecond);  // 回退
  const auto h = wd.Evaluate(10ull * kSecond);
  EXPECT_FALSE(h.timestamp_monotonic);
}

TEST(DataStreamWatchdog, FlagsTimestampJump) {
  DataStreamWatchdog::Config cfg;
  cfg.min_samples = 2;
  cfg.nominal_lidar_period_us = kSecond / 10;  // 0.1s
  cfg.jump_period_multiplier = 5.0;
  DataStreamWatchdog wd(cfg);
  wd.RecordLidar(0);
  wd.RecordLidar(kSecond);  // 间隔 1s，远超 5×0.1s
  const auto h = wd.Evaluate(kSecond);
  EXPECT_TRUE(h.timestamp_jump);
}
```

- [ ] **Step 2: 加 CMake 测试目标**

在 `CMakeLists.txt` 的 `if(BUILD_TESTING)` 块内（紧接 `test_bounded_async` 之后，[CMakeLists.txt:236-237](../../../CMakeLists.txt#L236)）追加：

```cmake
  ament_add_gtest(test_data_stream_watchdog test/test_data_stream_watchdog.cpp
    src/localization_health/data_stream_watchdog.cpp)
  target_include_directories(test_data_stream_watchdog PRIVATE include)
```

- [ ] **Step 3: 运行测试，确认因缺头文件而失败**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai --cmake-target tests 2>&1 | tail -20`
Expected: 编译失败，`data_stream_watchdog.h: No such file or directory`。

- [ ] **Step 4: 写头文件**

创建 `include/localization_health/data_stream_watchdog.h`：

```cpp
#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_DATA_STREAM_WATCHDOG_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_DATA_STREAM_WATCHDOG_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "localization_health/health_types.h"

namespace localization_health {

// 把雷达/IMU 到达时间戳序列转成 StreamHealth。线程安全（内部锁）。
class DataStreamWatchdog {
 public:
  struct Config {
    std::size_t window_size = 50;               // 时间戳窗口容量
    std::size_t min_samples = 5;                // 少于此数则 rates_known=false
    std::uint64_t lidar_stall_timeout_us = 500000u;   // 0.5s
    std::uint64_t imu_stall_timeout_us = 100000u;     // 0.1s
    std::uint64_t nominal_lidar_period_us = 100000u;  // 10 Hz
    std::uint64_t nominal_imu_period_us = 10000u;     // 100 Hz
    double jump_period_multiplier = 5.0;        // 相邻间隔超此倍额定周期判跳变
  };

  explicit DataStreamWatchdog(const Config& config);

  void RecordLidar(std::uint64_t stamp_us);
  void RecordImu(std::uint64_t stamp_us);

  [[nodiscard]] StreamHealth Evaluate(std::uint64_t now_us) const;

 private:
  struct Channel {
    std::deque<std::uint64_t> stamps;
    std::uint64_t last_stamp = 0u;
    bool has_last = false;
    bool monotonic = true;
    bool jump = false;
  };

  void Record(Channel& ch, std::uint64_t stamp_us,
              std::uint64_t nominal_period_us);
  double RateHz(const Channel& ch) const;

  Config config_;
  Channel lidar_;
  Channel imu_;
  mutable std::mutex mutex_;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_DATA_STREAM_WATCHDOG_H
```

- [ ] **Step 5: 写实现**

创建 `src/localization_health/data_stream_watchdog.cpp`：

```cpp
#include "localization_health/data_stream_watchdog.h"

namespace localization_health {

DataStreamWatchdog::DataStreamWatchdog(const Config& config)
    : config_(config) {}

void DataStreamWatchdog::Record(Channel& ch, std::uint64_t stamp_us,
                                std::uint64_t nominal_period_us) {
  if (ch.has_last) {
    if (stamp_us < ch.last_stamp) {
      ch.monotonic = false;
      // 时钟回退：清空窗口，避免污染频率估计
      ch.stamps.clear();
      ch.has_last = true;
      ch.last_stamp = stamp_us;
      return;
    }
    const std::uint64_t delta = stamp_us - ch.last_stamp;
    if (static_cast<double>(delta) >
        config_.jump_period_multiplier * static_cast<double>(nominal_period_us)) {
      ch.jump = true;
    }
  }
  ch.last_stamp = stamp_us;
  ch.has_last = true;
  ch.stamps.push_back(stamp_us);
  while (ch.stamps.size() > config_.window_size) {
    ch.stamps.pop_front();
  }
}

double DataStreamWatchdog::RateHz(const Channel& ch) const {
  if (ch.stamps.size() < config_.min_samples) {
    return 0.0;
  }
  const std::uint64_t span = ch.stamps.back() - ch.stamps.front();
  if (span == 0u) {
    return 0.0;
  }
  const double intervals = static_cast<double>(ch.stamps.size() - 1);
  return intervals * 1.0e6 / static_cast<double>(span);
}

void DataStreamWatchdog::RecordLidar(std::uint64_t stamp_us) {
  std::lock_guard<std::mutex> lg(mutex_);
  Record(lidar_, stamp_us, config_.nominal_lidar_period_us);
}

void DataStreamWatchdog::RecordImu(std::uint64_t stamp_us) {
  std::lock_guard<std::mutex> lg(mutex_);
  Record(imu_, stamp_us, config_.nominal_imu_period_us);
}

StreamHealth DataStreamWatchdog::Evaluate(std::uint64_t now_us) const {
  std::lock_guard<std::mutex> lg(mutex_);
  StreamHealth h;
  h.lidar_rate_hz = RateHz(lidar_);
  h.imu_rate_hz = RateHz(imu_);
  h.rates_known = lidar_.stamps.size() >= config_.min_samples &&
                  imu_.stamps.size() >= config_.min_samples;
  h.lidar_stalled = lidar_.has_last &&
                    now_us > lidar_.last_stamp &&
                    (now_us - lidar_.last_stamp) > config_.lidar_stall_timeout_us;
  h.imu_stalled = imu_.has_last &&
                  now_us > imu_.last_stamp &&
                  (now_us - imu_.last_stamp) > config_.imu_stall_timeout_us;
  h.timestamp_monotonic = lidar_.monotonic && imu_.monotonic;
  h.timestamp_jump = lidar_.jump || imu_.jump;
  return h;
}

}  // namespace localization_health
```

- [ ] **Step 6: 运行测试，确认全部通过**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai && colcon test --packages-select lio_sam_hesai --ctest-args -R test_data_stream_watchdog 2>&1 | tail -20`
Expected: 5 个测试全 PASS。

- [ ] **Step 7: 提交**

```bash
git add include/localization_health/data_stream_watchdog.h src/localization_health/data_stream_watchdog.cpp test/test_data_stream_watchdog.cpp CMakeLists.txt
git commit -m "feat(health): add data stream watchdog with unit tests"
```

---

### Task 3: LocalizationStateMachine（单元2，TDD）

**Files:**
- Create: `test/test_localization_state_machine.cpp`
- Create: `include/localization_health/localization_state_machine.h`
- Create: `src/localization_health/localization_state_machine.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

创建 `test/test_localization_state_machine.cpp`：

```cpp
#include "localization_health/localization_state_machine.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using localization_health::HealthInputs;
using localization_health::HealthLevel;
using localization_health::HealthReport;
using localization_health::LocalizationState;
using localization_health::LocalizationStateMachine;
using localization_health::StreamHealth;

namespace {

// 构造一个“健康跟踪中”的输入。
HealthInputs GoodTracking() {
  HealthInputs in;
  in.has_imu_init = true;
  in.map_ready = true;
  in.has_init = true;
  in.is_tracking = true;
  in.reg_fitness = 0.1f;
  in.consistency_required = 3;
  in.consistency_hit = 3;
  in.stream.lidar_rate_hz = 10.0;
  in.stream.imu_rate_hz = 100.0;
  in.stream.rates_known = true;
  in.last_result_age_us = 0u;
  return in;
}

LocalizationStateMachine::Config FastConfig() {
  LocalizationStateMachine::Config cfg;
  cfg.degrade_enter_frames = 2;
  cfg.recover_frames = 2;
  cfg.lost_enter_frames = 3;
  cfg.degraded_fitness_thres = 0.5;
  cfg.lost_fitness_thres = 1.0;
  cfg.nominal_lidar_rate_hz = 10.0;
  cfg.nominal_imu_rate_hz = 100.0;
  cfg.result_timeout_us = 1000000u;
  cfg.base_covariance = 0.05;
  cfg.max_covariance_scale = 100.0;
  return cfg;
}

}  // namespace

TEST(LocalizationStateMachine, StartsInInit) {
  LocalizationStateMachine sm(FastConfig());
  EXPECT_EQ(sm.state(), LocalizationState::kInit);
}

TEST(LocalizationStateMachine, InitToRelocalizingWhenImuAndMapReady) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs in;
  in.has_imu_init = true;
  in.map_ready = true;
  in.has_init = false;
  const auto r = sm.Update(in);
  EXPECT_EQ(r.state, LocalizationState::kRelocalizing);
  EXPECT_EQ(r.level, HealthLevel::kWarn);
}

TEST(LocalizationStateMachine, RelocalizingToTrackingOnInit) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  const auto r = sm.Update(GoodTracking());
  EXPECT_EQ(r.state, LocalizationState::kTracking);
  EXPECT_EQ(r.level, HealthLevel::kOk);
}

TEST(LocalizationStateMachine, TrackingToDegradedAfterHysteresis) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  sm.Update(GoodTracking());

  HealthInputs bad = GoodTracking();
  bad.reg_fitness = 0.8f;  // > degraded(0.5), < lost(1.0)
  EXPECT_EQ(sm.Update(bad).state, LocalizationState::kTracking);  // 1st frame
  EXPECT_EQ(sm.Update(bad).state, LocalizationState::kDegraded);  // 2nd -> enter
}

TEST(LocalizationStateMachine, DegradedToLostOnPersistentBadFitness) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  sm.Update(GoodTracking());
  HealthInputs bad = GoodTracking();
  bad.reg_fitness = 1.5f;  // > lost(1.0)
  LocalizationState last = LocalizationState::kTracking;
  for (int i = 0; i < 6; ++i) {
    last = sm.Update(bad).state;
  }
  EXPECT_EQ(last, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, DegradedToLostOnLidarStall) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  sm.Update(GoodTracking());
  HealthInputs stalled = GoodTracking();
  stalled.stream.lidar_stalled = true;
  LocalizationState last = LocalizationState::kTracking;
  for (int i = 0; i < 6; ++i) {
    last = sm.Update(stalled).state;
  }
  EXPECT_EQ(last, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, LostToRelocalizingWhenStreamRecovers) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  sm.Update(GoodTracking());
  HealthInputs bad = GoodTracking();
  bad.reg_fitness = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 6; ++i) {
    sm.Update(bad);
  }
  ASSERT_EQ(sm.state(), LocalizationState::kLost);
  // 数据流仍在但已丢失跟踪：需要重新初始化 → RELOCALIZING
  HealthInputs recover = GoodTracking();
  recover.has_init = false;  // System 会在 LOST 后要求重定位
  const auto r = sm.Update(recover);
  EXPECT_EQ(r.state, LocalizationState::kRelocalizing);
}

TEST(LocalizationStateMachine, HealthScoreUsesWeakestLink) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs in = GoodTracking();
  in.reg_fitness = 0.1f;             // s_fitness 高
  in.stream.lidar_rate_hz = 2.0;     // s_stream 低（额定 10）
  const auto r = sm.Update(in);
  // 短板制：health_score 应接近 s_stream 而非被高 fitness 掩盖
  EXPECT_LT(r.health_score, 0.5);
}

TEST(LocalizationStateMachine, InfiniteFitnessGivesZeroFitnessScore) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs in = GoodTracking();
  in.reg_fitness = std::numeric_limits<float>::infinity();
  const auto r = sm.Update(in);
  EXPECT_DOUBLE_EQ(r.s_fitness, 0.0);
}

TEST(LocalizationStateMachine, LostClampsCovarianceScaleToMax) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  sm.Update(GoodTracking());
  HealthInputs bad = GoodTracking();
  bad.reg_fitness = std::numeric_limits<float>::infinity();
  localization_health::HealthReport r;
  for (int i = 0; i < 6; ++i) {
    r = sm.Update(bad);
  }
  EXPECT_EQ(r.state, LocalizationState::kLost);
  EXPECT_DOUBLE_EQ(r.covariance_scale, 100.0);
}

TEST(LocalizationStateMachine, ResultTimeoutForcesLost) {
  LocalizationStateMachine sm(FastConfig());
  HealthInputs reloc;
  reloc.has_imu_init = true;
  reloc.map_ready = true;
  sm.Update(reloc);
  sm.Update(GoodTracking());
  HealthInputs stale = GoodTracking();
  stale.last_result_age_us = 2000000u;  // 2s > result_timeout(1s)
  LocalizationState last = LocalizationState::kTracking;
  for (int i = 0; i < 6; ++i) {
    last = sm.Update(stale).state;
  }
  EXPECT_EQ(last, LocalizationState::kLost);
}
```

- [ ] **Step 2: 加 CMake 测试目标**

在 `CMakeLists.txt` 的 `if(BUILD_TESTING)` 块内，紧接 Task 2 追加的目标之后追加：

```cmake
  ament_add_gtest(test_localization_state_machine
    test/test_localization_state_machine.cpp
    src/localization_health/localization_state_machine.cpp)
  target_include_directories(test_localization_state_machine PRIVATE include)
```

- [ ] **Step 3: 运行测试，确认因缺头文件而失败**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai --cmake-target tests 2>&1 | tail -20`
Expected: 编译失败，`localization_state_machine.h: No such file or directory`。

- [ ] **Step 4: 写头文件**

创建 `include/localization_health/localization_state_machine.h`：

```cpp
#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_STATE_MACHINE_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_STATE_MACHINE_H

#include <cstdint>

#include "localization_health/health_types.h"

namespace localization_health {

// 五态降级状态机 + 健康评分。单线程使用（仅系统定时器调用）。
class LocalizationStateMachine {
 public:
  struct Config {
    int degrade_enter_frames = 3;
    int recover_frames = 5;
    int lost_enter_frames = 10;
    double degraded_fitness_thres = 0.5;
    double lost_fitness_thres = 1.0;
    double nominal_lidar_rate_hz = 10.0;
    double nominal_imu_rate_hz = 100.0;
    std::uint64_t result_timeout_us = 1000000u;  // 1s
    double base_covariance = 0.05;
    double max_covariance_scale = 100.0;
  };

  explicit LocalizationStateMachine(const Config& config);

  HealthReport Update(const HealthInputs& in);

  [[nodiscard]] LocalizationState state() const { return state_; }

  void Reset();

 private:
  // 评分（不改状态）
  double ScoreFitness(float fitness) const;
  double ScoreStream(const StreamHealth& s) const;
  double ScoreConsistency(const HealthInputs& in) const;

  // 单帧“是否变差 / 是否失效”判据
  bool IsDegradedFrame(const HealthInputs& in) const;
  bool IsLostFrame(const HealthInputs& in) const;

  static HealthLevel LevelFor(LocalizationState state);

  Config config_;
  LocalizationState state_ = LocalizationState::kInit;
  int degrade_counter_ = 0;
  int recover_counter_ = 0;
  int lost_counter_ = 0;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_STATE_MACHINE_H
```

- [ ] **Step 5: 写实现**

创建 `src/localization_health/localization_state_machine.cpp`：

```cpp
#include "localization_health/localization_state_machine.h"

#include <algorithm>
#include <cmath>

namespace localization_health {

namespace {
double Clamp01(double v) { return std::clamp(v, 0.0, 1.0); }
}  // namespace

LocalizationStateMachine::LocalizationStateMachine(const Config& config)
    : config_(config) {}

void LocalizationStateMachine::Reset() {
  state_ = LocalizationState::kInit;
  degrade_counter_ = 0;
  recover_counter_ = 0;
  lost_counter_ = 0;
}

double LocalizationStateMachine::ScoreFitness(float fitness) const {
  if (!std::isfinite(fitness)) {
    return 0.0;
  }
  return Clamp01(1.0 - static_cast<double>(fitness) / config_.lost_fitness_thres);
}

double LocalizationStateMachine::ScoreStream(const StreamHealth& s) const {
  if (s.lidar_stalled || s.imu_stalled) {
    return 0.0;
  }
  if (!s.rates_known) {
    return 1.0;  // 冷启动不据此判负
  }
  const double sl = Clamp01(s.lidar_rate_hz / config_.nominal_lidar_rate_hz);
  const double si = Clamp01(s.imu_rate_hz / config_.nominal_imu_rate_hz);
  return std::min(sl, si);
}

double LocalizationStateMachine::ScoreConsistency(const HealthInputs& in) const {
  if (in.consistency_required <= 0) {
    return 1.0;
  }
  return Clamp01(static_cast<double>(in.consistency_hit) /
                 static_cast<double>(in.consistency_required));
}

bool LocalizationStateMachine::IsDegradedFrame(const HealthInputs& in) const {
  const bool bad_fitness =
      !std::isfinite(in.reg_fitness) ||
      static_cast<double>(in.reg_fitness) > config_.degraded_fitness_thres;
  const bool stream_warn = in.stream.lidar_stalled || in.stream.imu_stalled ||
                           in.stream.timestamp_jump ||
                           !in.stream.timestamp_monotonic;
  return bad_fitness || stream_warn;
}

bool LocalizationStateMachine::IsLostFrame(const HealthInputs& in) const {
  const bool bad_fitness =
      !std::isfinite(in.reg_fitness) ||
      static_cast<double>(in.reg_fitness) > config_.lost_fitness_thres;
  const bool stalled = in.stream.lidar_stalled;
  const bool result_timeout = in.last_result_age_us > config_.result_timeout_us;
  return bad_fitness || stalled || result_timeout;
}

HealthLevel LocalizationStateMachine::LevelFor(LocalizationState state) {
  switch (state) {
    case LocalizationState::kTracking: return HealthLevel::kOk;
    case LocalizationState::kDegraded: return HealthLevel::kWarn;
    case LocalizationState::kLost: return HealthLevel::kError;
    case LocalizationState::kInit:
    case LocalizationState::kRelocalizing: return HealthLevel::kWarn;
  }
  return HealthLevel::kWarn;
}

HealthReport LocalizationStateMachine::Update(const HealthInputs& in) {
  // 1. 状态迁移
  switch (state_) {
    case LocalizationState::kInit:
      if (in.has_imu_init && in.map_ready) {
        state_ = in.has_init ? LocalizationState::kTracking
                             : LocalizationState::kRelocalizing;
      }
      break;
    case LocalizationState::kRelocalizing:
      if (in.has_init) {
        state_ = LocalizationState::kTracking;
        degrade_counter_ = recover_counter_ = lost_counter_ = 0;
      }
      break;
    case LocalizationState::kTracking:
      if (IsDegradedFrame(in)) {
        if (++degrade_counter_ >= config_.degrade_enter_frames) {
          state_ = LocalizationState::kDegraded;
          degrade_counter_ = 0;
          lost_counter_ = 0;
        }
      } else {
        degrade_counter_ = 0;
      }
      break;
    case LocalizationState::kDegraded:
      if (IsLostFrame(in)) {
        if (++lost_counter_ >= config_.lost_enter_frames) {
          state_ = LocalizationState::kLost;
          lost_counter_ = 0;
          recover_counter_ = 0;
        }
      } else {
        lost_counter_ = 0;
      }
      if (state_ == LocalizationState::kDegraded) {
        if (!IsDegradedFrame(in)) {
          if (++recover_counter_ >= config_.recover_frames) {
            state_ = LocalizationState::kTracking;
            recover_counter_ = 0;
          }
        } else {
          recover_counter_ = 0;
        }
      }
      break;
    case LocalizationState::kLost:
      // 数据流恢复即回退到重定位（A 阶段仅标记状态，不实施主动 re-init）
      if (!in.stream.lidar_stalled &&
          in.last_result_age_us <= config_.result_timeout_us) {
        state_ = in.has_init ? LocalizationState::kTracking
                             : LocalizationState::kRelocalizing;
        degrade_counter_ = recover_counter_ = lost_counter_ = 0;
      }
      break;
  }

  // 2. 评分
  HealthReport r;
  r.state = state_;
  r.level = LevelFor(state_);
  r.s_fitness = ScoreFitness(in.reg_fitness);
  r.s_stream = ScoreStream(in.stream);
  r.s_consistency = ScoreConsistency(in);
  if (state_ == LocalizationState::kRelocalizing) {
    r.health_score = std::min({r.s_fitness, r.s_stream, r.s_consistency});
  } else {
    r.health_score = std::min(r.s_fitness, r.s_stream);
  }

  // 3. 协方差缩放
  if (state_ == LocalizationState::kLost) {
    r.covariance_scale = config_.max_covariance_scale;
  } else if (state_ == LocalizationState::kTracking) {
    r.covariance_scale = 1.0;
  } else {
    const double eps = 1.0e-3;
    r.covariance_scale = std::clamp(
        1.0 / std::max(r.health_score, eps), 1.0, config_.max_covariance_scale);
  }

  r.reason = ToString(state_);
  return r;
}

}  // namespace localization_health
```

- [ ] **Step 6: 运行测试，确认全部通过**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai && colcon test --packages-select lio_sam_hesai --ctest-args -R test_localization_state_machine 2>&1 | tail -25`
Expected: 全部测试 PASS。若 `LostToRelocalizingWhenStreamRecovers` 等用例暴露迟滞/回退语义不符，微调 `Config` 默认或迁移判据顺序，使实现与 spec 的迁移表一致后再通过。

- [ ] **Step 7: 提交**

```bash
git add include/localization_health/localization_state_machine.h src/localization_health/localization_state_machine.cpp test/test_localization_state_machine.cpp CMakeLists.txt
git commit -m "feat(health): add localization state machine with unit tests"
```

---

### Task 4: 扩展 LocalizationResult 并填充 fitness/一致性

**Files:**
- Modify: `include/common/sensor_data_type.h:60-69`
- Modify: `src/slam/localization.cpp` (Init 路径 ~347-358；Tracking 路径 ~560-571)

- [ ] **Step 1: 给 LocalizationResult 增字段**

在 `include/common/sensor_data_type.h` 的 `LocalizationResult` 结构体内（`map_ground_z` 之后、结构体结束前）追加：

```cpp
    // 健康可观测性（默认值保证旧路径/建图侧不受影响）
    float reg_fitness = std::numeric_limits<float>::infinity();
    int consistency_hit = 0;
    int consistency_required = 0;
    bool is_tracking = false;
```

并确保文件顶部有 `#include <limits>`（若无则加在现有 include 区）。

- [ ] **Step 2: Init 路径填充**

在 `src/slam/localization.cpp` 的 `Init()` 中，`init_result` 构造之后、`CacheResultToSystem(init_result);`（当前 [localization.cpp:358](../../../src/slam/localization.cpp#L358)）之前插入：

```cpp
  init_result.reg_fitness = fitness_score;   // 复用本函数已计算的配准得分
  init_result.consistency_hit = consistency_hit_count_;
  init_result.consistency_required = std::max(
      1, ConfigParameters::Instance().localization_sc_consistency_frames_);
  init_result.is_tracking = false;
```

> 说明：`fitness_score` 是 `Init()` 内 [localization.cpp:311](../../../src/slam/localization.cpp#L311) 已算出的局部变量；`consistency_hit_count_` 是 `Localization` 成员。

- [ ] **Step 3: Tracking 路径填充**

在 `src/slam/localization.cpp` 跟踪循环里，`result` 构造之后、`CacheResultToSystem(result);`（当前 [localization.cpp:571](../../../src/slam/localization.cpp#L571)）之前插入：

```cpp
    result.reg_fitness = matcher_->GetFitnessScore(2.0F);
    result.consistency_hit = 0;
    result.consistency_required = 0;
    result.is_tracking = true;
```

> 说明：跟踪阶段不再走一致性确认，故一致性字段留 0（状态机在 `consistency_required<=0` 时把 `s_consistency` 记为 1）。`GetFitnessScore` 与配准同源，代价可忽略。

- [ ] **Step 4: 编译验证**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai 2>&1 | tail -15`
Expected: 编译通过，无告警。

- [ ] **Step 5: 提交**

```bash
git add include/common/sensor_data_type.h src/slam/localization.cpp
git commit -m "feat(health): carry fitness and consistency in LocalizationResult"
```

---

### Task 5: 健康参数组（config + config_parameters + 解析）

**Files:**
- Modify: `include/slam/config_parameters.h` (localization 段之后)
- Modify: `src/slam/system.cpp` (`InitConfigParameters`, ~132 处 localization 段之后)
- Modify: `config/relocalization.yaml`

- [ ] **Step 1: 加参数成员**

在 `include/slam/config_parameters.h` 的 `localization_ground_map_path_` 声明（[config_parameters.h:138](../../../include/slam/config_parameters.h#L138)）之后追加：

```cpp
    // localization health / degradation
    bool health_enable_{true};
    int health_degrade_enter_frames_{3};
    int health_recover_frames_{5};
    int health_lost_enter_frames_{10};
    double health_degraded_fitness_thres_{0.5};
    double health_lost_fitness_thres_{1.0};
    double health_nominal_lidar_rate_hz_{10.0};
    double health_nominal_imu_rate_hz_{100.0};
    double health_lidar_stall_timeout_{0.5};
    double health_imu_stall_timeout_{0.1};
    double health_result_timeout_{1.0};
    double health_base_covariance_{0.05};
    double health_max_covariance_scale_{100.0};
```

- [ ] **Step 2: 加解析**

在 `src/slam/system.cpp` 的 `InitConfigParameters()` 中，localization 段末尾（`localization.ground_map_path` 解析之后，[system.cpp:129-131](../../../src/slam/system.cpp#L129)）追加：

```cpp
  // localization health
  util::param(this, "localization_health.enable", config.health_enable_, true);
  util::param(this, "localization_health.degrade_enter_frames",
              config.health_degrade_enter_frames_, 3);
  util::param(this, "localization_health.recover_frames",
              config.health_recover_frames_, 5);
  util::param(this, "localization_health.lost_enter_frames",
              config.health_lost_enter_frames_, 10);
  util::param(this, "localization_health.degraded_fitness_thres",
              config.health_degraded_fitness_thres_, 0.5);
  util::param(this, "localization_health.lost_fitness_thres",
              config.health_lost_fitness_thres_, 1.0);
  util::param(this, "localization_health.nominal_lidar_rate_hz",
              config.health_nominal_lidar_rate_hz_, 10.0);
  util::param(this, "localization_health.nominal_imu_rate_hz",
              config.health_nominal_imu_rate_hz_, 100.0);
  util::param(this, "localization_health.lidar_stall_timeout",
              config.health_lidar_stall_timeout_, 0.5);
  util::param(this, "localization_health.imu_stall_timeout",
              config.health_imu_stall_timeout_, 0.1);
  util::param(this, "localization_health.result_timeout",
              config.health_result_timeout_, 1.0);
  util::param(this, "localization_health.base_covariance",
              config.health_base_covariance_, 0.05);
  util::param(this, "localization_health.max_covariance_scale",
              config.health_max_covariance_scale_, 100.0);
```

- [ ] **Step 3: 加 YAML 参数组**

在 `config/relocalization.yaml` 的 `localization:` 组之后（`ground_map_path` 行后，[relocalization.yaml:36](../../../config/relocalization.yaml#L36)）追加同级组：

```yaml
    localization_health:
      enable: true                       # 是否发布定位健康诊断与协方差
      degrade_enter_frames: 3            # 进入 DEGRADED 所需连续坏帧
      recover_frames: 5                  # 从 DEGRADED 恢复所需连续好帧（回滞）
      lost_enter_frames: 10              # 进入 LOST 所需连续失效帧
      degraded_fitness_thres: 0.5        # fitness 超此判该帧变差
      lost_fitness_thres: 1.0            # fitness 超此判该帧失效
      nominal_lidar_rate_hz: 10.0        # 雷达额定频率
      nominal_imu_rate_hz: 100.0         # IMU 额定频率
      lidar_stall_timeout: 0.5           # 雷达断流判定/s
      imu_stall_timeout: 0.1             # IMU 断流判定/s
      result_timeout: 1.0                # 定位结果超时（线程挂死判据）/s
      base_covariance: 0.05              # TRACKING 基础位姿协方差对角
      max_covariance_scale: 100.0        # DEGRADED/LOST 协方差放大上限
```

- [ ] **Step 4: 编译验证**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai 2>&1 | tail -15`
Expected: 编译通过。

- [ ] **Step 5: 提交**

```bash
git add include/slam/config_parameters.h src/slam/system.cpp config/relocalization.yaml
git commit -m "feat(health): add localization_health parameter group"
```

---

### Task 6: System 集成（喂数据 + 定时器评估 + 发布诊断/协方差）

**Files:**
- Modify: `include/slam/system.h`
- Modify: `src/slam/system.cpp`
- Modify: `CMakeLists.txt`（relocalization_lib 源）
- Modify: `package.xml`（diagnostic_msgs 依赖）

- [ ] **Step 1: 头文件加成员与发布器**

在 `include/slam/system.h` 顶部 include 区加：

```cpp
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "localization_health/data_stream_watchdog.h"
#include "localization_health/localization_state_machine.h"
```

在 System 私有成员区（publishers 附近，[system.h:87](../../../include/slam/system.h#L87) 之后）加：

```cpp
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
```

在私有成员区（`last_localization_z_offset_` 附近，[system.h:132](../../../include/slam/system.h#L132) 之后）加：

```cpp
  std::unique_ptr<localization_health::DataStreamWatchdog> stream_watchdog_;
  std::unique_ptr<localization_health::LocalizationStateMachine>
      health_state_machine_;
  std::mutex mutex_health_last_result_;
  TimeStampUs last_result_stamp_us_ = 0u;
  bool has_last_result_stamp_ = false;
  float last_reg_fitness_ = std::numeric_limits<float>::infinity();
  int last_consistency_hit_ = 0;
  int last_consistency_required_ = 0;
  bool last_is_tracking_ = false;
```

并加一个私有方法声明（`ProcessLocalizationResultCache` 附近）：

```cpp
  void PublishLocalizationHealth();
```

确保 system.h 顶部已有 `#include <limits>`（若无则加）。

- [ ] **Step 2: 构造健康组件**

在 `src/slam/system.cpp` 的构造流程里（`InitConfigParameters()` 调用之后、`InitPublisher()` 之前——参照 System 构造函数体），插入初始化。先确认构造顺序：

Run: `grep -n "InitConfigParameters()\|InitPublisher()\|InitSubscriber()" src/slam/system.cpp | head`
然后在 `InitConfigParameters();` 之后插入：

```cpp
  {
    const auto& cfg = ConfigParameters::Instance();
    localization_health::DataStreamWatchdog::Config wd_cfg;
    wd_cfg.lidar_stall_timeout_us =
        static_cast<std::uint64_t>(cfg.health_lidar_stall_timeout_ * 1.0e6);
    wd_cfg.imu_stall_timeout_us =
        static_cast<std::uint64_t>(cfg.health_imu_stall_timeout_ * 1.0e6);
    wd_cfg.nominal_lidar_period_us = static_cast<std::uint64_t>(
        1.0e6 / std::max(1.0, cfg.health_nominal_lidar_rate_hz_));
    wd_cfg.nominal_imu_period_us = static_cast<std::uint64_t>(
        1.0e6 / std::max(1.0, cfg.health_nominal_imu_rate_hz_));
    stream_watchdog_ =
        std::make_unique<localization_health::DataStreamWatchdog>(wd_cfg);

    localization_health::LocalizationStateMachine::Config sm_cfg;
    sm_cfg.degrade_enter_frames = cfg.health_degrade_enter_frames_;
    sm_cfg.recover_frames = cfg.health_recover_frames_;
    sm_cfg.lost_enter_frames = cfg.health_lost_enter_frames_;
    sm_cfg.degraded_fitness_thres = cfg.health_degraded_fitness_thres_;
    sm_cfg.lost_fitness_thres = cfg.health_lost_fitness_thres_;
    sm_cfg.nominal_lidar_rate_hz = cfg.health_nominal_lidar_rate_hz_;
    sm_cfg.nominal_imu_rate_hz = cfg.health_nominal_imu_rate_hz_;
    sm_cfg.result_timeout_us =
        static_cast<std::uint64_t>(cfg.health_result_timeout_ * 1.0e6);
    sm_cfg.base_covariance = cfg.health_base_covariance_;
    sm_cfg.max_covariance_scale = cfg.health_max_covariance_scale_;
    health_state_machine_ =
        std::make_unique<localization_health::LocalizationStateMachine>(sm_cfg);
  }
```

确保 system.cpp 顶部有 `#include <algorithm>` 与 `#include <cstdint>`。

- [ ] **Step 3: 创建诊断发布器**

在 `InitPublisher()` 末尾（`localization_odom_pub_` 之后，[system.cpp:299](../../../src/slam/system.cpp#L299)）加：

```cpp
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
```

- [ ] **Step 4: 回调喂时间戳**

在 `LidarMsgCallBack` 开头（进入函数体第一行，[system.cpp:335](../../../src/slam/system.cpp#L335)）加：

```cpp
  if (stream_watchdog_) {
    stream_watchdog_->RecordLidar(RosTimeToUs(cloud_ros_ptr->header));
  }
```

在 `ImuMsgCallBack` 中 `imu_data.timestamp_ = RosTimeToUs(imu_ptr->header);`（[system.cpp:379](../../../src/slam/system.cpp#L379)）之后加：

```cpp
  if (stream_watchdog_) {
    stream_watchdog_->RecordImu(imu_data.timestamp_);
  }
```

- [ ] **Step 5: 缓存最近结果的健康量**

在 `ProcessLocalizationResultCache()` 中，取到 `result` 之后（`localization_results_deque_.clear();` 之后，[system.cpp:568](../../../src/slam/system.cpp#L568) 附近）加：

```cpp
  {
    std::lock_guard<std::mutex> lg(mutex_health_last_result_);
    last_result_stamp_us_ = result.timestamp_;
    has_last_result_stamp_ = true;
    last_reg_fitness_ = result.reg_fitness;
    last_consistency_hit_ = result.consistency_hit;
    last_consistency_required_ = result.consistency_required;
    last_is_tracking_ = result.is_tracking;
  }
```

- [ ] **Step 6: 定时器里评估并发布**

在 `System::Run()` 末尾（[system.cpp:555](../../../src/slam/system.cpp#L555) 的函数结束前）加一行调用：

```cpp
  PublishLocalizationHealth();
```

然后在 `system.cpp` 中新增该方法实现（放在 `ProcessLocalizationResultCache` 之后）：

```cpp
void System::PublishLocalizationHealth() {
  if (!ConfigParameters::Instance().health_enable_ || !stream_watchdog_ ||
      !health_state_machine_) {
    return;
  }

  const TimeStampUs now_us =
      static_cast<TimeStampUs>(this->get_clock()->now().nanoseconds() / 1000);

  localization_health::HealthInputs in;
  in.has_imu_init = has_imu_init_.load();
  in.map_ready = true;  // Localization::Run 启动即加载地图；到此已就绪
  in.now_us = now_us;
  in.stream = stream_watchdog_->Evaluate(now_us);
  {
    std::lock_guard<std::mutex> lg(mutex_health_last_result_);
    in.has_init = has_last_result_stamp_;
    in.is_tracking = last_is_tracking_;
    in.reg_fitness = last_reg_fitness_;
    in.consistency_hit = last_consistency_hit_;
    in.consistency_required = last_consistency_required_;
    in.last_result_age_us =
        (has_last_result_stamp_ && now_us > last_result_stamp_us_)
            ? (now_us - last_result_stamp_us_)
            : 0u;
  }

  const auto report = health_state_machine_->Update(in);

  // 发布 DiagnosticArray
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "lio_sam_hesai: localization";
  st.hardware_id = "lio_sam_hesai_relocalization";
  switch (report.level) {
    case localization_health::HealthLevel::kOk:
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;
    case localization_health::HealthLevel::kWarn:
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    case localization_health::HealthLevel::kError:
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
  }
  st.message = localization_health::ToString(report.state);
  const auto kv = [](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue e;
    e.key = k;
    e.value = std::to_string(v);
    return e;
  };
  st.values.push_back(kv("health_score", report.health_score));
  st.values.push_back(kv("s_fitness", report.s_fitness));
  st.values.push_back(kv("s_stream", report.s_stream));
  st.values.push_back(kv("s_consistency", report.s_consistency));
  st.values.push_back(kv("lidar_rate_hz", in.stream.lidar_rate_hz));
  st.values.push_back(kv("imu_rate_hz", in.stream.imu_rate_hz));
  st.values.push_back(kv("covariance_scale", report.covariance_scale));
  arr.status.push_back(st);
  diagnostics_pub_->publish(arr);
}
```

- [ ] **Step 7: 把协方差写进 odom**

在 `ProcessLocalizationResultCache()` 组装 `mapping_odom` 之后、`localization_odom_pub_->publish(mapping_odom);`（[system.cpp:633](../../../src/slam/system.cpp#L633)）之前插入：

```cpp
  {
    const auto& cfg = ConfigParameters::Instance();
    double scale = 1.0;
    if (cfg.health_enable_ && health_state_machine_) {
      const auto st = health_state_machine_->state();
      if (st == localization_health::LocalizationState::kLost) {
        scale = cfg.health_max_covariance_scale_;
      } else if (st != localization_health::LocalizationState::kTracking) {
        scale = std::min(cfg.health_max_covariance_scale_, 10.0);
      }
    }
    const double pos_var = cfg.health_base_covariance_ * scale;
    const double rot_var = cfg.health_base_covariance_ * scale;
    for (int i = 0; i < 36; ++i) mapping_odom.pose.covariance[i] = 0.0;
    mapping_odom.pose.covariance[0] = pos_var;   // x
    mapping_odom.pose.covariance[7] = pos_var;   // y
    mapping_odom.pose.covariance[14] = pos_var;  // z
    mapping_odom.pose.covariance[21] = rot_var;  // roll
    mapping_odom.pose.covariance[28] = rot_var;  // pitch
    mapping_odom.pose.covariance[35] = rot_var;  // yaw
  }
```

> 说明：这里用状态机的**当前状态**（上一 tick 由 `PublishLocalizationHealth` 更新）做协方差缩放，避免在结果处理路径重复驱动状态机。两者都在同一 `timer_cb_group_`（互斥）内，无并发问题。

- [ ] **Step 8: relocalization_lib 加源文件**

在 `CMakeLists.txt` 的 `add_library(${PROJECT_NAME}_relocalization_lib ...)` 源列表（[CMakeLists.txt:126-151](../../../CMakeLists.txt#L126)）中，`src/slam/system.cpp` 附近加：

```cmake
  src/localization_health/data_stream_watchdog.cpp
  src/localization_health/localization_state_machine.cpp
```

- [ ] **Step 9: package.xml 加依赖**

在 `package.xml` 中 `nav_msgs` 附近加一对：

```xml
  <build_depend>diagnostic_msgs</build_depend>
  <exec_depend>diagnostic_msgs</exec_depend>
```

在 `CMakeLists.txt` 顶部 `find_package` 区加 `find_package(diagnostic_msgs REQUIRED)`，并把 `diagnostic_msgs` 加入 `relocalization_lib` 与 `relocalization` 可执行文件的 `ament_target_dependencies` 列表。

- [ ] **Step 10: 编译整包**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai 2>&1 | tail -20`
Expected: 全部目标编译通过。

- [ ] **Step 11: 提交**

```bash
git add include/slam/system.h src/slam/system.cpp CMakeLists.txt package.xml
git commit -m "feat(health): wire watchdog and state machine into System with diagnostics"
```

---

### Task 7: 全量验证与自检

**Files:**（无新增，验证既有）

- [ ] **Step 1: 全部单元测试通过**

Run: `cd ~/slam_ws && colcon build --packages-select lio_sam_hesai && colcon test --packages-select lio_sam_hesai 2>&1 | tail -30 && colcon test-result --verbose 2>&1 | tail -30`
Expected: 新增 2 个测试文件（watchdog、state machine）全绿；既有测试无回归失败。

- [ ] **Step 2: 参数暴露自检（启动冒烟）**

Run（有地图时）：`ros2 launch lio_sam_hesai localization.launch.py &` 然后 `ros2 param list | grep localization_health` 与 `ros2 topic echo /diagnostics --once`
Expected: `localization_health.*` 参数全部可见；`/diagnostics` 输出一条 name 为 `lio_sam_hesai: localization` 的状态，包含 health_score 等键值。无地图环境下至少确认节点无参数报错启动、`ros2 param list` 含新参数。

- [ ] **Step 3: 清理与最终提交**

删除任何临时文件；确认 `git status` 干净。若前序步骤有未提交改动：

```bash
git add -A && git commit -m "test(health): verify localization health end to end"
```

---

## 自检清单（写计划后执行，非子代理）

- Spec 覆盖：TF 始终发+协方差(Task 7 Step2 + Task6 Step7)、fitness/数据流/一致性信号(Task2/3/4)、DiagnosticArray+协方差输出(Task6)、纯单测(Task2/3)、五态迁移(Task3)、错误处理 inf/断流/超时/时钟回退(Task2/3 测试)——均有对应任务。
- 类型一致：`HealthInputs`/`HealthReport`/`StreamHealth` 字段在 Task1 定义，Task3/6 使用一致；`DataStreamWatchdog::Config`、`LocalizationStateMachine::Config` 字段名 Task2/3/6 一致。
- 占位符：无 TBD/TODO；每个代码步骤含完整代码。

