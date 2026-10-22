#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOG_THROTTLE_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOG_THROTTLE_H

#include <cstdint>

namespace localization_health {

// Instance-owned, steady-time based logging gate. It intentionally has no
// shared static state so two Localization/System instances cannot suppress
// each other's diagnostics.
class SteadyLogThrottle {
 public:
  explicit SteadyLogThrottle(std::uint64_t interval_us = 1000000U)
      : interval_us_(interval_us) {}

  [[nodiscard]] bool ShouldLog(std::uint64_t now_us) {
    if (!has_logged_ || now_us < last_log_us_ ||
        now_us - last_log_us_ >= interval_us_) {
      has_logged_ = true;
      last_log_us_ = now_us;
      return true;
    }
    return false;
  }

  void Reset() {
    has_logged_ = false;
    last_log_us_ = 0U;
  }

 private:
  std::uint64_t interval_us_;
  std::uint64_t last_log_us_{0U};
  bool has_logged_{false};
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOG_THROTTLE_H
