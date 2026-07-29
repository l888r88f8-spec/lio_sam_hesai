#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_REINITIALIZATION_CONTROL_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_REINITIALIZATION_CONTROL_H

#include <atomic>
#include <functional>
#include <mutex>

namespace localization_health {

class ReinitializationControl {
 public:
  class ManualInitializationTransaction {
   public:
    ManualInitializationTransaction(ManualInitializationTransaction&&) noexcept =
        default;
    ManualInitializationTransaction& operator=(
        ManualInitializationTransaction&&) noexcept = default;
    ~ManualInitializationTransaction() = default;

    ManualInitializationTransaction(const ManualInitializationTransaction&) =
        delete;
    ManualInitializationTransaction& operator=(
        const ManualInitializationTransaction&) = delete;

   private:
    friend class ReinitializationControl;

    explicit ManualInitializationTransaction(
        std::unique_lock<std::mutex>&& lock);

    std::unique_lock<std::mutex> lock_;
  };

  class WorkerAttempt {
   public:
    WorkerAttempt(WorkerAttempt&&) noexcept = default;
    WorkerAttempt& operator=(WorkerAttempt&&) noexcept = default;
    ~WorkerAttempt() = default;

    WorkerAttempt(const WorkerAttempt&) = delete;
    WorkerAttempt& operator=(const WorkerAttempt&) = delete;

    [[nodiscard]] bool automatic_relocalization_requested() const;
    [[nodiscard]] bool owns_automatic_initialization() const;
    [[nodiscard]] bool has_manual_initialization() const;
    [[nodiscard]] bool waiting_for_automatic_request() const;

    void RecordInitializationResult(bool initialized);

   private:
    friend class ReinitializationControl;

    WorkerAttempt(ReinitializationControl* control,
                  std::unique_lock<std::mutex>&& lock);

    ReinitializationControl* control_ = nullptr;
    std::unique_lock<std::mutex> lock_;
    bool automatic_relocalization_requested_ = false;
    bool owns_automatic_initialization_ = false;
    bool has_manual_initialization_ = false;
  };

  void RequestAutomaticRelocalization();

  // Tests configure this before starting worker or manual transactions. The
  // observer runs only when a manual transaction contends for mutex_.
  void SetManualContentionObserverForTest(std::function<void()> observer);

  [[nodiscard]] ManualInitializationTransaction BeginManualInitialization();
  [[nodiscard]] WorkerAttempt BeginWorkerAttempt();

 private:
  void NotifyManualContentionForTest();

  std::atomic_bool automatic_relocalization_request_pending_{false};
  std::mutex mutex_;
  bool manual_initialization_pending_ = false;
  std::atomic_bool waiting_for_automatic_relocalization_request_{false};
  std::mutex mutex_manual_contention_observer_;
  std::function<void()> manual_contention_observer_for_test_;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_REINITIALIZATION_CONTROL_H
