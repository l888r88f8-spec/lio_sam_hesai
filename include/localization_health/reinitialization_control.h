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
    // Manual requests become visible before the worker mutex is acquired.
    // These checks let a worker holding the mutex observe a manual request
    // that is currently blocked waiting for it.
    [[nodiscard]] std::uint64_t manual_cancellation_generation_at_start() const;
    [[nodiscard]] bool manual_cancellation_observed() const;
    [[nodiscard]] bool CanCommitAutomatic() const;

    // Keeps a request-owned automatic relocalization attempt active while
    // Scan Context gathers the remaining consistency frames. This is not a
    // failed initialization and therefore must not wait for stream recovery.
    void RecordAutomaticConsistencyPending();

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
    std::uint64_t manual_cancellation_generation_at_start_ = 0U;
  };

  void RequestAutomaticRelocalization();

  // Tests configure this before starting worker or manual transactions. The
  // observer runs only when a manual transaction contends for mutex_.
  void SetManualContentionObserverForTest(std::function<void()> observer);

  [[nodiscard]] ManualInitializationTransaction BeginManualInitialization();
  [[nodiscard]] WorkerAttempt BeginWorkerAttempt();

  // Lock-free cancellation signal for a worker already inside Init().
  [[nodiscard]] std::uint64_t ManualCancellationGeneration() const {
    return manual_cancellation_generation_.load();
  }
  [[nodiscard]] bool HasPendingManualInitialization() const {
    // True while any published manual request has not yet been represented by
    // a consumed pending pose. A single boolean would let one worker consume
    // the flag while another manual request is still blocked on mutex_.
    return manual_cancellation_generation_.load() >
           manual_initialization_consumed_generation_.load();
  }

 private:
  void NotifyManualContentionForTest();

  std::atomic_bool automatic_relocalization_request_pending_{false};
  // Published before BeginManualInitialization blocks on mutex_.
  std::atomic<std::uint64_t> manual_cancellation_generation_{0U};
  std::atomic<std::uint64_t> manual_initialization_consumed_generation_{0U};
  std::mutex mutex_;
  bool manual_initialization_pending_ = false;
  std::uint64_t manual_initialization_pending_generation_{0U};
  bool automatic_relocalization_in_progress_ = false;
  std::atomic_bool waiting_for_automatic_relocalization_request_{false};
  std::mutex mutex_manual_contention_observer_;
  std::function<void()> manual_contention_observer_for_test_;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_REINITIALIZATION_CONTROL_H
