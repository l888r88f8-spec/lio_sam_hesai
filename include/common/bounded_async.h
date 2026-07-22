#ifndef FUNNY_LIDAR_SLAM_BOUNDED_ASYNC_H
#define FUNNY_LIDAR_SLAM_BOUNDED_ASYNC_H

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <future>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

inline std::size_t AutomaticAsyncWorkerCount(
    std::size_t task_count,
    unsigned int reported_hardware_threads =
        std::thread::hardware_concurrency()) {
  if (task_count == 0U) {
    return 0U;
  }
  const std::size_t available_threads =
      std::max<std::size_t>(1U, reported_hardware_threads);
  return std::min(task_count, available_threads);
}

template <typename Function>
auto RunBoundedAsync(std::size_t task_count, std::size_t worker_count,
                     Function&& task)
    -> std::vector<std::invoke_result_t<std::decay_t<Function>&,
                                        std::size_t>> {
  using Task = std::decay_t<Function>;
  using Result = std::invoke_result_t<Task&, std::size_t>;
  static_assert(!std::is_void_v<Result>,
                "RunBoundedAsync tasks must return a value");

  if (task_count == 0U) {
    return {};
  }

  worker_count = std::clamp<std::size_t>(worker_count, 1U, task_count);
  std::vector<std::optional<Result>> indexed_results(task_count);
  std::atomic_size_t next_index{0U};
  Task task_function(std::forward<Function>(task));

  std::vector<std::future<void>> workers;
  workers.reserve(worker_count);
  for (std::size_t worker = 0; worker < worker_count; ++worker) {
    workers.emplace_back(std::async(std::launch::async, [&]() {
      while (true) {
        const std::size_t index = next_index.fetch_add(1U);
        if (index >= task_count) {
          return;
        }
        indexed_results[index].emplace(task_function(index));
      }
    }));
  }
  for (auto& worker : workers) {
    worker.get();
  }

  std::vector<Result> results;
  results.reserve(task_count);
  for (auto& result : indexed_results) {
    results.emplace_back(std::move(result.value()));
  }
  return results;
}

#endif  // FUNNY_LIDAR_SLAM_BOUNDED_ASYNC_H
