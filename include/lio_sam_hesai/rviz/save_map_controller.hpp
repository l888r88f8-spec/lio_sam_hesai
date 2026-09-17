#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "lio_sam_hesai/srv/save_map.hpp"

namespace lio_sam_hesai
{
namespace rviz
{

/// Thread-safe, Qt-free state machine behind the mapping RViz "Save Map"
/// button.
///
/// The Qt panel calls this class from the GUI thread while the rclcpp executor
/// runs the service response callback on its own thread. Everything shared
/// between those threads lives in SharedState behind a mutex, and the response
/// callback captures a shared_ptr to that state instead of `this`, so a
/// response that races panel destruction can never touch freed memory.
class SaveMapController
{
public:
  /// A negative value asks mapOptimization to use its active map_save
  /// parameters, including values from a custom launch params_file.
  /// The empty destination likewise uses its configured map_save.directory.
  static constexpr float kDefaultResolution = -1.0F;
  static constexpr float kDefaultGroundResolution = -1.0F;

  enum class Outcome
  {
    kSucceeded,
    kFailed,
  };

  explicit SaveMapController(rclcpp::Node::SharedPtr node);

  /// Polls the graph for /lio_sam/save_map; true while mapping mode offers it.
  bool ServiceAvailable() const;

  /// True between accepting a request and receiving its response.
  bool SaveInProgress() const;

  /// Starts an asynchronous save. Returns false without calling the service
  /// when it is unavailable or another save is still in flight.
  bool RequestSave(float resolution = kDefaultResolution,
                   float ground_resolution = kDefaultGroundResolution);

  /// Returns and clears the outcome of the most recent completed save.
  std::optional<Outcome> TakeOutcome();

private:
  struct SharedState
  {
    std::mutex mutex;
    bool in_flight{false};
    std::uint64_t sequence{0};
    std::optional<std::int64_t> pending_request_id;
    std::optional<Outcome> outcome;
  };

  using SaveMapClient = rclcpp::Client<lio_sam_hesai::srv::SaveMap>;

  SaveMapClient::SharedPtr client_;
  std::shared_ptr<SharedState> state_;
};

}  // namespace rviz
}  // namespace lio_sam_hesai
