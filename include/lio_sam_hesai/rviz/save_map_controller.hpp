#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lio_sam_hesai/srv/save_map_options.hpp"

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
  static constexpr float kDefaultResolution = -1.0F;
  static constexpr float kDefaultGroundResolution = -1.0F;

  enum class Outcome
  {
    kSucceeded,
    kFailed,
  };

  /// Completed save: the success flag plus the mapping node's human-readable
  /// response message (the resolved destination on success, or the reason the
  /// request was rejected or failed), so the panel can display what actually
  /// happened.
  struct Result
  {
    Outcome outcome{Outcome::kFailed};
    std::string message;
  };

  explicit SaveMapController(rclcpp::Node::SharedPtr node);

  /// Polls the graph for /lio_sam/save_map_options; true while mapping mode
  /// offers it.
  bool ServiceAvailable() const;

  /// True between accepting a request and receiving its response.
  bool SaveInProgress() const;

  /// Starts an asynchronous save. `destination` must be a non-empty absolute
  /// path; an empty value is rejected here without contacting the service,
  /// while the mapping node validates and reports the path form (root, home,
  /// or symlink) in the response message. Negative resolutions use the
  /// mapping node's configured map_save resolutions. Returns false without
  /// calling the service when it is unavailable, the destination is empty, or
  /// another save is still in flight.
  bool RequestSave(const std::string& destination, bool save_ground_map,
                   bool save_tiled_map,
                   float resolution = kDefaultResolution,
                   float ground_resolution = kDefaultGroundResolution);

  /// Returns and clears the result of the most recent completed save.
  std::optional<Result> TakeResult();

private:
  struct SharedState
  {
    std::mutex mutex;
    bool in_flight{false};
    std::uint64_t sequence{0};
    std::optional<std::int64_t> pending_request_id;
    std::optional<Result> result;
  };

  using SaveMapOptionsClient =
    rclcpp::Client<lio_sam_hesai::srv::SaveMapOptions>;

  SaveMapOptionsClient::SharedPtr client_;
  std::shared_ptr<SharedState> state_;
};

}  // namespace rviz
}  // namespace lio_sam_hesai
