#include "lio_sam_hesai/rviz/save_map_controller.hpp"

#include <exception>
#include <memory>
#include <utility>

namespace lio_sam_hesai
{
namespace rviz
{

namespace
{

// Message shown when the mapping node disappears while a save is in flight, so
// the panel can explain the failure instead of silently re-enabling the button.
constexpr char kServiceLostMessage[] =
  "The save_map_options service disappeared before the save finished.";

}  // namespace

SaveMapController::SaveMapController(rclcpp::Node::SharedPtr node)
: client_(node->create_client<lio_sam_hesai::srv::SaveMapOptions>(
    "lio_sam/save_map_options")),
  state_(std::make_shared<SharedState>())
{
}

bool SaveMapController::ServiceAvailable() const
{
  return client_->service_is_ready();
}

bool SaveMapController::SaveInProgress() const
{
  const bool service_available = client_->service_is_ready();
  std::optional<std::int64_t> abandoned_request_id;
  bool in_flight;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->in_flight && !service_available)
    {
      abandoned_request_id = state_->pending_request_id;
      state_->pending_request_id.reset();
      ++state_->sequence;
      state_->in_flight = false;
      state_->result = Result{Outcome::kFailed, kServiceLostMessage};
    }
    in_flight = state_->in_flight;
  }
  if (abandoned_request_id)
  {
    client_->remove_pending_request(*abandoned_request_id);
  }
  return in_flight;
}

bool SaveMapController::RequestSave(
  const std::string& destination, bool save_ground_map, bool save_tiled_map,
  float resolution, float ground_resolution)
{
  if (destination.empty() || !client_->service_is_ready())
  {
    return false;
  }

  std::uint64_t sequence;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->in_flight)
    {
      return false;
    }
    state_->in_flight = true;
    sequence = ++state_->sequence;
    state_->pending_request_id.reset();
    // Drop feedback the GUI has not consumed yet so the next result the panel
    // reads belongs to this request.
    state_->result.reset();
  }

  auto request =
    std::make_shared<lio_sam_hesai::srv::SaveMapOptions::Request>();
  request->destination = destination;
  request->resolution = resolution;
  request->ground_resolution = ground_resolution;
  request->save_ground_map = save_ground_map;
  request->save_tiled_map = save_tiled_map;

  const std::shared_ptr<SharedState> state = state_;
  try
  {
    const auto future = client_->async_send_request(
      request,
      [state, sequence](SaveMapOptionsClient::SharedFuture future)
      {
        bool success = false;
        std::string message;
        try
        {
          const std::shared_ptr<lio_sam_hesai::srv::SaveMapOptions::Response>
            response = future.get();
          success = response->success;
          message = response->message;
        }
        catch (const std::exception& error)
        {
          success = false;
          message = std::string("Save map service call failed: ") + error.what();
        }
        catch (...)
        {
          success = false;
          message = "Save map service call failed";
        }
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->in_flight || state->sequence != sequence)
        {
          return;
        }
        state->result =
          Result{success ? Outcome::kSucceeded : Outcome::kFailed,
                 std::move(message)};
        state->in_flight = false;
        state->pending_request_id.reset();
      });
    bool still_pending;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      still_pending = state_->in_flight && state_->sequence == sequence;
      if (still_pending)
      {
        state_->pending_request_id = future.request_id;
      }
    }
    if (!still_pending)
    {
      client_->remove_pending_request(future.request_id);
    }
  }
  catch (const std::exception&)
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    ++state_->sequence;
    state_->in_flight = false;
    return false;
  }
  return true;
}

std::optional<SaveMapController::Result> SaveMapController::TakeResult()
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  const std::optional<Result> result = state_->result;
  state_->result.reset();
  return result;
}

}  // namespace rviz
}  // namespace lio_sam_hesai
