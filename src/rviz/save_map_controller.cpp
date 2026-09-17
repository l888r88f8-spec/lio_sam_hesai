#include "lio_sam_hesai/rviz/save_map_controller.hpp"

#include <exception>
#include <memory>

namespace lio_sam_hesai
{
namespace rviz
{

SaveMapController::SaveMapController(rclcpp::Node::SharedPtr node)
: client_(node->create_client<lio_sam_hesai::srv::SaveMap>("lio_sam/save_map")),
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
      state_->outcome = Outcome::kFailed;
    }
    in_flight = state_->in_flight;
  }
  if (abandoned_request_id)
  {
    client_->remove_pending_request(*abandoned_request_id);
  }
  return in_flight;
}

bool SaveMapController::RequestSave(float resolution, float ground_resolution)
{
  if (!client_->service_is_ready())
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
    // Drop feedback the GUI has not consumed yet so the next outcome the panel
    // reads belongs to this request.
    state_->outcome.reset();
  }

  auto request = std::make_shared<lio_sam_hesai::srv::SaveMap::Request>();
  request->resolution = resolution;
  request->ground_resolution = ground_resolution;
  request->destination.clear();

  const std::shared_ptr<SharedState> state = state_;
  try
  {
    const auto future = client_->async_send_request(
      request,
      [state, sequence](SaveMapClient::SharedFuture future)
      {
        bool success = false;
        try
        {
          success = future.get()->success;
        }
        catch (const std::exception&)
        {
          success = false;
        }
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->in_flight || state->sequence != sequence)
        {
          return;
        }
        state->outcome = success ? Outcome::kSucceeded : Outcome::kFailed;
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

std::optional<SaveMapController::Outcome> SaveMapController::TakeOutcome()
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  const std::optional<Outcome> outcome = state_->outcome;
  state_->outcome.reset();
  return outcome;
}

}  // namespace rviz
}  // namespace lio_sam_hesai
