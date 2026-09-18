#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "common/map_save_resolution.hpp"
#include "lio_sam_hesai/rviz/save_map_controller.hpp"
#include "lio_sam_hesai/srv/save_map_options.hpp"

namespace
{

using SaveMapOptions = lio_sam_hesai::srv::SaveMapOptions;
using lio_sam_hesai::rviz::SaveMapController;

constexpr auto kSpinSlice = std::chrono::milliseconds(10);
constexpr auto kDiscoveryTimeout = std::chrono::seconds(5);
constexpr auto kCallbackTimeout = std::chrono::seconds(5);

// rclcpp writes its log file below $HOME by default; point it at a writable
// temporary directory so the test also runs in read-only home environments.
void RedirectRosLogsToTempDirectory()
{
  if (std::getenv("ROS_LOG_DIR") != nullptr)
  {
    return;
  }
  const auto log_dir =
    std::filesystem::temp_directory_path() / "lio_sam_hesai_save_map_test_logs";
  std::error_code error;
  std::filesystem::create_directories(log_dir, error);
  if (!error)
  {
    setenv("ROS_LOG_DIR", log_dir.c_str(), 1);
  }
}

// Exercises the controller against an in-process namespaced
// save_map_options service so the async client, the destination/toggle
// forwarding, the duplicate-save gate, the service-loss recovery, and the
// response-message hand-off are verified without Qt or a running
// mapOptimization node.
class SaveMapControllerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    RedirectRosLogsToTempDirectory();
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    const std::string test_namespace =
      "/save_map_controller_test_" + std::to_string(getpid()) + "_" +
      std::to_string(++node_sequence_);
    service_node_ = std::make_shared<rclcpp::Node>(
      "save_map_test_service", test_namespace);
    client_node_ = std::make_shared<rclcpp::Node>(
      "save_map_test_client", test_namespace);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(service_node_);
    executor_->add_node(client_node_);
    controller_ = std::make_unique<SaveMapController>(client_node_);
  }

  void TearDown() override
  {
    controller_.reset();
    if (executor_)
    {
      executor_->remove_node(service_node_);
      executor_->remove_node(client_node_);
      executor_.reset();
    }
    service_.reset();
    client_node_.reset();
    service_node_.reset();
  }

  void CreateService()
  {
    service_ = service_node_->create_service<SaveMapOptions>(
      "lio_sam/save_map_options",
      [this](
        std::shared_ptr<SaveMapOptions::Request> request,
        std::shared_ptr<SaveMapOptions::Response> response) {
        ++request_count_;
        last_request_ = *request;
        response->success = respond_success_;
        response->message = respond_message_;
      });
  }

  void SpinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_->spin_some(kSpinSlice);
    }
  }

  bool SpinUntil(
    const std::function<bool()>& predicate,
    std::chrono::milliseconds timeout = kCallbackTimeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_->spin_some(kSpinSlice);
      if (predicate())
      {
        return true;
      }
    }
    return predicate();
  }

  static std::size_t node_sequence_;

  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Service<SaveMapOptions>::SharedPtr service_;
  std::unique_ptr<SaveMapController> controller_;
  SaveMapOptions::Request last_request_;
  int request_count_{0};
  bool respond_success_{true};
  std::string respond_message_{"Saved map to /tmp/lio_sam_site"};
};

std::size_t SaveMapControllerFixture::node_sequence_ = 0;

class SaveMapControllerWithService : public SaveMapControllerFixture
{
protected:
  void SetUp() override
  {
    SaveMapControllerFixture::SetUp();
    CreateService();
    ASSERT_TRUE(SpinUntil([this]() { return controller_->ServiceAvailable(); },
                          kDiscoveryTimeout))
      << "save_map_options service was never discovered";
  }
};

class SaveMapControllerWithoutService : public SaveMapControllerFixture
{
};

TEST_F(SaveMapControllerWithoutService, ReportsUnavailableAndSendsNoRequest)
{
  SpinFor(std::chrono::milliseconds(300));
  EXPECT_FALSE(controller_->ServiceAvailable());
  EXPECT_FALSE(controller_->SaveInProgress());
  EXPECT_FALSE(controller_->RequestSave("/tmp/lio_sam_site", true, true));
  EXPECT_FALSE(controller_->TakeResult().has_value());
}

TEST_F(SaveMapControllerWithoutService, DetectsServiceAppearingLater)
{
  ASSERT_FALSE(controller_->ServiceAvailable());
  ASSERT_FALSE(controller_->RequestSave("/tmp/lio_sam_site", true, true));

  CreateService();
  ASSERT_TRUE(SpinUntil([this]() { return controller_->ServiceAvailable(); },
                        kDiscoveryTimeout));

  EXPECT_TRUE(controller_->RequestSave("/tmp/lio_sam_site", true, true));
  EXPECT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));
}

TEST_F(SaveMapControllerWithService,
       ForwardsDestinationTogglesAndConfiguredResolutions)
{
  ASSERT_TRUE(controller_->RequestSave(
    "/tmp/lio_sam_site", /*save_ground_map=*/true, /*save_tiled_map=*/false));
  ASSERT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));

  EXPECT_EQ(last_request_.destination, "/tmp/lio_sam_site");
  EXPECT_TRUE(last_request_.save_ground_map);
  EXPECT_FALSE(last_request_.save_tiled_map);
  EXPECT_FLOAT_EQ(last_request_.resolution, -1.0F);
  EXPECT_FLOAT_EQ(last_request_.ground_resolution, -1.0F);

  ASSERT_TRUE(SpinUntil(
    [this]() { return controller_->TakeResult().has_value(); }));
  EXPECT_FALSE(controller_->TakeResult().has_value())
    << "result must be consumed exactly once";
}

TEST_F(SaveMapControllerWithService, ForwardsExplicitResolutions)
{
  ASSERT_TRUE(controller_->RequestSave(
    "/tmp/lio_sam_site", /*save_ground_map=*/false, /*save_tiled_map=*/true,
    /*resolution=*/0.2F, /*ground_resolution=*/0.4F));
  ASSERT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));

  EXPECT_EQ(last_request_.destination, "/tmp/lio_sam_site");
  EXPECT_FALSE(last_request_.save_ground_map);
  EXPECT_TRUE(last_request_.save_tiled_map);
  EXPECT_FLOAT_EQ(last_request_.resolution, 0.2F);
  EXPECT_FLOAT_EQ(last_request_.ground_resolution, 0.4F);
}

TEST_F(SaveMapControllerWithService, RejectsEmptyDestinationWithoutSendingRequest)
{
  EXPECT_FALSE(controller_->RequestSave("", true, true));
  SpinFor(std::chrono::milliseconds(100));

  EXPECT_EQ(request_count_, 0);
  EXPECT_FALSE(controller_->SaveInProgress());
}

TEST_F(SaveMapControllerWithService, ReportsSuccessOutcomeWithResponseMessage)
{
  respond_success_ = true;
  respond_message_ = "Saved map to /tmp/lio_sam_site";
  ASSERT_TRUE(controller_->RequestSave("/tmp/lio_sam_site", true, true));

  std::optional<SaveMapController::Result> result;
  ASSERT_TRUE(SpinUntil([this, &result]() {
    result = controller_->TakeResult();
    return result.has_value();
  }));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->outcome, SaveMapController::Outcome::kSucceeded);
  EXPECT_EQ(result->message, "Saved map to /tmp/lio_sam_site");
  EXPECT_FALSE(controller_->SaveInProgress());
}

TEST_F(SaveMapControllerWithService, ReportsFailureOutcomeWithResponseMessage)
{
  respond_success_ = false;
  respond_message_ =
    "save destination must not be the home directory: /home/robot";
  ASSERT_TRUE(controller_->RequestSave("/home/robot", true, true));

  std::optional<SaveMapController::Result> result;
  ASSERT_TRUE(SpinUntil([this, &result]() {
    result = controller_->TakeResult();
    return result.has_value();
  }));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->outcome, SaveMapController::Outcome::kFailed);
  EXPECT_EQ(
    result->message,
    "save destination must not be the home directory: /home/robot");
}

TEST_F(SaveMapControllerWithService, RejectsDuplicateSaveWhileInFlight)
{
  // No executor spin happens between these calls, so the first save cannot
  // have completed when the duplicate is attempted.
  ASSERT_TRUE(controller_->RequestSave("/tmp/lio_sam_site", true, true));
  EXPECT_TRUE(controller_->SaveInProgress());
  EXPECT_FALSE(controller_->RequestSave("/tmp/lio_sam_site", true, true));

  ASSERT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));
  SpinFor(std::chrono::milliseconds(100));
  EXPECT_EQ(request_count_, 1);
}

TEST_F(SaveMapControllerWithService, FailsPendingSaveWhenServiceDisappears)
{
  ASSERT_TRUE(controller_->RequestSave("/tmp/lio_sam_site", true, true));
  ASSERT_TRUE(controller_->SaveInProgress());

  // Remove the server before the executor can handle the request. A response
  // will never arrive, so graph loss must release the button's in-flight gate
  // and explain the failure.
  service_.reset();
  ASSERT_TRUE(SpinUntil([this]() { return !controller_->ServiceAvailable(); },
                        kDiscoveryTimeout));
  EXPECT_FALSE(controller_->SaveInProgress());

  const std::optional<SaveMapController::Result> result =
    controller_->TakeResult();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->outcome, SaveMapController::Outcome::kFailed);
  EXPECT_NE(result->message.find("save_map_options"), std::string::npos)
    << "service-loss failure must name the missing service: " << result->message;
}

TEST(MapSaveResolution, UsesActiveConfigurationOnlyForNegativeRequests)
{
  EXPECT_FLOAT_EQ(lio_sam_hesai::ResolveMapSaveResolution(-1.0F, 0.32), 0.32F);
  EXPECT_FLOAT_EQ(lio_sam_hesai::ResolveMapSaveResolution(0.0F, 0.32), 0.0F);
  EXPECT_FLOAT_EQ(lio_sam_hesai::ResolveMapSaveResolution(0.2F, 0.32), 0.2F);
}

}  // namespace
