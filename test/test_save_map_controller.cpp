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
#include "lio_sam_hesai/srv/save_map.hpp"

namespace
{

using SaveMap = lio_sam_hesai::srv::SaveMap;
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

// Exercises the controller against an in-process namespaced save_map service
// so the async client, the duplicate-save gate, and the outcome hand-off are
// verified without Qt or a running mapOptimization node.
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
    service_ = service_node_->create_service<SaveMap>(
      "lio_sam/save_map",
      [this](
        std::shared_ptr<SaveMap::Request> request,
        std::shared_ptr<SaveMap::Response> response) {
        ++request_count_;
        last_request_ = *request;
        response->success = respond_success_;
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
  rclcpp::Service<SaveMap>::SharedPtr service_;
  std::unique_ptr<SaveMapController> controller_;
  SaveMap::Request last_request_;
  int request_count_{0};
  bool respond_success_{true};
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
      << "save_map service was never discovered";
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
  EXPECT_FALSE(controller_->RequestSave());
  EXPECT_FALSE(controller_->TakeOutcome().has_value());
}

TEST_F(SaveMapControllerWithoutService, DetectsServiceAppearingLater)
{
  ASSERT_FALSE(controller_->ServiceAvailable());
  ASSERT_FALSE(controller_->RequestSave());

  CreateService();
  ASSERT_TRUE(SpinUntil([this]() { return controller_->ServiceAvailable(); },
                        kDiscoveryTimeout));

  EXPECT_TRUE(controller_->RequestSave());
  EXPECT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));
}

TEST_F(SaveMapControllerWithService, UsesConfiguredValuesWithEmptyDestination)
{
  ASSERT_TRUE(controller_->RequestSave());
  ASSERT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));

  EXPECT_FLOAT_EQ(last_request_.resolution, -1.0F);
  EXPECT_FLOAT_EQ(last_request_.ground_resolution, -1.0F);
  EXPECT_TRUE(last_request_.destination.empty());

  ASSERT_TRUE(SpinUntil([this]() {
    return controller_->TakeOutcome().has_value();
  }));
  EXPECT_FALSE(controller_->TakeOutcome().has_value())
    << "outcome must be consumed exactly once";
}

TEST_F(SaveMapControllerWithService, ReportsSuccessOutcome)
{
  respond_success_ = true;
  ASSERT_TRUE(controller_->RequestSave());

  std::optional<SaveMapController::Outcome> outcome;
  ASSERT_TRUE(SpinUntil([this, &outcome]() {
    outcome = controller_->TakeOutcome();
    return outcome.has_value();
  }));
  EXPECT_EQ(*outcome, SaveMapController::Outcome::kSucceeded);
  EXPECT_FALSE(controller_->SaveInProgress());
}

TEST_F(SaveMapControllerWithService, ReportsFailureOutcome)
{
  respond_success_ = false;
  ASSERT_TRUE(controller_->RequestSave());

  std::optional<SaveMapController::Outcome> outcome;
  ASSERT_TRUE(SpinUntil([this, &outcome]() {
    outcome = controller_->TakeOutcome();
    return outcome.has_value();
  }));
  EXPECT_EQ(*outcome, SaveMapController::Outcome::kFailed);
}

TEST_F(SaveMapControllerWithService, RejectsDuplicateSaveWhileInFlight)
{
  // No executor spin happens between these calls, so the first save cannot
  // have completed when the duplicate is attempted.
  ASSERT_TRUE(controller_->RequestSave());
  EXPECT_TRUE(controller_->SaveInProgress());
  EXPECT_FALSE(controller_->RequestSave());

  ASSERT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));
  SpinFor(std::chrono::milliseconds(100));
  EXPECT_EQ(request_count_, 1);
}

TEST_F(SaveMapControllerWithService, FailsPendingSaveWhenServiceDisappears)
{
  ASSERT_TRUE(controller_->RequestSave());
  ASSERT_TRUE(controller_->SaveInProgress());

  // Remove the server before the executor can handle the request. A response
  // will never arrive, so graph loss must release the button's in-flight gate.
  service_.reset();
  ASSERT_TRUE(SpinUntil([this]() { return !controller_->ServiceAvailable(); },
                        kDiscoveryTimeout));
  EXPECT_FALSE(controller_->SaveInProgress());
  EXPECT_EQ(controller_->TakeOutcome(), SaveMapController::Outcome::kFailed);
}

TEST_F(SaveMapControllerWithService, ForwardsExplicitResolutions)
{
  ASSERT_TRUE(controller_->RequestSave(0.2F, 0.4F));
  ASSERT_TRUE(SpinUntil([this]() { return request_count_ > 0; }));

  EXPECT_FLOAT_EQ(last_request_.resolution, 0.2F);
  EXPECT_FLOAT_EQ(last_request_.ground_resolution, 0.4F);
}

TEST(MapSaveResolution, UsesActiveConfigurationOnlyForNegativeRequests)
{
  EXPECT_FLOAT_EQ(lio_sam_hesai::ResolveMapSaveResolution(-1.0F, 0.32), 0.32F);
  EXPECT_FLOAT_EQ(lio_sam_hesai::ResolveMapSaveResolution(0.0F, 0.32), 0.0F);
  EXPECT_FLOAT_EQ(lio_sam_hesai::ResolveMapSaveResolution(0.2F, 0.32), 0.2F);
}

}  // namespace
