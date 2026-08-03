#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

std::string ReadProjectFile(const std::string & relative_path)
{
  std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

template<std::size_t FileCount, std::size_t TokenCount>
void ExpectTokens(
  const std::array<std::string, FileCount> & files,
  const std::array<std::string, TokenCount> & tokens)
{
  for (const auto & token : tokens) {
    for (const auto & file : files) {
      EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
        << token << " is missing from " << file;
    }
  }
}

void ExpectToken(const std::string & file, const std::string & token)
{
  EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
    << token << " is missing from " << file;
}

void ExpectNoToken(const std::string & file, const std::string & token)
{
  EXPECT_EQ(ReadProjectFile(file).find(token), std::string::npos)
    << token << " must not appear in " << file;
}

std::string ReadFunctionBody(
  const std::string & file,
  const std::string & signature)
{
  const std::string text = ReadProjectFile(file);
  const std::size_t signature_start = text.find(signature);
  EXPECT_NE(signature_start, std::string::npos)
    << signature << " is missing from " << file;
  if (signature_start == std::string::npos) {
    return {};
  }

  const std::size_t body_start = text.find('{', signature_start);
  EXPECT_NE(body_start, std::string::npos)
    << signature << " has no body in " << file;
  if (body_start == std::string::npos) {
    return {};
  }

  std::size_t depth = 1;
  for (std::size_t cursor = body_start + 1; cursor < text.size(); ++cursor) {
    if (text[cursor] == '{') {
      ++depth;
    } else if (text[cursor] == '}' && --depth == 0) {
      return text.substr(body_start + 1, cursor - body_start - 1);
    }
  }

  ADD_FAILURE() << signature << " body is not balanced in " << file;
  return {};
}

void ExpectQuotedParameter(
  const std::string & file,
  const std::string & parameter_name)
{
  EXPECT_NE(
    ReadProjectFile(file).find("\"" + parameter_name + "\""),
    std::string::npos)
    << parameter_name << " is missing from " << file;
}

std::size_t FindYamlBlockEnd(
  const std::string & text,
  std::size_t block_start,
  std::size_t indentation)
{
  std::size_t line_start = text.find('\n', block_start + 1);
  while (line_start != std::string::npos && line_start + 1 < text.size()) {
    ++line_start;
    const std::size_t line_end = text.find('\n', line_start);
    const std::size_t first = text.find_first_not_of(' ', line_start);
    if (first != std::string::npos &&
      (line_end == std::string::npos || first < line_end) &&
      text[first] != '#' && first - line_start <= indentation)
    {
      return line_start;
    }
    line_start = line_end;
  }
  return text.size();
}

void ExpectMappingYamlParameter(const std::string & leaf)
{
  const std::string text = ReadProjectFile("config/mapping.yaml");
  const std::size_t map_save = text.find("\n    map_save:");
  ASSERT_NE(map_save, std::string::npos)
    << "map_save is missing from config/mapping.yaml";
  const std::size_t map_save_end = FindYamlBlockEnd(text, map_save, 4);
  const std::size_t yaml_leaf = text.find("\n      " + leaf + ":", map_save);
  EXPECT_TRUE(yaml_leaf != std::string::npos && yaml_leaf < map_save_end)
    << "map_save." << leaf << " is missing from config/mapping.yaml";
}

void ExpectLocalizationHealthYamlParameter(const std::string & leaf)
{
  const std::string text = ReadProjectFile("config/relocalization.yaml");
  const std::size_t health = text.find("\n    localization_health:");
  ASSERT_NE(health, std::string::npos)
    << "localization_health is missing from config/relocalization.yaml";
  const std::size_t health_end = FindYamlBlockEnd(text, health, 4);
  const std::size_t yaml_leaf = text.find("\n      " + leaf + ":", health);
  EXPECT_TRUE(yaml_leaf != std::string::npos && yaml_leaf < health_end)
    << "localization_health." << leaf
    << " is missing from config/relocalization.yaml";
}

void ExpectRuntimeYamlParameter(const std::string & leaf)
{
  const std::string text = ReadProjectFile("config/relocalization.yaml");
  const std::size_t runtime = text.find("\n    runtime:");
  ASSERT_NE(runtime, std::string::npos)
    << "runtime is missing from config/relocalization.yaml";
  const std::size_t runtime_end = FindYamlBlockEnd(text, runtime, 4);
  const std::size_t yaml_leaf = text.find("\n      " + leaf + ":", runtime);
  EXPECT_TRUE(yaml_leaf != std::string::npos && yaml_leaf < runtime_end)
    << "runtime." << leaf << " is missing from config/relocalization.yaml";
}

}  // namespace

TEST(AutoRelocalizationParameterSurface, ExposesAllRuntimeParameters) {
  const std::array<std::string, 3> files = {
    "include/slam/config_parameters.h",
    "src/slam/system.cpp",
    "config/relocalization.yaml",
  };
  const std::array<std::string, 7> tokens = {
    "auto_relocalization",
    "sc_db_path",
    "sc_top_n",
    "sc_fitness_thres",
    "sc_consistency_frames",
    "sc_consistency_trans_thres",
    "sc_consistency_yaw_thres",
  };

  ExpectTokens(files, tokens);
}

TEST(AutoRelocalizationParameterSurface, ExposesMappingDatabaseExport) {
  const std::array<std::string, 3> canonical_parameters = {
    "map_save.generate_scan_context_db",
    "map_save.align_ground_to_zero",
    "map_save.ground_z_offset",
  };
  const std::array<std::string, 3> yaml_leaves = {
    "generate_scan_context_db",
    "align_ground_to_zero",
    "ground_z_offset",
  };

  for (const auto & parameter : canonical_parameters) {
    ExpectQuotedParameter("include/lio_sam/utility.hpp", parameter);
  }
  for (const auto & leaf : yaml_leaves) {
    ExpectMappingYamlParameter(leaf);
  }

  const std::array<std::string, 9> tokens = {
    "generateScDb",
    "relocalization/scan_context.h",
    "ScanContext",
    "sc_db.bin",
    "SaveDB",
    "alignSavedMapGroundToZero",
    "savedMapGroundZOffset",
    "pclPointToAffine3f",
    "keyFramePose(2, 3) += savedMapGroundZOffset",
  };

  for (const auto & token : tokens) {
    ExpectToken("src/mapOptimization.cpp", token);
  }
}

TEST(AutoRelocalizationParameterSurface, ExposesLocalizationHealthParameters) {
  struct HealthParameter {
    std::string leaf;
    std::string member;
  };
  const std::array<HealthParameter, 14> parameters = {{
    {"enable", "health_enable_"},
    {"degrade_enter_frames", "health_degrade_enter_frames_"},
    {"recover_frames", "health_recover_frames_"},
    {"lost_enter_frames", "health_lost_enter_frames_"},
    {"degraded_fitness_thres", "health_degraded_fitness_thres_"},
    {"lost_fitness_thres", "health_lost_fitness_thres_"},
    {"nominal_lidar_rate_hz", "health_nominal_lidar_rate_hz_"},
    {"nominal_imu_rate_hz", "health_nominal_imu_rate_hz_"},
    {"min_rate_ratio", "health_min_rate_ratio_"},
    {"lidar_stall_timeout", "health_lidar_stall_timeout_"},
    {"imu_stall_timeout", "health_imu_stall_timeout_"},
    {"result_timeout", "health_result_timeout_"},
    {"base_covariance", "health_base_covariance_"},
    {"max_covariance_scale", "health_max_covariance_scale_"},
  }};

  for (const auto & parameter : parameters) {
    ExpectToken("include/slam/config_parameters.h", parameter.member);
    ExpectQuotedParameter(
      "src/slam/system.cpp", "localization_health." + parameter.leaf);
    ExpectLocalizationHealthYamlParameter(parameter.leaf);
  }
}

TEST(SystemRuntimeResourceSurface, ConfiguresWorkersBeforeTheirRunLoops) {
  struct RuntimeParameter {
    std::string leaf;
    std::string member;
  };
  const std::array<RuntimeParameter, 6> parameters = {{
    {"enable_realtime_scheduling", "runtime_enable_realtime_scheduling_"},
    {"preprocessing_fifo_priority", "runtime_preprocessing_fifo_priority_"},
    {"localization_fifo_priority", "runtime_localization_fifo_priority_"},
    {"preprocessing_cpu_core", "runtime_preprocessing_cpu_core_"},
    {"localization_cpu_core", "runtime_localization_cpu_core_"},
    {"frame_processing_timeout_ms", "runtime_frame_processing_timeout_ms_"},
  }};

  for (const auto & parameter : parameters) {
    ExpectToken("include/slam/config_parameters.h", parameter.member);
    ExpectQuotedParameter("src/slam/system.cpp", "runtime." + parameter.leaf);
    ExpectRuntimeYamlParameter(parameter.leaf);
  }

  ExpectToken(
    "include/slam/system.h", "runtime/thread_runtime_controller.h");
  ExpectToken(
    "include/slam/system.h", "preprocessing_thread_runtime_");
  ExpectToken(
    "include/slam/system.h", "localization_thread_runtime_");

  const std::string constructor_body = ReadFunctionBody(
    "src/slam/system.cpp", "System::System(const rclcpp::NodeOptions& options)");
  const std::size_t validation = constructor_body.find(
    "runtime::ThreadRuntimeController::Validate");
  const std::size_t localization_worker = constructor_body.find(
    "localization_thread_ptr_ =");
  const std::size_t preprocessing_worker = constructor_body.find(
    "pre_processing_thread_ptr_ =");
  ASSERT_NE(validation, std::string::npos);
  ASSERT_NE(localization_worker, std::string::npos);
  ASSERT_NE(preprocessing_worker, std::string::npos);
  EXPECT_LT(validation, localization_worker);
  EXPECT_LT(validation, preprocessing_worker);

  const std::size_t localization_apply = constructor_body.find(
    "localization_thread_runtime_->ApplyToCurrentThread();");
  const std::size_t localization_run = constructor_body.find(
    "localization_ptr_->Run();");
  const std::size_t preprocessing_apply = constructor_body.find(
    "preprocessing_thread_runtime_->ApplyToCurrentThread();");
  const std::size_t preprocessing_run = constructor_body.find(
    "pre_processing_ptr_->Run();");
  ASSERT_NE(localization_apply, std::string::npos);
  ASSERT_NE(localization_run, std::string::npos);
  ASSERT_NE(preprocessing_apply, std::string::npos);
  ASSERT_NE(preprocessing_run, std::string::npos);
  EXPECT_LT(localization_apply, localization_run);
  EXPECT_LT(preprocessing_apply, preprocessing_run);

  const std::array<std::string, 12> diagnostic_keys = {
    "preprocessing_fifo_priority",
    "preprocessing_cpu_core",
    "preprocessing_fifo_applied",
    "preprocessing_fifo_errno",
    "preprocessing_affinity_applied",
    "preprocessing_affinity_errno",
    "localization_fifo_priority",
    "localization_cpu_core",
    "localization_fifo_applied",
    "localization_fifo_errno",
    "localization_affinity_applied",
    "localization_affinity_errno",
  };
  ExpectToken("include/slam/system.h", "void PublishRuntimeDiagnostics()");
  ExpectToken("include/slam/system.h", "void AppendRuntimeDiagnostics(");

  const std::string diagnostics_body = ReadFunctionBody(
    "src/slam/system.cpp", "void System::PublishLocalizationHealth()");
  const std::size_t health_disabled = diagnostics_body.find(
    "if (!config.health_enable_)");
  const std::size_t publish_runtime = diagnostics_body.find(
    "PublishRuntimeDiagnostics();", health_disabled);
  ASSERT_NE(health_disabled, std::string::npos);
  ASSERT_NE(publish_runtime, std::string::npos);
  EXPECT_LT(health_disabled, publish_runtime);

  const std::string runtime_diagnostics_body = ReadFunctionBody(
    "src/slam/system.cpp", "void System::PublishRuntimeDiagnostics()");
  EXPECT_NE(runtime_diagnostics_body.find("diagnostics_pub_"), std::string::npos);
  EXPECT_NE(
    runtime_diagnostics_body.find("AppendRuntimeDiagnostics(status);"),
    std::string::npos);

  const std::string append_runtime_body = ReadFunctionBody(
    "src/slam/system.cpp", "void System::AppendRuntimeDiagnostics(");
  for (const auto & key : diagnostic_keys) {
    EXPECT_NE(append_runtime_body.find(key), std::string::npos);
  }
}

TEST(FrameProcessingPipelineSurface, DropsOverdueFramesBeforeStateCommit) {
  const std::array<std::string, 2> worker_headers = {
    "include/slam/preprocessing.h",
    "include/slam/localization.h",
  };
  const std::array<std::string, 3> worker_tokens = {
    "runtime/frame_processing_watchdog.h",
    "runtime::FrameProcessingWatchdog frame_processing_watchdog_",
    "FrameProcessingSnapshot() const",
  };
  ExpectTokens(worker_headers, worker_tokens);

  ExpectToken(
    "include/slam/system.h", "FrameProcessingTimeoutUs() const");
  ExpectToken(
    "src/slam/system.cpp", "FrameProcessingTimeoutUs() const");

  const std::string preprocessing_run = ReadFunctionBody(
    "src/slam/preprocessing.cpp", "void PreProcessing::Run()");
  EXPECT_NE(preprocessing_run.find("frame_processing_watchdog_.Begin("),
    std::string::npos);
  const std::size_t preprocessing_deadline = preprocessing_run.find(
    "frame_processing_watchdog_.Exceeded(");
  const std::size_t preprocessing_enqueue = preprocessing_run.find(
    "cloud_cluster_deque_.emplace_back");
  ASSERT_NE(preprocessing_deadline, std::string::npos);
  ASSERT_NE(preprocessing_enqueue, std::string::npos);
  EXPECT_LT(preprocessing_deadline, preprocessing_enqueue);
  EXPECT_NE(preprocessing_run.find("Complete(now_us, true)"),
    std::string::npos);
  EXPECT_NE(preprocessing_run.find("Complete(now_us, false)"),
    std::string::npos);

  const std::string localization_run = ReadFunctionBody(
    "src/slam/localization.cpp", "void Localization::Run()");
  EXPECT_NE(localization_run.find("frame_processing_watchdog_.Begin("),
    std::string::npos);
  const std::array<std::string, 4> localization_tokens = {
    "DiscardTimedOutFrameIfNeeded()",
    "LoadLocalMap(",
    "matcher_->Match(",
    "Optimize(",
  };
  for (const auto & token : localization_tokens) {
    EXPECT_NE(localization_run.find(token), std::string::npos);
  }
  const std::size_t discard = localization_run.find(
    "DiscardTimedOutFrameIfNeeded()");
  const std::size_t cache_result = localization_run.find("CacheResultToSystem(");
  ASSERT_NE(discard, std::string::npos);
  ASSERT_NE(cache_result, std::string::npos);
  EXPECT_LT(discard, cache_result);

  const std::string discard_body = ReadFunctionBody(
    "src/slam/localization.cpp",
    "bool Localization::DiscardTimedOutFrameIfNeeded(");
  const std::array<std::string, 6> discard_tokens = {
    "frame_processing_watchdog_.Exceeded(",
    "Complete(",
    "InitMatcher()",
    "InitPreintegration()",
    "need_update_local_map_ = true",
    "delta_pose_ = Mat4d::Identity()",
  };
  for (const auto & token : discard_tokens) {
    EXPECT_NE(discard_body.find(token), std::string::npos);
  }

  const std::size_t rebuild_preintegration = discard_body.find(
    "InitPreintegration()");
  const std::size_t restore_navigation_bias = discard_body.find(
    "pre_integration_ptr_->SetGyroAccBias(last_nav_state_.bg_,");
  const std::size_t restore_odometry_bias = discard_body.find(
    "odom_pre_integration_ptr_->SetGyroAccBias(last_odom_state_.bg_,");
  ASSERT_NE(rebuild_preintegration, std::string::npos);
  ASSERT_NE(restore_navigation_bias, std::string::npos);
  ASSERT_NE(restore_odometry_bias, std::string::npos);
  EXPECT_LT(rebuild_preintegration, restore_navigation_bias);
  EXPECT_LT(rebuild_preintegration, restore_odometry_bias);
  EXPECT_NE(discard_body.find("last_nav_state_.ba_", restore_navigation_bias),
    std::string::npos);
  EXPECT_NE(discard_body.find("last_odom_state_.ba_", restore_odometry_bias),
    std::string::npos);

  const std::string initialization_body = ReadFunctionBody(
    "src/slam/localization.cpp",
    "Localization::InitializationResult Localization::Init()");
  const std::size_t initialization_deadline = initialization_body.find(
    "DiscardTimedOutFrameIfNeeded(initialization_commit_now_us)");
  ASSERT_NE(initialization_deadline, std::string::npos);
  const std::array<std::string, 7> initialization_commit_tokens = {
    "curr_pose_ =",
    "last_nav_state_ =",
    "delta_pose_ =",
    "last_pose_ =",
    "last_odom_state_ =",
    "CacheResultToSystem(",
    "UpdateCurrentLidarCloud(",
  };
  for (const auto & token : initialization_commit_tokens) {
    const std::size_t commit = initialization_body.find(token);
    ASSERT_NE(commit, std::string::npos) << token;
    EXPECT_LT(initialization_deadline, commit)
      << token << " must remain a local candidate until the deadline passes";
  }

  const std::size_t initialization_branch = localization_run.find(
    "const InitializationResult initialization_result = Init()");
  const std::size_t tracking_branch = localization_run.find(
    "if (curr_cloud_cluster_ptr_->imu_data_.size() < 2)");
  ASSERT_NE(initialization_branch, std::string::npos);
  ASSERT_NE(tracking_branch, std::string::npos);
  EXPECT_EQ(
    localization_run.substr(initialization_branch,
      tracking_branch - initialization_branch).find(
        "DiscardTimedOutFrameIfNeeded()"),
    std::string::npos)
    << "Init() owns the final deadline check before committing initialization";

  ExpectToken("include/slam/localization.h", "force_local_map_rebuild_");
  EXPECT_NE(discard_body.find("force_local_map_rebuild_ = true"),
    std::string::npos);
  const std::size_t force_local_map_rebuild = localization_run.find(
    "const bool force_local_map_rebuild = force_local_map_rebuild_");
  const std::size_t load_local_map = localization_run.find("LoadLocalMap(");
  const std::size_t force_rebuild_argument = localization_run.find(
    "force_local_map_rebuild);", load_local_map);
  const std::size_t clear_forced_local_map = localization_run.find(
    "force_local_map_rebuild_ = false", load_local_map);
  const std::size_t add_local_map = localization_run.find(
    "matcher_->AddCloudToLocalMap({*local_map})", load_local_map);
  ASSERT_NE(force_local_map_rebuild, std::string::npos);
  ASSERT_NE(load_local_map, std::string::npos);
  ASSERT_NE(force_rebuild_argument, std::string::npos);
  ASSERT_NE(clear_forced_local_map, std::string::npos);
  ASSERT_NE(add_local_map, std::string::npos);
  EXPECT_LT(force_local_map_rebuild, load_local_map);
  EXPECT_LT(load_local_map, force_rebuild_argument);
  EXPECT_LT(force_rebuild_argument, add_local_map);
  EXPECT_LT(add_local_map, clear_forced_local_map);

  ExpectToken(
    "include/slam/localization.h",
    "LoadLocalMap(const Mat4d& pose, bool force_rebuild");
  const std::string load_local_map_body = ReadFunctionBody(
    "src/slam/localization.cpp",
    "PCLPointCloudXYZI::Ptr Localization::LoadLocalMap(");
  const std::size_t force_rebuild_input = load_local_map_body.find(
    "need_update_local_map_ = force_rebuild");
  const std::size_t global_map_early_return = load_local_map_body.find(
    "if (!need_update_local_map_)");
  const std::size_t global_crop = load_local_map_body.find("crop_box.filter(");
  ASSERT_NE(force_rebuild_input, std::string::npos);
  ASSERT_NE(global_map_early_return, std::string::npos);
  ASSERT_NE(global_crop, std::string::npos);
  EXPECT_LT(force_rebuild_input, global_map_early_return);
  EXPECT_LT(global_map_early_return, global_crop);

  EXPECT_NE(initialization_body.find(
    "NavStateData initialized_nav_state = last_nav_state_"),
    std::string::npos);
  EXPECT_NE(initialization_body.find(
    "NavStateData initialized_odom_state = last_odom_state_"),
    std::string::npos);
  EXPECT_EQ(initialization_body.find("initialized_odom_state.bg_ ="),
    std::string::npos);
  EXPECT_EQ(initialization_body.find("initialized_odom_state.ba_ ="),
    std::string::npos);
  EXPECT_EQ(initialization_body.find("initialized_nav_state.bg_ ="),
    std::string::npos);
  EXPECT_EQ(initialization_body.find("initialized_nav_state.ba_ ="),
    std::string::npos);

  const std::size_t initialization_complete = initialization_body.find(
    "Complete(initialization_commit_now_us, false)");
  const std::size_t initialized_lidar_cloud = initialization_body.find(
    "const auto initialized_lidar_cloud");
  const std::size_t initialization_ground_estimate = initialization_body.find(
    "EstimateMapGroundZ(");
  const std::array<std::string, 7> initialization_atomic_commit_tokens = {
    "curr_pose_ =",
    "last_nav_state_ =",
    "delta_pose_ =",
    "last_pose_ =",
    "last_odom_state_ =",
    "CacheResultToSystem(",
    "UpdateCurrentLidarCloud(",
  };
  ASSERT_NE(initialization_complete, std::string::npos);
  ASSERT_NE(initialized_lidar_cloud, std::string::npos);
  ASSERT_NE(initialization_ground_estimate, std::string::npos);
  EXPECT_LT(initialization_ground_estimate, initialization_complete);
  EXPECT_LT(initialized_lidar_cloud, initialization_complete);
  for (const auto & token : initialization_atomic_commit_tokens) {
    const std::size_t commit = initialization_body.find(token);
    ASSERT_NE(commit, std::string::npos) << token;
    EXPECT_LT(initialization_complete, commit);
  }

  const std::size_t tracking_complete = localization_run.find(
    "Complete(tracking_commit_now_us, false)");
  const std::size_t tracking_lidar_cloud = localization_run.find(
    "const auto current_lidar_cloud");
  const std::size_t tracking_ground_estimate = localization_run.find(
    "EstimateMapGroundZ(", tracking_lidar_cloud == std::string::npos ? 0 :
      localization_run.rfind("if (ConfigParameters::Instance()", tracking_lidar_cloud));
  const std::array<std::string, 6> tracking_atomic_commit_tokens = {
    "CacheResultToSystem(result)",
    "UpdateCurrentLidarCloud(",
    "delta_pose_ = delta_pose",
    "last_pose_ = curr_pose",
    "last_nav_state_ = curr_nav_state",
    "last_odom_state_ = predict_odom_state",
  };
  ASSERT_NE(tracking_complete, std::string::npos);
  ASSERT_NE(tracking_lidar_cloud, std::string::npos);
  ASSERT_NE(tracking_ground_estimate, std::string::npos);
  EXPECT_LT(tracking_ground_estimate, tracking_complete);
  EXPECT_LT(tracking_lidar_cloud, tracking_complete);
  for (const auto & token : tracking_atomic_commit_tokens) {
    const std::size_t commit = localization_run.find(token);
    ASSERT_NE(commit, std::string::npos) << token;
    EXPECT_LT(tracking_complete, commit);
  }
  const std::string initialization_commit_and_after = initialization_body.substr(
    initialization_complete +
    std::string("Complete(initialization_commit_now_us, false)").size());
  EXPECT_EQ(initialization_commit_and_after.find(
    "DiscardTimedOutFrameIfNeeded"), std::string::npos);
  EXPECT_EQ(initialization_commit_and_after.find(
    "frame_processing_watchdog_.Complete("), std::string::npos);

  const std::string tracking_commit_and_after = localization_run.substr(
    tracking_complete +
    std::string("Complete(tracking_commit_now_us, false)").size());
  EXPECT_EQ(tracking_commit_and_after.find(
    "DiscardTimedOutFrameIfNeeded"), std::string::npos);
  EXPECT_EQ(tracking_commit_and_after.find(
    "frame_processing_watchdog_.Complete("), std::string::npos);

  const std::array<std::string, 8> diagnostic_keys = {
    "preprocessing_last_frame_duration_ms",
    "preprocessing_last_frame_timed_out",
    "preprocessing_timeout_count",
    "preprocessing_dropped_count",
    "localization_last_frame_duration_ms",
    "localization_last_frame_timed_out",
    "localization_timeout_count",
    "localization_dropped_count",
  };
  const std::string runtime_diagnostics_body = ReadFunctionBody(
    "src/slam/system.cpp", "void System::AppendRuntimeDiagnostics(");
  for (const auto & key : diagnostic_keys) {
    EXPECT_NE(runtime_diagnostics_body.find(key), std::string::npos);
  }
}

TEST(AutomaticRecoverySurface, UsesOneShotWorkerOwnedReinitialization) {
  const std::array<std::string, 2> header_tokens = {
    "void RequestAutomaticRelocalization()",
    "reinitialization_control_",
  };
  for (const auto& token : header_tokens) {
    ExpectToken("include/slam/localization.h", token);
  }

  const std::array<std::string, 5> source_tokens = {
    "InitPreintegration",
    "RequestAutomaticRelocalization",
    "BeginManualInitialization",
    "BeginWorkerAttempt",
    "RecordInitializationResult",
  };
  for (const auto& token : source_tokens) {
    ExpectToken("src/slam/localization.cpp", token);
  }

  ExpectToken(
    "include/localization_health/reinitialization_control.h",
    "class ReinitializationControl");
  ExpectToken(
    "include/localization_health/reinitialization_control.h",
    "automatic_relocalization_request_pending_");
  ExpectToken(
    "include/localization_health/reinitialization_control.h",
    "waiting_for_automatic_relocalization_request_");
  ExpectToken(
    "src/localization_health/reinitialization_control.cpp", ".exchange(false)");
  ExpectToken(
    "src/localization_health/reinitialization_control.cpp", ".store(true)");
  ExpectToken(
    "src/localization_health/reinitialization_control.cpp", ".store(false)");
}

TEST(LocalizationMapReadinessSurface, PreparesMapsBeforeRun) {
  ExpectToken("include/slam/localization.h", "bool PrepareMap()");
  ExpectToken(
    "include/slam/localization.h", "MapPreparationDurationMs() const");

  const std::string run_body = ReadFunctionBody(
    "src/slam/localization.cpp", "void Localization::Run()");
  EXPECT_EQ(run_body.find("LoadGlobalMap("), std::string::npos)
    << "Run() must only consume a prepared global map";
  EXPECT_EQ(run_body.find("LoadScanContextDb("), std::string::npos)
    << "Run() must only consume a prepared Scan Context database";
}

TEST(ThreadRuntimeControllerSurface, ExposesIndependentNativeThreadControls) {
  const std::array<std::string, 7> header_tokens = {
    "namespace runtime",
    "struct ThreadRuntimeConfig",
    "struct ThreadRuntimeStatus",
    "struct ThreadRuntimeOperations",
    "class ThreadRuntimeController",
    "static void Validate",
    "ApplyToCurrentThread",
  };
  for (const auto & token : header_tokens) {
    ExpectToken("include/runtime/thread_runtime_controller.h", token);
  }
  const std::array<std::string, 3> source_tokens = {
    "NativeOperations",
    "pthread_setschedparam",
    "pthread_setaffinity_np",
  };
  for (const auto & token : source_tokens) {
    ExpectToken("src/runtime/thread_runtime_controller.cpp", token);
  }
  ExpectToken("CMakeLists.txt", "src/runtime/thread_runtime_controller.cpp");
  ExpectToken("CMakeLists.txt", "test_thread_runtime_controller");
  ExpectToken("CMakeLists.txt", "Threads::Threads");

  const std::array<std::string, 5> warning_tokens = {
    "LOG(WARNING)",
    "FIFO scheduling",
    "priority",
    "CPU affinity",
    "pthread error code",
  };
  for (const auto & token : warning_tokens) {
    ExpectToken("src/runtime/thread_runtime_controller.cpp", token);
  }
}

TEST(FrameProcessingWatchdogSurface, ExposesThreadSafeFrameDeadlines) {
  const std::array<std::string, 7> header_tokens = {
    "namespace runtime",
    "struct FrameProcessingSnapshot",
    "class FrameProcessingWatchdog",
    "SteadyNowMicroseconds",
    "Begin",
    "Exceeded",
    "Complete",
  };
  for (const auto & token : header_tokens) {
    ExpectToken("include/runtime/frame_processing_watchdog.h", token);
  }

  const std::array<std::string, 3> source_tokens = {
    "std::chrono::steady_clock",
    "std::lock_guard<std::mutex>",
    "timeout_count",
  };
  for (const auto & token : source_tokens) {
    ExpectToken("src/runtime/frame_processing_watchdog.cpp", token);
  }
  ExpectToken("CMakeLists.txt", "src/runtime/frame_processing_watchdog.cpp");
  ExpectToken("CMakeLists.txt", "test_frame_processing_watchdog");
}

TEST(SystemStartupReadinessSurface, PreparesMapBeforeAcceptingInput) {
  const std::string constructor_body = ReadFunctionBody(
    "src/slam/system.cpp",
    "System::System(const rclcpp::NodeOptions& options)");
  const std::size_t localization_construction = constructor_body.find(
    "localization_ptr_ = std::make_shared<Localization>(this);");
  const std::size_t map_preparation = constructor_body.find(
    "localization_ptr_->PrepareMap()");
  const std::size_t publisher_initialization = constructor_body.find(
    "InitPublisher();");
  const std::size_t subscriber_initialization = constructor_body.find(
    "InitSubscriber();");
  const std::size_t localization_worker = constructor_body.find(
    "localization_thread_ptr_ =");
  const std::size_t preprocessing_worker = constructor_body.find(
    "pre_processing_thread_ptr_ =");

  ASSERT_NE(localization_construction, std::string::npos);
  ASSERT_NE(map_preparation, std::string::npos);
  ASSERT_NE(publisher_initialization, std::string::npos);
  ASSERT_NE(subscriber_initialization, std::string::npos);
  ASSERT_NE(localization_worker, std::string::npos);
  ASSERT_NE(preprocessing_worker, std::string::npos);
  EXPECT_LT(localization_construction, map_preparation);
  EXPECT_LT(map_preparation, publisher_initialization);
  EXPECT_LT(map_preparation, subscriber_initialization);
  EXPECT_LT(map_preparation, localization_worker);
  EXPECT_LT(map_preparation, preprocessing_worker);
  ExpectToken("src/slam/system.cpp", "if (!localization_ptr_->PrepareMap())");
  ExpectToken("src/slam/system.cpp", "throw std::runtime_error");

  const std::string diagnostics_body = ReadFunctionBody(
    "src/slam/system.cpp", "void System::PublishLocalizationHealth()");
  const std::size_t map_ready = diagnostics_body.find(
    "add_value(\"map_ready\"");
  const std::size_t preparation_duration = diagnostics_body.find(
    "add_value(\"map_preparation_duration_ms\"");
  ASSERT_NE(map_ready, std::string::npos);
  ASSERT_NE(preparation_duration, std::string::npos);
  EXPECT_LT(map_ready, preparation_duration);
  ExpectToken("src/slam/system.cpp", "MapPreparationDurationMs()");
}

TEST(SystemIntegration, WiresLocalizationHealthObservability) {
  const std::array<std::string, 8> header_tokens = {
    "diagnostic_msgs/msg/diagnostic_array.hpp",
    "localization_health/data_stream_watchdog.h",
    "localization_health/localization_recovery_coordinator.h",
    "diagnostics_pub_",
    "data_stream_watchdog_",
    "localization_recovery_coordinator_",
    "has_new_health_result_",
    "last_health_result_arrival_steady_us_",
  };
  for (const auto & token : header_tokens) {
    ExpectToken("include/slam/system.h", token);
  }

  const std::array<std::string, 35> source_tokens = {
    "RecordLidar",
    "RecordImu",
    "has_new_result",
    "has_new_stream_observation",
    "inputs.stream.has_new_observation",
    "std::chrono::steady_clock",
    "data_stream_watchdog_->Evaluate(steady_now_us)",
    "localization_results.swap(localization_results_deque_)",
    "for (const auto& health_result : localization_results)",
    "\"/diagnostics\"",
    "\"health_score\"",
    "\"s_fitness\"",
    "\"s_stream\"",
    "\"s_consistency\"",
    "\"lidar_rate_hz\"",
    "\"imu_rate_hz\"",
    "\"covariance_scale\"",
    "status.name = \"lio_sam_hesai: localization\"",
    "status.hardware_id = \"lio_sam_hesai_relocalization\"",
    "mapping_odom.pose.covariance[0]",
    "mapping_odom.pose.covariance[7]",
    "mapping_odom.pose.covariance[14]",
    "mapping_odom.pose.covariance[21]",
    "mapping_odom.pose.covariance[28]",
    "mapping_odom.pose.covariance[35]",
    "current_health_covariance_scale_",
    "PublishLocalizationHealth",
    "state_config.min_stream_rate_ratio = config.health_min_rate_ratio_",
    "LocalizationRecoveryCoordinator",
    "localization_recovery_coordinator_->Update",
    "request_automatic_relocalization",
    "recovery_update.request_sent",
    "RequestAutomaticRelocalization",
    "auto_relocalization_trigger",
    "ToString(recovery_update.reason)",
  };
  for (const auto & token : source_tokens) {
    ExpectToken("src/slam/system.cpp", token);
  }
  ExpectToken(
    "include/common/sensor_data_type.h", "health_arrival_steady_us");
  ExpectToken(
    "src/slam/localization.cpp",
    "queued_result.health_arrival_steady_us = SteadyNowMicroseconds()");
  ExpectToken(
    "src/slam/system.cpp", "health_result.health_arrival_steady_us");
  ExpectNoToken(
    "include/slam/system.h",
    "std::atomic_bool has_new_stream_observation_");
  ExpectNoToken(
    "src/slam/system.cpp", "has_new_stream_observation_.");

  const std::array<std::string, 6> cmake_tokens = {
    "find_package(diagnostic_msgs REQUIRED)",
    "src/localization_health/data_stream_watchdog.cpp",
    "src/localization_health/localization_state_machine.cpp",
    "src/localization_health/recovery_attempt_gate.cpp",
    "src/localization_health/localization_recovery_coordinator.cpp",
    "diagnostic_msgs",
  };
  for (const auto & token : cmake_tokens) {
    ExpectToken("CMakeLists.txt", token);
  }

  ExpectToken("package.xml", "<build_depend>diagnostic_msgs</build_depend>");
  ExpectToken("package.xml", "<exec_depend>diagnostic_msgs</exec_depend>");
}
