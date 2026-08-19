#include "backend/pose_graph_optimizer.h"

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
using my_livo::backend::Keyframe;
using my_livo::backend::KeyframeCloud;
using my_livo::backend::KeyframePoint;
using my_livo::backend::KeyframeTrigger;
using my_livo::backend::Matrix6d;
using my_livo::backend::Pose3d;
using my_livo::backend::PoseGraphOptimizer;

constexpr double kPi = 3.14159265358979323846;

void Require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

Pose3d MakePose(double x, double y, double z, double yaw_degrees)
{
  return Pose3d(
      Eigen::Quaterniond(Eigen::AngleAxisd(
          yaw_degrees * kPi / 180.0, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(x, y, z));
}

Keyframe::Ptr MakeKeyframe(std::uint64_t id, double timestamp,
                           const Pose3d &raw_pose)
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  KeyframePoint point;
  point.x = 1.0F;
  point.intensity = 1.0F;
  cloud->push_back(point);
  return std::make_shared<Keyframe>(
      id, timestamp, raw_pose, cloud, Matrix6d::Identity(),
      static_cast<std::uint8_t>(
          id == 0U ? KeyframeTrigger::kFirst
                   : KeyframeTrigger::kTranslation));
}

void TestChainOptimizationAndStatistics()
{
  const std::filesystem::path csv_path =
      std::filesystem::temp_directory_path() /
      "my_livo_pose_graph_optimizer_test.csv";

  PoseGraphOptimizer::Options options;
  options.additional_update_steps = 2;
  options.csv_path = csv_path.string();
  PoseGraphOptimizer optimizer(options);

  auto keyframe0 = MakeKeyframe(0, 100.0, MakePose(0.0, 0.0, 0.0, 0.0));
  auto keyframe1 = MakeKeyframe(1, 101.0, MakePose(1.0, 0.2, 0.1, 5.0));
  auto keyframe2 = MakeKeyframe(2, 102.0, MakePose(2.0, 0.6, 0.2, 12.0));

  // Deliberately perturb the first node's initial T_map_body while keeping its
  // immutable raw pose as the prior measurement. This proves iSAM2 performs
  // an update instead of merely returning identical initial values.
  keyframe0->set_T_map_body(MakePose(0.4, -0.2, 0.3, 8.0));

  const auto result0 = optimizer.AddKeyframe(keyframe0);
  Require(!result0.optimization_ran,
          "prior initialization was miscounted as an odometry optimization");
  Require(result0.variables_reeliminated > 0,
          "iSAM2 did not eliminate the initialized prior node");
  Require(result0.variables_reeliminated <= 3,
          "iSAM2 prior update reported impossible elimination work");
  const auto result1 = optimizer.AddKeyframe(keyframe1);
  Require(result1.optimization_ran && result1.solution_usable,
          "two-node pose graph did not produce a usable solution");
  const auto result2 = optimizer.AddKeyframe(keyframe2);
  Require(result2.optimization_ran && result2.solution_usable,
          "three-node pose graph did not produce a usable solution");
  Require(result2.final_cost <= result2.initial_cost + 1.0e-12,
          "iSAM2 update increased the nonlinear graph error");
  Require(result1.variables_relinearized <= 6 &&
              result1.variables_reeliminated <= 6,
          "two-node update reported impossible iSAM2 work");
  Require(result2.variables_relinearized <= 9 &&
              result2.variables_reeliminated <= 9,
          "three-node update reported impossible iSAM2 work");

  for (const auto &keyframe : {keyframe0, keyframe1, keyframe2})
  {
    const Pose3d raw = keyframe->T_odom_body();
    const Pose3d optimized = keyframe->T_map_body();
    Require((raw.translation - optimized.translation).norm() < 1.0e-8,
            "optimized chain translation differs from raw odometry");
    Require(raw.rotation.angularDistance(optimized.rotation) < 1.0e-9,
            "optimized chain rotation differs from raw odometry");
  }

  const auto statistics = optimizer.statistics();
  Require(statistics.nodes == 3, "pose-graph node count is incorrect");
  Require(statistics.odometry_factors == 2,
          "pose-graph odometry-factor count is incorrect");
  Require(statistics.optimization_runs == 2,
          "pose-graph optimization-run count is incorrect");
  Require(statistics.failed_optimizations == 0,
          "pose-graph reported a failed optimization");

  const auto poses = optimizer.optimized_poses();
  Require(poses.size() == 3,
          "optimized pose snapshot has the wrong size");

  std::ifstream csv(csv_path);
  Require(csv.good(), "pose-graph CSV was not created");
  std::string line;
  int lines = 0;
  bool gtsam_solver_logged = false;
  while (std::getline(csv, line))
  {
    ++lines;
    if (line.rfind("gtsam_isam2,", 0) == 0) gtsam_solver_logged = true;
  }
  Require(lines == 4, "pose-graph CSV does not contain one row per node");
  Require(gtsam_solver_logged, "pose-graph CSV did not identify GTSAM iSAM2");
  std::filesystem::remove(csv_path);
}

void TestIdInvariant()
{
  PoseGraphOptimizer optimizer{PoseGraphOptimizer::Options()};
  bool rejected = false;
  try
  {
    (void)optimizer.AddKeyframe(
        MakeKeyframe(1, 1.0, MakePose(0.0, 0.0, 0.0, 0.0)));
  }
  catch (const std::logic_error &)
  {
    rejected = true;
  }
  Require(rejected, "non-contiguous pose-graph ID was not rejected");
}

void TestRobustLoopFactorUpdate()
{
  const auto temporary = std::filesystem::temp_directory_path();
  const auto loop_path = temporary / "my_livo_loop_factor_test.csv";
  const auto trajectory_path =
      temporary / "my_livo_optimized_trajectory_test.csv";
  {
    PoseGraphOptimizer::Options options;
    options.loop_translation_sigma_m = 0.05;
    options.loop_rotation_sigma_deg = 1.0;
    options.loop_robust_kernel = "cauchy";
    options.loop_robust_delta = 10.0;
    options.loop_additional_update_steps = 2;
    options.loop_csv_path = loop_path.string();
    options.optimized_trajectory_csv_path = trajectory_path.string();
    PoseGraphOptimizer optimizer(options);
    std::vector<Keyframe::Ptr> keyframes;
    for (std::uint64_t id = 0; id < 4; ++id)
    {
      keyframes.push_back(MakeKeyframe(
          id, static_cast<double>(id),
          MakePose(1.1 * static_cast<double>(id), 0.0, 0.0, 0.0)));
      (void)optimizer.AddKeyframe(keyframes.back());
    }
    const auto loop = optimizer.AddLoopFactor(
        0, 3, MakePose(3.0, 0.0, 0.0, 0.0));
    Require(loop.added && loop.solution_usable,
            "valid robust loop factor was not added");
    Require(loop.residual_translation_before_m > 0.29 &&
                loop.residual_translation_after_m <
                    loop.residual_translation_before_m,
            "loop optimization did not reduce its translation residual");
    Require(loop.maximum_pose_correction_m > 1.0e-3,
            "loop factor did not change any optimized pose");
    Require(std::abs(keyframes.back()->T_odom_body().translation.x() - 3.3) <
                1.0e-12,
            "loop optimization modified immutable raw odometry");
    const auto statistics = optimizer.statistics();
    Require(statistics.loop_factors == 1 &&
                statistics.loop_optimization_runs == 1,
            "loop-factor statistics are incorrect");

    bool duplicate_rejected = false;
    try
    {
      (void)optimizer.AddLoopFactor(
          0, 3, MakePose(3.0, 0.0, 0.0, 0.0));
    }
    catch (const std::logic_error &)
    {
      duplicate_rejected = true;
    }
    Require(duplicate_rejected, "duplicate loop factor was not rejected");
  }
  std::ifstream loop_csv(loop_path);
  std::ifstream trajectory_csv(trajectory_path);
  Require(loop_csv.good() && trajectory_csv.good(),
          "loop/final trajectory CSV was not created");
  std::string line;
  int loop_lines = 0;
  while (std::getline(loop_csv, line)) ++loop_lines;
  int trajectory_lines = 0;
  while (std::getline(trajectory_csv, line)) ++trajectory_lines;
  Require(loop_lines == 2, "loop-factor CSV row count is incorrect");
  Require(trajectory_lines == 5,
          "optimized-trajectory CSV row count is incorrect");
  std::filesystem::remove(loop_path);
  std::filesystem::remove(trajectory_path);
}

void TestRobustRtkPositionFactorUpdate()
{
  const auto path = std::filesystem::temp_directory_path() /
                    "my_livo_rtk_factor_test.csv";
  {
    PoseGraphOptimizer::Options options;
    options.rtk_robust_kernel = "huber";
    options.rtk_robust_delta = 10.0;
    options.rtk_additional_update_steps = 2;
    options.rtk_csv_path = path.string();
    PoseGraphOptimizer optimizer(options);
    std::vector<Keyframe::Ptr> keyframes;
    for (std::uint64_t id = 0; id < 4; ++id)
    {
      keyframes.push_back(MakeKeyframe(
          id, static_cast<double>(id),
          MakePose(1.1 * static_cast<double>(id), 0, 0, 0)));
      (void)optimizer.AddKeyframe(keyframes.back());
    }
    const Eigen::Matrix3d covariance =
        0.01 * Eigen::Matrix3d::Identity();
    const auto update = optimizer.AddRtkPositionFactor(
        3, Eigen::Vector3d(3.0, 0, 0), covariance);
    Require(update.added && update.solution_usable,
            "valid RTK position factor was not added");
    Require(update.innovation_before_m.norm() > 0.29 &&
                update.innovation_after_m.norm() <
                    update.innovation_before_m.norm(),
            "RTK position factor did not reduce its residual");
    Require(std::abs(keyframes.back()->T_odom_body().translation.x() - 3.3) <
                1.0e-12,
            "RTK optimization modified immutable raw odometry");
    Require(optimizer.statistics().rtk_factors == 1,
            "RTK factor statistics are incorrect");
    bool duplicate_rejected = false;
    try
    {
      (void)optimizer.AddRtkPositionFactor(
          3, Eigen::Vector3d(3.0, 0, 0), covariance);
    }
    catch (const std::logic_error &)
    {
      duplicate_rejected = true;
    }
    Require(duplicate_rejected, "duplicate RTK factor was not rejected");
  }
  std::ifstream csv(path);
  std::string line;
  int lines = 0;
  while (std::getline(csv, line)) ++lines;
  Require(lines == 2, "RTK factor CSV row count is incorrect");
  std::filesystem::remove(path);
}

void TestRobustRtkOutlierDoesNotCollapseGraph()
{
  PoseGraphOptimizer::Options options;
  options.rtk_robust_kernel = "cauchy";
  options.rtk_robust_delta = 1.0;
  options.rtk_additional_update_steps = 2;
  PoseGraphOptimizer optimizer(options);
  std::vector<Keyframe::Ptr> keyframes;
  for (std::uint64_t id = 0; id < 10; ++id)
  {
    keyframes.push_back(MakeKeyframe(
        id, static_cast<double>(id), MakePose(id, 0, 0, 0)));
    (void)optimizer.AddKeyframe(keyframes.back());
  }
  const Eigen::Matrix3d covariance =
      0.09 * Eigen::Matrix3d::Identity();
  const auto update = optimizer.AddRtkPositionFactor(
      9, Eigen::Vector3d(100, 100, 100), covariance);
  Require(update.added, "robust RTK outlier factor was not processed");
  Require(update.maximum_pose_correction_m < 0.1,
          "Cauchy RTK outlier collapsed the odometry graph");
}
}  // namespace

int main()
{
  try
  {
    TestChainOptimizationAndStatistics();
    TestIdInvariant();
    TestRobustLoopFactorUpdate();
    TestRobustRtkPositionFactorUpdate();
    TestRobustRtkOutlierDoesNotCollapseGraph();
  }
  catch (const std::exception &error)
  {
    std::cerr << "pose_graph_optimizer_test failed: "
              << error.what() << '\n';
    return 1;
  }
  std::cout << "pose_graph_optimizer_test passed\n";
  return 0;
}
