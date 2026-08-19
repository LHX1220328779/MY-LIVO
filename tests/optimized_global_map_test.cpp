#include "backend/frame_transform.h"
#include "backend/optimized_global_map.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using my_livo::backend::Keyframe;
using my_livo::backend::KeyframeCloud;
using my_livo::backend::KeyframePoint;
using my_livo::backend::Matrix6d;
using my_livo::backend::OptimizedGlobalMap;
using my_livo::backend::Pose3d;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

Keyframe::Ptr MakeKeyframe(std::uint64_t id, double x)
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  KeyframePoint point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = static_cast<float>(id + 1);
  cloud->push_back(point);
  return std::make_shared<Keyframe>(
      id, static_cast<double>(id),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, 0, 0)),
      cloud, Matrix6d::Identity(), 0U);
}

double MaximumX(const KeyframeCloud::ConstPtr &cloud)
{
  double maximum_x = -1.0e9;
  for (const auto &point : cloud->points)
    maximum_x = std::max(maximum_x, static_cast<double>(point.x));
  return maximum_x;
}

void TestOptimizedGlobalMapUsesGraphSnapshot()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.periodic_keyframe_interval = 2;
  OptimizedGlobalMap map(options);
  const std::vector<Keyframe::Ptr> keyframes{
      MakeKeyframe(0, 0.0), MakeKeyframe(1, 10.0)};
  std::vector<Pose3d> poses{
      Pose3d(), Pose3d(Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d(10, 0, 0))};
  const auto first = map.Build(keyframes, poses, "periodic");
  Require(first.output_points == 2, "separated points were lost");
  Require(first.build_mode == "full", "initial build was not full");
  Require(!map.NeedsPeriodicBuild(3), "map rebuilt too early");
  Require(map.NeedsPeriodicBuild(4), "periodic rebuild was not requested");

  poses[1].translation.x() = 20.0;
  const auto corrected = map.Build(keyframes, poses, "loop");
  Require(corrected.revision == 2, "map revision did not increase");
  Require(corrected.build_mode == "full", "loop build was not full");
  Require(std::abs(MaximumX(corrected.cloud) - 21.0) < 1.0e-5,
          "map did not use the corrected graph pose");
}

void TestTiledIncrementalUpdateAndCorrectionThreshold()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.periodic_keyframe_interval = 1;
  options.incremental_max_pose_change_m = 0.10;
  options.incremental_max_pose_change_deg = 0.25;
  OptimizedGlobalMap map(options);

  std::vector<Keyframe::Ptr> keyframes{
      MakeKeyframe(0, 0.0), MakeKeyframe(1, 11.0)};
  std::vector<Pose3d> poses{
      Pose3d(), Pose3d(Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d(11, 0, 0))};
  const auto initial = map.Build(keyframes, poses, "periodic");
  Require(initial.build_mode == "full", "initial tiled build was not full");
  Require(initial.total_tiles == 2, "initial build did not create two tiles");

  keyframes.push_back(MakeKeyframe(2, 21.0));
  poses.emplace_back(Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d(21, 0, 0));
  const auto appended = map.Build(keyframes, poses, "periodic");
  Require(appended.build_mode == "incremental",
          "append-only map update was not incremental");
  Require(appended.processed_keyframes == 1,
          "incremental update reprocessed old keyframes");
  Require(appended.updated_tiles == 1 && appended.total_tiles == 3,
          "incremental update touched the wrong tiles");

  poses[0].translation.x() = 0.25;
  keyframes.push_back(MakeKeyframe(3, 31.0));
  poses.emplace_back(Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d(31, 0, 0));
  const auto corrected = map.Build(keyframes, poses, "periodic");
  Require(corrected.build_mode == "full",
          "large historical correction did not force a full rebuild");
  Require(corrected.processed_keyframes == keyframes.size(),
          "full correction rebuild did not process every keyframe");
  Require(map.CorrectionRequiresFullRebuild(0.11, 0.0),
          "position threshold did not request a rebuild");
  Require(map.CorrectionRequiresFullRebuild(0.0, 0.26),
          "angle threshold did not request a rebuild");
  Require(!map.CorrectionRequiresFullRebuild(0.05, 0.10),
          "small correction incorrectly requested a full rebuild");
}

void TestMapToOdomComposition()
{
  const Pose3d T_odom_body(
      Eigen::Quaterniond(
          Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(10, 3, 1));
  const Pose3d expected_map_odom(
      Eigen::Quaterniond(
          Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(2, -4, 0.5));
  const Pose3d T_map_body = expected_map_odom * T_odom_body;
  const Pose3d actual =
      my_livo::backend::ComputeMapToOdom(T_map_body, T_odom_body);
  Require((actual.translation - expected_map_odom.translation).norm() < 1.0e-10,
          "map->odom translation is wrong");
  Require(actual.rotation.angularDistance(expected_map_odom.rotation) < 1.0e-10,
          "map->odom rotation is wrong");
}
}  // namespace

int main()
{
  try
  {
    TestOptimizedGlobalMapUsesGraphSnapshot();
    TestTiledIncrementalUpdateAndCorrectionThreshold();
    TestMapToOdomComposition();
    std::cout << "optimized_global_map_test passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "optimized_global_map_test failed: " << error.what() << '\n';
    return 1;
  }
}
