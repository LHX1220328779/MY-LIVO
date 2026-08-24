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

Keyframe::Ptr MakeObservedPointKeyframe(std::uint64_t id, double pose_x,
                                        double point_x)
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  KeyframePoint point;
  point.x = static_cast<float>(point_x);
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = static_cast<float>(id + 1);
  cloud->push_back(point);
  return std::make_shared<Keyframe>(
      id, static_cast<double>(id),
      Pose3d(Eigen::Quaterniond::Identity(),
             Eigen::Vector3d(pose_x, 0, 0)),
      cloud, Matrix6d::Identity(), 0U);
}

double MaximumX(const KeyframeCloud::ConstPtr &cloud)
{
  double maximum_x = -1.0e9;
  for (const auto &point : cloud->points)
    maximum_x = std::max(maximum_x, static_cast<double>(point.x));
  return maximum_x;
}

double XForIntensity(const KeyframeCloud::ConstPtr &cloud, float intensity)
{
  for (const auto &point : cloud->points)
    if (std::abs(point.intensity - intensity) < 1.0e-5F)
      return point.x;
  throw std::runtime_error("point intensity was not found");
}

void TestOptimizedGlobalMapUsesGraphSnapshot()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.periodic_keyframe_interval = 2;
  options.preserve_rigid_local_submaps = false;
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
  options.preserve_rigid_local_submaps = false;
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

void TestQuarantinedKeyframesAreRemoved()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.periodic_keyframe_interval = 1;
  options.preserve_rigid_local_submaps = false;
  OptimizedGlobalMap map(options);
  const std::vector<Keyframe::Ptr> keyframes{
      MakeKeyframe(0, 0.0), MakeKeyframe(1, 11.0),
      MakeKeyframe(2, 21.0)};
  const std::vector<Pose3d> poses{
      Pose3d(),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(11, 0, 0)),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(21, 0, 0))};
  const auto complete = map.Build(keyframes, poses, "periodic");
  Require(complete.output_points == 3,
          "complete map did not contain every keyframe");

  const std::vector<std::uint8_t> eligible{1U, 1U, 0U};
  const auto quarantined =
      map.Build(keyframes, poses, eligible, "graph_update");
  Require(quarantined.build_mode == "full",
          "eligibility change did not rebuild the map");
  Require(quarantined.included_keyframes == 2 &&
              quarantined.quarantined_keyframes == 1 &&
              quarantined.output_points == 2,
          "quarantined cloud remained in the trusted map");
  Require(std::abs(MaximumX(quarantined.cloud) - 12.0) < 1.0e-5,
          "trusted map retained a point from the quarantined tail");
}

void TestRigidLocalSubmapsSuppressPerKeyframeWarp()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.rigid_submap_length_m = 10.0;
  options.rigid_submap_boundary_position_change_m = 0.50;
  options.rigid_submap_boundary_angle_change_deg = 0.50;
  OptimizedGlobalMap map(options);
  std::vector<Keyframe::Ptr> keyframes{
      MakeKeyframe(0, 0.0), MakeKeyframe(1, 4.0),
      MakeKeyframe(2, 8.0), MakeKeyframe(3, 12.0)};
  keyframes[1]->set_T_slam_body(Pose3d(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(4.1, 0, 0)));
  keyframes[2]->set_T_slam_body(Pose3d(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(8.2, 0, 0)));
  keyframes[3]->set_T_slam_body(Pose3d(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(12.3, 0, 0)));
  const std::vector<Pose3d> global_poses{
      Pose3d(),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(4.3, 0, 0)),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(8.6, 0, 0)),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(12.9, 0, 0))};
  const auto result = map.Build(keyframes, global_poses, "periodic");
  Require(result.rigid_submaps == 2,
          "ten-metre local trajectory was partitioned incorrectly");
  Require(std::abs(XForIntensity(result.cloud, 2.0F) - 5.1) < 1.0e-5 &&
              std::abs(XForIntensity(result.cloud, 3.0F) - 9.2) < 1.0e-5,
          "map did not preserve the local-SLAM relative structure");
  Require(std::abs(XForIntensity(result.cloud, 4.0F) - 13.9) < 1.0e-5,
          "new rigid submap did not use its global elastic anchor");
  Require(std::abs(result.maximum_suppressed_position_warp_m - 0.4) <
              1.0e-10,
          "suppressed per-keyframe warp diagnostic differs");
}

void TestSpatialDeformationDeduplicatesBeforeGlobalWarp()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.preserve_rigid_local_submaps = false;
  options.spatial_deformation_enabled = true;
  options.spatial_deformation_neighbors = 2;
  options.spatial_deformation_sigma_m = 5.0;
  OptimizedGlobalMap map(options);

  // Both scans observe exactly the same local-map point at x=5 m. Their
  // global corrections disagree by 0.2 m; scan-wise warping would produce a
  // double surface, while local-first voxelization must retain one point.
  const std::vector<Keyframe::Ptr> keyframes{
      MakeObservedPointKeyframe(0, 0.0, 5.0),
      MakeObservedPointKeyframe(1, 1.0, 4.0)};
  const std::vector<Pose3d> global_poses{
      Pose3d(),
      Pose3d(Eigen::Quaterniond::Identity(),
             Eigen::Vector3d(1.2, 0, 0))};
  const auto result = map.Build(keyframes, global_poses, "periodic");
  Require(result.build_mode == "full",
          "spatial deformation was not rebuilt atomically");
  Require(result.local_voxel_points == 1 && result.output_points == 1,
          "spatial deformation duplicated one local surface");
  Require(result.spatial_deformation_nodes == 2,
          "spatial deformation did not use all eligible nodes");
  const double x = result.cloud->front().x;
  Require(x >= 5.0 && x <= 5.2,
          "spatial deformation produced an invalid blended correction");
}

void TestSpatialDeformationDoesNotBlendAcrossQuarantine()
{
  OptimizedGlobalMap::Options options;
  options.voxel_leaf_size_m = 0.1;
  options.tile_size_m = 10.0;
  options.preserve_rigid_local_submaps = false;
  options.spatial_deformation_enabled = true;
  options.spatial_deformation_neighbors = 2;
  OptimizedGlobalMap map(options);
  const std::vector<Keyframe::Ptr> keyframes{
      MakeObservedPointKeyframe(0, 0.0, 5.0),
      MakeObservedPointKeyframe(1, 1.0, 4.0),
      MakeObservedPointKeyframe(2, 2.0, 3.0)};
  const std::vector<Pose3d> global_poses{
      Pose3d(),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 0, 0)),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(4, 0, 0))};
  const auto result = map.Build(
      keyframes, global_poses, std::vector<std::uint8_t>{1U, 0U, 1U},
      "graph_update");
  Require(result.local_voxel_points == 2 && result.output_points == 2,
          "spatial deformation blended across a quarantined topology gap");
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
    TestQuarantinedKeyframesAreRemoved();
    TestRigidLocalSubmapsSuppressPerKeyframeWarp();
    TestSpatialDeformationDeduplicatesBeforeGlobalWarp();
    TestSpatialDeformationDoesNotBlendAcrossQuarantine();
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
