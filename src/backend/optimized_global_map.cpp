#include "backend/optimized_global_map.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

namespace my_livo::backend
{

OptimizedGlobalMap::OptimizedGlobalMap(const Options &options)
    : options_(options)
{
  ValidateOptions(options_);
  if (!options_.csv_path.empty())
  {
    const std::filesystem::path path(options_.csv_path);
    if (path.has_parent_path())
      std::filesystem::create_directories(path.parent_path());
    csv_stream_.open(path, std::ios::out | std::ios::trunc);
    if (!csv_stream_.is_open())
      throw std::runtime_error("Cannot open optimized-map CSV: " +
                               path.string());
    csv_stream_ << "revision,reason,build_mode,keyframes,included_keyframes,"
                   "quarantined_keyframes,input_points,output_points,"
                   "processed_keyframes,updated_tiles,"
                   "total_tiles,rigid_submaps,"
                   "local_voxel_points,spatial_deformation_nodes,"
                   "maximum_suppressed_position_warp_m,"
                   "maximum_suppressed_angle_warp_deg,"
                   "build_time_ms,min_x,min_y,min_z,max_x,"
                   "max_y,max_z\n";
  }
}

void OptimizedGlobalMap::ValidateOptions(const Options &options)
{
  if (!std::isfinite(options.voxel_leaf_size_m) ||
      options.voxel_leaf_size_m <= 0.0)
    throw std::invalid_argument(
        "Optimized global-map voxel leaf size must be positive.");
  if (!std::isfinite(options.tile_size_m) || options.tile_size_m <= 0.0)
    throw std::invalid_argument(
        "Optimized global-map tile size must be positive.");
  if (options.tile_size_m < options.voxel_leaf_size_m)
    throw std::invalid_argument(
        "Optimized global-map tile size must not be smaller than its voxel.");
  if (!std::isfinite(options.incremental_max_pose_change_m) ||
      options.incremental_max_pose_change_m < 0.0 ||
      !std::isfinite(options.incremental_max_pose_change_deg) ||
      options.incremental_max_pose_change_deg < 0.0)
    throw std::invalid_argument(
        "Optimized global-map incremental pose thresholds must be finite "
        "and non-negative.");
  if (!std::isfinite(options.rigid_submap_length_m) ||
      options.rigid_submap_length_m <= 0.0 ||
      !std::isfinite(options.rigid_submap_boundary_position_change_m) ||
      options.rigid_submap_boundary_position_change_m <= 0.0 ||
      !std::isfinite(options.rigid_submap_boundary_angle_change_deg) ||
      options.rigid_submap_boundary_angle_change_deg <= 0.0)
    throw std::invalid_argument(
        "Rigid local-submap thresholds must be finite and positive.");
  if (options.spatial_deformation_neighbors == 0 ||
      !std::isfinite(options.spatial_deformation_sigma_m) ||
      options.spatial_deformation_sigma_m <= 0.0)
    throw std::invalid_argument(
        "Spatial map-deformation parameters must be positive.");
  if (options.periodic_keyframe_interval == 0)
    throw std::invalid_argument(
        "Optimized global-map keyframe interval must be positive.");
  if (options.minimum_graph_rebuild_keyframe_interval == 0)
    throw std::invalid_argument(
        "Optimized global-map graph rebuild interval must be positive.");
}

OptimizedGlobalMap::RigidSubmapResult
OptimizedGlobalMap::PreserveRigidLocalSubmaps(
    const std::vector<Keyframe::Ptr> &keyframes,
    const std::vector<Pose3d> &optimized_poses,
    const std::vector<std::uint8_t> &eligible) const
{
  RigidSubmapResult result;
  result.poses = optimized_poses;
  if (!options_.preserve_rigid_local_submaps) return result;

  std::size_t begin = 0;
  while (begin < keyframes.size())
  {
    if (eligible[begin] == 0U)
    {
      ++begin;
      continue;
    }
    const Pose3d local_anchor = keyframes[begin]->T_slam_body();
    const Pose3d T_global_local =
        optimized_poses[begin] * local_anchor.inverse();
    ++result.count;
    double path_length_m = 0.0;
    std::size_t end = begin;
    for (; end < keyframes.size(); ++end)
    {
      if (eligible[end] == 0U) break;
      if (end > begin)
      {
        const Pose3d local_previous = keyframes[end - 1]->T_slam_body();
        const Pose3d local_current = keyframes[end]->T_slam_body();
        const double step_m =
            (local_current.translation - local_previous.translation).norm();
        const Pose3d rigid_candidate = T_global_local * local_current;
        const double position_warp_m =
            (rigid_candidate.translation -
             optimized_poses[end].translation).norm();
        const double angle_warp_deg = 180.0 / std::acos(-1.0) *
            rigid_candidate.rotation.angularDistance(
                optimized_poses[end].rotation);
        const bool segment_boundary =
            position_warp_m >
                options_.rigid_submap_boundary_position_change_m ||
            angle_warp_deg >
                options_.rigid_submap_boundary_angle_change_deg;
        if (path_length_m + step_m > options_.rigid_submap_length_m ||
            segment_boundary)
          break;
        path_length_m += step_m;
      }
      const Pose3d local_pose = keyframes[end]->T_slam_body();
      result.poses[end] = T_global_local * local_pose;
      result.maximum_suppressed_position_warp_m = std::max(
          result.maximum_suppressed_position_warp_m,
          (result.poses[end].translation -
           optimized_poses[end].translation).norm());
      result.maximum_suppressed_angle_warp_deg = std::max(
          result.maximum_suppressed_angle_warp_deg,
          180.0 / std::acos(-1.0) *
              result.poses[end].rotation.angularDistance(
                  optimized_poses[end].rotation));
    }
    begin = end > begin ? end : begin + 1U;
  }
  return result;
}

bool OptimizedGlobalMap::CorrectionRequiresFullRebuild(
    double position_change_m, double angle_change_deg) const
{
  return !std::isfinite(position_change_m) ||
         !std::isfinite(angle_change_deg) ||
         position_change_m > options_.incremental_max_pose_change_m ||
         angle_change_deg > options_.incremental_max_pose_change_deg;
}

bool OptimizedGlobalMap::NeedsPeriodicBuild(
    std::size_t keyframe_count) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframe_count > 0 &&
         (latest_.keyframes == 0 ||
          keyframe_count >= latest_.keyframes +
                                options_.periodic_keyframe_interval);
}

bool OptimizedGlobalMap::CanBuildGraphUpdate(
    std::size_t keyframe_count) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframe_count > 0 &&
         (latest_.keyframes == 0 ||
          keyframe_count >= latest_.keyframes +
                                options_.minimum_graph_rebuild_keyframe_interval);
}

OptimizedGlobalMap::BuildResult OptimizedGlobalMap::Build(
    const std::vector<Keyframe::Ptr> &keyframes,
    const std::vector<Pose3d> &optimized_poses,
    const std::string &reason)
{
  return Build(keyframes, optimized_poses,
               std::vector<std::uint8_t>(keyframes.size(), 1U), reason);
}

OptimizedGlobalMap::BuildResult OptimizedGlobalMap::Build(
    const std::vector<Keyframe::Ptr> &keyframes,
    const std::vector<Pose3d> &optimized_poses,
    const std::vector<std::uint8_t> &eligible,
    const std::string &reason)
{
  // Build() can be called by the asynchronous online worker and by the final
  // shutdown save.  Serializing the complete operation prevents an older
  // snapshot from overwriting a newer revision.
  std::lock_guard<std::mutex> build_lock(build_mutex_);
  if (keyframes.empty() || keyframes.size() != optimized_poses.size() ||
      keyframes.size() != eligible.size())
    throw std::invalid_argument(
        "Optimized global map requires equal, non-empty keyframe and pose "
        "snapshots.");
  const auto start = std::chrono::steady_clock::now();
  std::size_t input_points = 0;
  std::size_t included_keyframes = 0;
  for (std::size_t index = 0; index < keyframes.size(); ++index)
  {
    if (!keyframes[index] || keyframes[index]->id() != index ||
        !keyframes[index]->cloud_body() ||
        !optimized_poses[index].isFinite())
      throw std::invalid_argument(
          "Optimized global-map snapshot contains an invalid keyframe.");
    if (eligible[index] > 1U)
      throw std::invalid_argument(
          "Optimized global-map eligibility must be zero or one.");
    if (eligible[index] != 0U)
    {
      ++included_keyframes;
      input_points += keyframes[index]->cloud_body()->size();
    }
  }
  const RigidSubmapResult rigid_submaps = PreserveRigidLocalSubmaps(
      keyframes, optimized_poses, eligible);
  const std::vector<Pose3d> &map_poses = rigid_submaps.poses;

  std::map<TileKey, KeyframeCloud::ConstPtr> tiles;
  std::vector<Pose3d> previous_poses;
  std::vector<std::uint8_t> previous_eligibility;
  std::size_t previous_keyframes = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tiles = tiles_;
    previous_poses = latest_poses_;
    previous_eligibility = latest_eligibility_;
    previous_keyframes = latest_.keyframes;
  }

  bool incremental = reason == "periodic" && previous_keyframes > 0 &&
                     previous_keyframes < keyframes.size() &&
                     previous_poses.size() == previous_keyframes &&
                     previous_eligibility.size() == previous_keyframes &&
                     !tiles.empty();
  if (incremental)
  {
    for (std::size_t index = 0; index < previous_keyframes; ++index)
    {
      if (previous_eligibility[index] != eligible[index])
      {
        incremental = false;
        break;
      }
      const double position_change =
          (previous_poses[index].translation -
           map_poses[index].translation).norm();
      const double angle_change = 180.0 / std::acos(-1.0) *
          previous_poses[index].rotation.angularDistance(
              map_poses[index].rotation);
      if (CorrectionRequiresFullRebuild(position_change, angle_change))
      {
        incremental = false;
        break;
      }
    }
  }
  if (!incremental)
  {
    tiles.clear();
    previous_keyframes = 0;
  }

  // Spatial deformation depends on the complete local voxelization: adding a
  // scan can change which observations represent an existing local voxel.
  // Consequently this mode intentionally rebuilds atomically instead of
  // appending stale, differently corrected copies of local surfaces.
  if (options_.spatial_deformation_enabled)
  {
    incremental = false;
    tiles.clear();
    previous_keyframes = 0;
  }

  KeyframeCloud::Ptr map_frame_points(new KeyframeCloud());
  std::size_t local_voxel_points = 0;
  std::size_t spatial_deformation_nodes = 0;
  if (options_.spatial_deformation_enabled)
  {
    map_frame_points->reserve(input_points);
    const auto append_continuous_component = [
        &keyframes, &optimized_poses, this, &map_frame_points,
        &local_voxel_points, &spatial_deformation_nodes](
        std::size_t begin, std::size_t end) {
      KeyframeCloud::Ptr local_raw(new KeyframeCloud());
      pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_nodes(
          new pcl::PointCloud<pcl::PointXYZ>());
      trajectory_nodes->reserve(end - begin);
      std::vector<Pose3d> corrections;
      corrections.reserve(end - begin);
      for (std::size_t index = begin; index < end; ++index)
      {
        const Pose3d &T_local_body = keyframes[index]->T_slam_body();
        corrections.push_back(
            optimized_poses[index] * T_local_body.inverse());
        pcl::PointXYZ node;
        node.x = static_cast<float>(T_local_body.translation.x());
        node.y = static_cast<float>(T_local_body.translation.y());
        node.z = static_cast<float>(T_local_body.translation.z());
        trajectory_nodes->push_back(node);
        for (const KeyframePoint &point_body :
             keyframes[index]->cloud_body()->points)
        {
          const Eigen::Vector3d point_local =
              T_local_body * Eigen::Vector3d(
                  point_body.x, point_body.y, point_body.z);
          if (!point_local.allFinite()) continue;
          KeyframePoint point;
          point.x = static_cast<float>(point_local.x());
          point.y = static_cast<float>(point_local.y());
          point.z = static_cast<float>(point_local.z());
          point.intensity = point_body.intensity;
          local_raw->push_back(point);
        }
      }

      KeyframeCloud::Ptr local_filtered(new KeyframeCloud());
      const float local_leaf =
          static_cast<float>(options_.voxel_leaf_size_m);
      pcl::VoxelGrid<KeyframePoint> local_voxel_filter;
      local_voxel_filter.setLeafSize(local_leaf, local_leaf, local_leaf);
      local_voxel_filter.setInputCloud(local_raw);
      local_voxel_filter.filter(*local_filtered);
      local_voxel_points += local_filtered->size();
      spatial_deformation_nodes += trajectory_nodes->size();

      pcl::KdTreeFLANN<pcl::PointXYZ> trajectory_tree;
      trajectory_tree.setInputCloud(trajectory_nodes);
      const int neighbor_count = static_cast<int>(std::min(
          options_.spatial_deformation_neighbors,
          trajectory_nodes->size()));
      const double two_sigma_squared = 2.0 *
          options_.spatial_deformation_sigma_m *
          options_.spatial_deformation_sigma_m;
      std::vector<int> neighbor_indices(
          static_cast<std::size_t>(neighbor_count));
      std::vector<float> squared_distances(
          static_cast<std::size_t>(neighbor_count));
      for (const KeyframePoint &point_local_pcl : local_filtered->points)
      {
        pcl::PointXYZ query;
        query.x = point_local_pcl.x;
        query.y = point_local_pcl.y;
        query.z = point_local_pcl.z;
        const int found = trajectory_tree.nearestKSearch(
            query, neighbor_count, neighbor_indices, squared_distances);
        if (found <= 0) continue;
        const Eigen::Vector3d point_local(
            point_local_pcl.x, point_local_pcl.y, point_local_pcl.z);
        Eigen::Vector3d point_map = Eigen::Vector3d::Zero();
        double weight_sum = 0.0;
        const double nearest_squared_distance = squared_distances.front();
        for (int neighbor = 0; neighbor < found; ++neighbor)
        {
          const double weight = std::exp(-(
              static_cast<double>(squared_distances[neighbor]) -
              nearest_squared_distance) / two_sigma_squared);
          point_map += weight *
              (corrections[neighbor_indices[neighbor]] * point_local);
          weight_sum += weight;
        }
        if (!(weight_sum > 0.0) || !std::isfinite(weight_sum)) continue;
        point_map /= weight_sum;
        if (!point_map.allFinite()) continue;
        KeyframePoint point_map_pcl;
        point_map_pcl.x = static_cast<float>(point_map.x());
        point_map_pcl.y = static_cast<float>(point_map.y());
        point_map_pcl.z = static_cast<float>(point_map.z());
        point_map_pcl.intensity = point_local_pcl.intensity;
        map_frame_points->push_back(point_map_pcl);
      }
    };

    std::size_t begin = 0;
    while (begin < keyframes.size())
    {
      while (begin < keyframes.size() && eligible[begin] == 0U) ++begin;
      if (begin == keyframes.size()) break;
      std::size_t end = begin + 1U;
      while (end < keyframes.size() && eligible[end] != 0U) ++end;
      append_continuous_component(begin, end);
      begin = end;
    }
    if (spatial_deformation_nodes == 0U)
      throw std::runtime_error(
          "Spatial global-map deformation has no eligible trajectory node.");
  }
  else
  {
    map_frame_points->reserve(input_points);
    for (std::size_t index = previous_keyframes;
         index < keyframes.size(); ++index)
    {
      if (eligible[index] == 0U) continue;
      const Pose3d &pose = map_poses[index];
      for (const KeyframePoint &point_body :
           keyframes[index]->cloud_body()->points)
      {
        const Eigen::Vector3d point_map = pose * Eigen::Vector3d(
            point_body.x, point_body.y, point_body.z);
        if (!point_map.allFinite()) continue;
        KeyframePoint point;
        point.x = static_cast<float>(point_map.x());
        point.y = static_cast<float>(point_map.y());
        point.z = static_cast<float>(point_map.z());
        point.intensity = point_body.intensity;
        map_frame_points->push_back(point);
      }
    }
  }

  std::map<TileKey, KeyframeCloud::Ptr> additions;
  for (const KeyframePoint &point_map_pcl : map_frame_points->points)
  {
    const TileKey tile_key{
        static_cast<std::int64_t>(std::floor(
            point_map_pcl.x / options_.tile_size_m)),
        static_cast<std::int64_t>(std::floor(
            point_map_pcl.y / options_.tile_size_m)),
        static_cast<std::int64_t>(std::floor(
            point_map_pcl.z / options_.tile_size_m))};
    auto &addition = additions[tile_key];
    if (!addition) addition.reset(new KeyframeCloud());
    addition->push_back(point_map_pcl);
  }

  const float leaf = static_cast<float>(options_.voxel_leaf_size_m);
  for (auto &[tile_key, addition] : additions)
  {
    KeyframeCloud::Ptr combined(new KeyframeCloud());
    const auto existing = tiles.find(tile_key);
    if (existing != tiles.end() && existing->second)
      *combined = *existing->second;
    combined->insert(combined->end(), addition->begin(), addition->end());
    KeyframeCloud::Ptr filtered_tile(new KeyframeCloud());
    pcl::VoxelGrid<KeyframePoint> voxel_filter;
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    voxel_filter.setInputCloud(combined);
    voxel_filter.filter(*filtered_tile);
    tiles[tile_key] = filtered_tile;
  }

  KeyframeCloud::Ptr filtered(new KeyframeCloud());
  std::size_t output_capacity = 0;
  for (const auto &[tile_key, cloud] : tiles)
  {
    (void)tile_key;
    if (cloud) output_capacity += cloud->size();
  }
  filtered->reserve(output_capacity);
  for (const auto &[tile_key, cloud] : tiles)
  {
    (void)tile_key;
    if (cloud)
      filtered->insert(filtered->end(), cloud->begin(), cloud->end());
  }

  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  if (!filtered->empty())
  {
    minimum.setConstant(std::numeric_limits<double>::infinity());
    maximum.setConstant(-std::numeric_limits<double>::infinity());
    for (const KeyframePoint &point : filtered->points)
    {
      const Eigen::Vector3d value(point.x, point.y, point.z);
      minimum = minimum.cwiseMin(value);
      maximum = maximum.cwiseMax(value);
    }
  }
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();

  std::lock_guard<std::mutex> lock(mutex_);
  tiles_ = std::move(tiles);
  latest_poses_ = options_.spatial_deformation_enabled ?
      optimized_poses : map_poses;
  latest_eligibility_ = eligible;
  latest_.cloud = filtered;
  ++latest_.revision;
  latest_.keyframes = keyframes.size();
  latest_.included_keyframes = included_keyframes;
  latest_.quarantined_keyframes = keyframes.size() - included_keyframes;
  latest_.input_points = input_points;
  latest_.output_points = filtered->size();
  latest_.processed_keyframes = keyframes.size() - previous_keyframes;
  latest_.updated_tiles = additions.size();
  latest_.total_tiles = tiles_.size();
  latest_.rigid_submaps = rigid_submaps.count;
  latest_.local_voxel_points = local_voxel_points;
  latest_.spatial_deformation_nodes = spatial_deformation_nodes;
  latest_.maximum_suppressed_position_warp_m =
      rigid_submaps.maximum_suppressed_position_warp_m;
  latest_.maximum_suppressed_angle_warp_deg =
      rigid_submaps.maximum_suppressed_angle_warp_deg;
  latest_.build_mode = incremental ? "incremental" : "full";
  latest_.build_time_ms = elapsed_ms;
  if (csv_stream_.is_open())
  {
    csv_stream_ << std::setprecision(17) << latest_.revision << ','
                << reason << ',' << latest_.build_mode << ','
                << latest_.keyframes << ',' << latest_.included_keyframes
                << ',' << latest_.quarantined_keyframes << ','
                << latest_.input_points << ','
                << latest_.output_points << ','
                << latest_.processed_keyframes << ','
                << latest_.updated_tiles << ',' << latest_.total_tiles << ','
                << latest_.rigid_submaps << ','
                << latest_.local_voxel_points << ','
                << latest_.spatial_deformation_nodes << ','
                << latest_.maximum_suppressed_position_warp_m << ','
                << latest_.maximum_suppressed_angle_warp_deg << ','
                << elapsed_ms << ',' << minimum.x() << ','
                << minimum.y() << ',' << minimum.z() << ',' << maximum.x()
                << ',' << maximum.y() << ',' << maximum.z() << '\n';
    csv_stream_.flush();
  }
  return latest_;
}

bool OptimizedGlobalMap::SaveLatest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (options_.pcd_path.empty() || !latest_.cloud || latest_.cloud->empty())
    return false;
  const std::filesystem::path path(options_.pcd_path);
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path());
  return pcl::io::savePCDFileBinaryCompressed(
             path.string(), *latest_.cloud) == 0;
}

OptimizedGlobalMap::BuildResult OptimizedGlobalMap::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

}  // namespace my_livo::backend
