#include "backend/optimized_global_map.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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
    csv_stream_ << "revision,reason,build_mode,keyframes,input_points,"
                   "output_points,processed_keyframes,updated_tiles,"
                   "total_tiles,build_time_ms,min_x,min_y,min_z,max_x,"
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
  if (options.periodic_keyframe_interval == 0)
    throw std::invalid_argument(
        "Optimized global-map keyframe interval must be positive.");
  if (options.minimum_graph_rebuild_keyframe_interval == 0)
    throw std::invalid_argument(
        "Optimized global-map graph rebuild interval must be positive.");
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
  // Build() can be called by the asynchronous online worker and by the final
  // shutdown save.  Serializing the complete operation prevents an older
  // snapshot from overwriting a newer revision.
  std::lock_guard<std::mutex> build_lock(build_mutex_);
  if (keyframes.empty() || keyframes.size() != optimized_poses.size())
    throw std::invalid_argument(
        "Optimized global map requires equal, non-empty keyframe and pose "
        "snapshots.");
  const auto start = std::chrono::steady_clock::now();
  std::size_t input_points = 0;
  for (std::size_t index = 0; index < keyframes.size(); ++index)
  {
    if (!keyframes[index] || keyframes[index]->id() != index ||
        !keyframes[index]->cloud_body() ||
        !optimized_poses[index].isFinite())
      throw std::invalid_argument(
          "Optimized global-map snapshot contains an invalid keyframe.");
    input_points += keyframes[index]->cloud_body()->size();
  }

  std::map<TileKey, KeyframeCloud::ConstPtr> tiles;
  std::vector<Pose3d> previous_poses;
  std::size_t previous_keyframes = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tiles = tiles_;
    previous_poses = latest_poses_;
    previous_keyframes = latest_.keyframes;
  }

  bool incremental = reason == "periodic" && previous_keyframes > 0 &&
                     previous_keyframes < keyframes.size() &&
                     previous_poses.size() == previous_keyframes &&
                     !tiles.empty();
  if (incremental)
  {
    for (std::size_t index = 0; index < previous_keyframes; ++index)
    {
      const double position_change =
          (previous_poses[index].translation -
           optimized_poses[index].translation).norm();
      const double angle_change = 180.0 / std::acos(-1.0) *
          previous_poses[index].rotation.angularDistance(
              optimized_poses[index].rotation);
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

  std::map<TileKey, KeyframeCloud::Ptr> additions;
  for (std::size_t index = previous_keyframes; index < keyframes.size(); ++index)
  {
    const Pose3d &pose = optimized_poses[index];
    for (const KeyframePoint &point_body : keyframes[index]->cloud_body()->points)
    {
      const Eigen::Vector3d point_map = pose * Eigen::Vector3d(
          point_body.x, point_body.y, point_body.z);
      if (!point_map.allFinite()) continue;
      const TileKey tile_key{
          static_cast<std::int64_t>(std::floor(
              point_map.x() / options_.tile_size_m)),
          static_cast<std::int64_t>(std::floor(
              point_map.y() / options_.tile_size_m)),
          static_cast<std::int64_t>(std::floor(
              point_map.z() / options_.tile_size_m))};
      auto &addition = additions[tile_key];
      if (!addition) addition.reset(new KeyframeCloud());
      KeyframePoint point;
      point.x = static_cast<float>(point_map.x());
      point.y = static_cast<float>(point_map.y());
      point.z = static_cast<float>(point_map.z());
      point.intensity = point_body.intensity;
      addition->push_back(point);
    }
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
  latest_poses_ = optimized_poses;
  latest_.cloud = filtered;
  ++latest_.revision;
  latest_.keyframes = keyframes.size();
  latest_.input_points = input_points;
  latest_.output_points = filtered->size();
  latest_.processed_keyframes = keyframes.size() - previous_keyframes;
  latest_.updated_tiles = additions.size();
  latest_.total_tiles = tiles_.size();
  latest_.build_mode = incremental ? "incremental" : "full";
  latest_.build_time_ms = elapsed_ms;
  if (csv_stream_.is_open())
  {
    csv_stream_ << std::setprecision(17) << latest_.revision << ','
                << reason << ',' << latest_.build_mode << ','
                << latest_.keyframes << ',' << latest_.input_points << ','
                << latest_.output_points << ','
                << latest_.processed_keyframes << ','
                << latest_.updated_tiles << ',' << latest_.total_tiles << ','
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
