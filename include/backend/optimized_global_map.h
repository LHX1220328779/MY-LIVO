#ifndef MY_LIVO_BACKEND_OPTIMIZED_GLOBAL_MAP_H
#define MY_LIVO_BACKEND_OPTIMIZED_GLOBAL_MAP_H

#include "backend/keyframe.h"

#include <cstddef>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace my_livo::backend
{

// Rebuilds a globally consistent point cloud from immutable body-frame
// keyframe clouds and one atomic snapshot of optimized graph poses.  It never
// reads or mutates the frontend voxel map.
class OptimizedGlobalMap
{
public:
  struct Options
  {
    double voxel_leaf_size_m = 0.75;
    double tile_size_m = 60.0;
    std::size_t periodic_keyframe_interval = 20;
    std::size_t minimum_graph_rebuild_keyframe_interval = 10;
    double incremental_max_pose_change_m = 0.10;
    double incremental_max_pose_change_deg = 0.25;
    std::string csv_path;
    std::string pcd_path;
  };

  struct BuildResult
  {
    KeyframeCloud::ConstPtr cloud;
    std::size_t revision = 0;
    std::size_t keyframes = 0;
    std::size_t input_points = 0;
    std::size_t output_points = 0;
    std::size_t processed_keyframes = 0;
    std::size_t updated_tiles = 0;
    std::size_t total_tiles = 0;
    std::string build_mode;
    double build_time_ms = 0.0;
  };

  explicit OptimizedGlobalMap(const Options &options);
  ~OptimizedGlobalMap() = default;

  bool NeedsPeriodicBuild(std::size_t keyframe_count) const;
  bool CanBuildGraphUpdate(std::size_t keyframe_count) const;
  bool CorrectionRequiresFullRebuild(double position_change_m,
                                     double angle_change_deg) const;
  BuildResult Build(const std::vector<Keyframe::Ptr> &keyframes,
                    const std::vector<Pose3d> &optimized_poses,
                    const std::string &reason);
  bool SaveLatest() const;
  BuildResult latest() const;

private:
  struct TileKey
  {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator<(const TileKey &other) const
    {
      if (x != other.x) return x < other.x;
      if (y != other.y) return y < other.y;
      return z < other.z;
    }
  };

  static void ValidateOptions(const Options &options);

  Options options_;
  mutable std::mutex mutex_;
  mutable std::mutex build_mutex_;
  BuildResult latest_;
  std::vector<Pose3d> latest_poses_;
  std::map<TileKey, KeyframeCloud::ConstPtr> tiles_;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_OPTIMIZED_GLOBAL_MAP_H
