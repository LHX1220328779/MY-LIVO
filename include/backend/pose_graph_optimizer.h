#ifndef MY_LIVO_BACKEND_POSE_GRAPH_OPTIMIZER_H
#define MY_LIVO_BACKEND_POSE_GRAPH_OPTIMIZER_H

#include "backend/keyframe.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace my_livo::backend
{

// Persistent odometry pose graph.  New keyframes append one pose node and,
// except for node zero, exactly one relative-pose factor:
//
//   Z_(k-1,k) = inverse(T_odom_(k-1)) * T_odom_k.
//
// A tight prior on the first node removes the global SE(3) gauge freedom.
// Backend estimates are written only to Keyframe::T_map_body; raw frontend
// poses stay immutable and are never fed back into FAST-LIVO2.
class PoseGraphOptimizer
{
public:
  struct Options
  {
    double odometry_translation_sigma_m = 0.10;
    double odometry_rotation_sigma_deg = 0.50;
    double prior_translation_sigma_m = 1.0e-6;
    double prior_rotation_sigma_deg = 1.0e-4;
    double relinearize_threshold = 0.01;
    int relinearize_skip = 1;
    double wildfire_threshold = 0.001;
    int additional_update_steps = 1;
    std::string csv_path;
  };

  struct UpdateResult
  {
    bool optimization_ran = false;
    bool solution_usable = true;
    int iterations = 0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double optimization_time_ms = 0.0;
    std::uint64_t variables_relinearized = 0;
    std::uint64_t variables_reeliminated = 0;
    Pose3d latest_pose;
  };

  struct Statistics
  {
    std::uint64_t nodes = 0;
    std::uint64_t odometry_factors = 0;
    std::uint64_t optimization_runs = 0;
    std::uint64_t failed_optimizations = 0;
    std::uint64_t variables_relinearized = 0;
    std::uint64_t variables_reeliminated = 0;
    double last_optimization_time_ms = 0.0;
    double maximum_optimization_time_ms = 0.0;
  };

  explicit PoseGraphOptimizer(const Options &options);
  ~PoseGraphOptimizer();

  PoseGraphOptimizer(const PoseGraphOptimizer &) = delete;
  PoseGraphOptimizer &operator=(const PoseGraphOptimizer &) = delete;

  UpdateResult AddKeyframe(const Keyframe::Ptr &keyframe);
  std::vector<Pose3d> optimized_poses() const;
  Statistics statistics() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_POSE_GRAPH_OPTIMIZER_H
