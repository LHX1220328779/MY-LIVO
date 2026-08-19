#ifndef MY_LIVO_BACKEND_LOOP_REGISTRATION_H
#define MY_LIVO_BACKEND_LOOP_REGISTRATION_H

#include "backend/loop_candidate_detector.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace my_livo::backend
{

enum class LoopRegistrationStatus : std::uint8_t
{
  kCompleted = 0,
  kInvalidInput,
  kInsufficientSourcePoints,
  kInsufficientTargetPoints,
  kNdtException,
  kNonfiniteResult,
};

const char *LoopRegistrationStatusToString(LoopRegistrationStatus status);

struct LoopRegistrationLevelResult
{
  double resolution_m = 0.0;
  double voxel_leaf_size_m = 0.0;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  bool converged = false;
  int iterations = 0;
  double transformation_probability = 0.0;
  double fitness_score_m2 = 0.0;
  double elapsed_ms = 0.0;
};

struct LoopRegistrationResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LoopCandidate candidate;
  LoopRegistrationStatus status = LoopRegistrationStatus::kInvalidInput;
  std::string diagnostic;
  bool converged = false;
  std::size_t converged_levels = 0;
  std::size_t raw_source_points = 0;
  std::size_t raw_target_points = 0;
  std::size_t final_source_points = 0;
  std::size_t final_target_points = 0;
  double transformation_probability = 0.0;
  double fitness_score_m2 = 0.0;
  double overlap = 0.0;
  double overlap_rmse_m = 0.0;
  Pose3d T_candidate_current_initial;
  Pose3d T_candidate_current;
  Pose3d T_map_candidate_snapshot;
  Pose3d T_map_current_snapshot;
  double correction_translation_m = 0.0;
  double correction_angle_deg = 0.0;
  double registration_time_ms = 0.0;
  std::vector<LoopRegistrationLevelResult> levels;
};

// Multi-resolution PCL NDT loop registration. Runtime jobs execute on one
// background worker so registration never blocks FAST-LIVO2 state estimation.
// Results are diagnostic only: this class has no access to the pose graph.
class LoopRegistration
{
public:
  struct Options
  {
    int target_submap_half_width_keyframes = 40;
    int target_submap_stride_keyframes = 4;
    std::vector<double> resolutions_m{10.0, 5.0, 2.0, 1.0};
    double voxel_leaf_size_ratio = 0.25;
    double minimum_voxel_leaf_size_m = 0.50;
    double transformation_epsilon = 0.05;
    double step_size = 0.70;
    int maximum_iterations = 40;
    double overlap_max_correspondence_distance_m = 1.0;
    std::size_t minimum_source_points = 100;
    std::size_t minimum_target_points = 300;
    std::size_t maximum_queue_size = 64;
    std::string registration_csv_path;
    std::string level_csv_path;
  };

  struct Statistics
  {
    std::uint64_t enqueued = 0;
    std::uint64_t completed = 0;
    std::uint64_t converged = 0;
    std::uint64_t failed = 0;
    std::uint64_t queue_drops = 0;
    std::size_t maximum_queue_depth = 0;
    double total_registration_time_ms = 0.0;
  };

  using ResultCallback =
      std::function<void(const LoopRegistrationResult &)>;

  explicit LoopRegistration(const Options &options);
  ~LoopRegistration();

  LoopRegistration(const LoopRegistration &) = delete;
  LoopRegistration &operator=(const LoopRegistration &) = delete;

  // Synchronous entry point used by deterministic tests and offline tools.
  // It does not write CSV or update asynchronous worker statistics.
  LoopRegistrationResult Register(
      const LoopCandidate &candidate,
      const std::vector<Keyframe::Ptr> &keyframes) const;

  // Runtime entry point. Keyframe poses and cloud pointers needed by the job
  // are frozen before it enters the queue.
  bool Enqueue(const LoopCandidate &candidate,
               const std::vector<Keyframe::Ptr> &keyframes);
  void SetResultCallback(ResultCallback callback);
  void WaitUntilIdle();
  Statistics statistics() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_LOOP_REGISTRATION_H
