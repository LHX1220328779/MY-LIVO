#ifndef MY_LIVO_BACKEND_ELASTIC_SEGMENT_ACCEPTANCE_MONITOR_H
#define MY_LIVO_BACKEND_ELASTIC_SEGMENT_ACCEPTANCE_MONITOR_H

#include <cstddef>
#include <cstdint>
#include <deque>

namespace my_livo::backend
{

// Accepts a quarantined recovery segment only after a sustained, bounded
// position/yaw/velocity/field window.  It never changes the local LIO state.
class ElasticSegmentAcceptanceMonitor
{
public:
  struct Options
  {
    bool enabled = true;
    std::size_t minimum_position_observations = 6;
    double minimum_path_length_m = 30.0;
    double maximum_position_residual_m = 0.30;
    double maximum_yaw_residual_deg = 0.50;
    double maximum_post_velocity_error_mps = 0.80;
    double maximum_field_gradient_m_per_m = 0.30;
    double maximum_yaw_gradient_deg_per_m = 0.20;
    double maximum_outlier_fraction = 0.25;
  };

  enum class Decision
  {
    kMonitoring,
    kRejected,
    kAccepted,
    kLatched,
  };

  struct Result
  {
    Decision decision = Decision::kMonitoring;
    std::uint64_t keyframe_id = 0;
    std::uint64_t window_start_keyframe_id = 0;
    std::size_t window_size = 0;
    std::size_t valid_samples = 0;
    double valid_ratio = 0.0;
    double window_path_length_m = 0.0;
    bool accepted = false;
    bool state_changed = false;
  };

  ElasticSegmentAcceptanceMonitor();
  explicit ElasticSegmentAcceptanceMonitor(const Options &options);

  Result AddObservation(
      std::uint64_t keyframe_id, double cumulative_distance_m,
      double position_residual_m, double yaw_residual_deg,
      double post_velocity_error_mps, bool velocity_baseline_available,
      bool guard_active, double field_gradient_m_per_m,
      double yaw_gradient_deg_per_m);
  void Reset();

private:
  static void ValidateOptions(const Options &options);

  Options options_;
  struct Sample
  {
    std::uint64_t keyframe_id = 0;
    double distance_m = 0.0;
    bool valid = false;
  };
  bool accepted_ = false;
  std::uint64_t window_start_keyframe_id_ = 0;
  double window_start_distance_m_ = 0.0;
  std::size_t window_size_ = 0;
  std::deque<Sample> samples_;
};

const char *ElasticSegmentAcceptanceDecisionToString(
    ElasticSegmentAcceptanceMonitor::Decision decision);

}  // namespace my_livo::backend

#endif
