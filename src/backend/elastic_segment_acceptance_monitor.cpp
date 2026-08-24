#include "backend/elastic_segment_acceptance_monitor.h"

#include <cmath>
#include <stdexcept>

namespace my_livo::backend
{

ElasticSegmentAcceptanceMonitor::ElasticSegmentAcceptanceMonitor()
    : ElasticSegmentAcceptanceMonitor(Options())
{
}

ElasticSegmentAcceptanceMonitor::ElasticSegmentAcceptanceMonitor(
    const Options &options) : options_(options)
{
  ValidateOptions(options_);
}

void ElasticSegmentAcceptanceMonitor::ValidateOptions(
    const Options &options)
{
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (options.minimum_position_observations < 2U ||
      !positive(options.minimum_path_length_m) ||
      !positive(options.maximum_position_residual_m) ||
      !positive(options.maximum_yaw_residual_deg) ||
      !positive(options.maximum_post_velocity_error_mps) ||
      !positive(options.maximum_field_gradient_m_per_m) ||
      !positive(options.maximum_yaw_gradient_deg_per_m) ||
      !std::isfinite(options.maximum_outlier_fraction) ||
      options.maximum_outlier_fraction < 0.0 ||
      options.maximum_outlier_fraction >= 0.5)
    throw std::invalid_argument(
        "Elastic-segment acceptance options are invalid.");
}

void ElasticSegmentAcceptanceMonitor::Reset()
{
  accepted_ = false;
  window_start_keyframe_id_ = 0;
  window_start_distance_m_ = 0.0;
  window_size_ = 0;
  samples_.clear();
}

ElasticSegmentAcceptanceMonitor::Result
ElasticSegmentAcceptanceMonitor::AddObservation(
    std::uint64_t keyframe_id, double cumulative_distance_m,
    double position_residual_m, double yaw_residual_deg,
    double post_velocity_error_mps, bool velocity_baseline_available,
    bool guard_active, double field_gradient_m_per_m,
    double yaw_gradient_deg_per_m)
{
  const double values[] = {
      cumulative_distance_m, position_residual_m, yaw_residual_deg,
      post_velocity_error_mps, field_gradient_m_per_m,
      yaw_gradient_deg_per_m};
  for (double value : values)
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument(
          "Elastic-segment acceptance input is invalid.");

  Result result;
  result.keyframe_id = keyframe_id;
  if (!options_.enabled)
    return result;
  if (accepted_)
  {
    result.decision = Decision::kLatched;
    result.accepted = true;
    result.window_start_keyframe_id = window_start_keyframe_id_;
    result.window_size = window_size_;
    result.window_path_length_m =
        cumulative_distance_m - window_start_distance_m_;
    return result;
  }

  const bool valid = velocity_baseline_available && guard_active &&
      position_residual_m <= options_.maximum_position_residual_m &&
      yaw_residual_deg <= options_.maximum_yaw_residual_deg &&
      post_velocity_error_mps <= options_.maximum_post_velocity_error_mps &&
      field_gradient_m_per_m <= options_.maximum_field_gradient_m_per_m &&
      yaw_gradient_deg_per_m <= options_.maximum_yaw_gradient_deg_per_m;
  samples_.push_back({keyframe_id, cumulative_distance_m, valid});
  while (samples_.size() > 1U &&
         cumulative_distance_m - samples_[1].distance_m + 1.0e-12 >=
             options_.minimum_path_length_m)
    samples_.pop_front();
  window_start_keyframe_id_ = samples_.front().keyframe_id;
  window_start_distance_m_ = samples_.front().distance_m;
  window_size_ = samples_.size();
  result.valid_samples = 0;
  for (const Sample &sample : samples_)
    result.valid_samples += sample.valid;
  result.valid_ratio = static_cast<double>(result.valid_samples) /
      static_cast<double>(window_size_);
  result.window_start_keyframe_id = window_start_keyframe_id_;
  result.window_size = window_size_;
  result.window_path_length_m =
      cumulative_distance_m - window_start_distance_m_;
  const bool enough_valid = result.valid_ratio + 1.0e-12 >=
      1.0 - options_.maximum_outlier_fraction;
  if (window_size_ >= options_.minimum_position_observations &&
      result.window_path_length_m + 1.0e-12 >=
          options_.minimum_path_length_m &&
      enough_valid && valid)
  {
    accepted_ = true;
    result.decision = Decision::kAccepted;
    result.accepted = true;
    result.state_changed = true;
  }
  else
  {
    result.decision = valid ? Decision::kMonitoring : Decision::kRejected;
  }
  return result;
}

const char *ElasticSegmentAcceptanceDecisionToString(
    ElasticSegmentAcceptanceMonitor::Decision decision)
{
  switch (decision)
  {
    case ElasticSegmentAcceptanceMonitor::Decision::kMonitoring:
      return "monitoring";
    case ElasticSegmentAcceptanceMonitor::Decision::kRejected:
      return "rejected";
    case ElasticSegmentAcceptanceMonitor::Decision::kAccepted:
      return "accepted";
    case ElasticSegmentAcceptanceMonitor::Decision::kLatched:
      return "latched";
  }
  return "unknown";
}

}  // namespace my_livo::backend
