#include "backend/correction_feasibility_monitor.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace my_livo::backend
{
namespace
{
constexpr double kRadiansToDegrees =
    180.0 / 3.14159265358979323846;

double WrapRadians(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

const char *CorrectionRegimeToString(CorrectionRegime regime)
{
  switch (regime)
  {
    case CorrectionRegime::kWarmup:
      return "warmup";
    case CorrectionRegime::kElastic:
      return "elastic";
    case CorrectionRegime::kDegraded:
      return "degraded";
    case CorrectionRegime::kRelocalizationRequired:
      return "relocalization_required";
  }
  return "unknown";
}

CorrectionFeasibilityMonitor::CorrectionFeasibilityMonitor(
    const Options &options)
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
      throw std::runtime_error(
          "Cannot open correction-feasibility CSV: " + path.string());
    csv_stream_
        << "keyframe_id,timestamp,regime,state_changed,"
           "elastic_correction_allowed,residual_m,baseline_keyframe_id,"
           "baseline_m,correction_gradient_m_per_m,"
           "planar_gradient_m_per_m,vertical_gradient_m_per_m,"
           "chord_scale_ratio,heading_disagreement_deg,local_x,local_y,"
           "local_z,rtk_x,rtk_y,rtk_z\n";
    csv_stream_.flush();
  }
}

void CorrectionFeasibilityMonitor::ValidateOptions(const Options &options)
{
  if (!std::isfinite(options.minimum_baseline_m) ||
      options.minimum_baseline_m <= 0.0)
    throw std::invalid_argument(
        "Correction monitor baseline must be finite and positive.");
  if (!std::isfinite(options.elastic_gradient_limit_m_per_m) ||
      options.elastic_gradient_limit_m_per_m <= 0.0 ||
      !std::isfinite(options.relocalization_gradient_m_per_m) ||
      options.relocalization_gradient_m_per_m <=
          options.elastic_gradient_limit_m_per_m)
    throw std::invalid_argument(
        "Correction monitor gradients must be positive and ordered.");
  if (options.relocalization_consecutive_observations <= 0)
    throw std::invalid_argument(
        "Correction monitor evidence count must be positive.");
}

CorrectionFeasibilityMonitor::Result
CorrectionFeasibilityMonitor::AddObservation(
    std::uint64_t keyframe_id, double timestamp,
    double cumulative_distance_m,
    const Eigen::Vector3d &aligned_local_position,
    const Eigen::Vector3d &rtk_position)
{
  if (!std::isfinite(timestamp) || !std::isfinite(cumulative_distance_m) ||
      cumulative_distance_m < 0.0 || !aligned_local_position.allFinite() ||
      !rtk_position.allFinite())
    throw std::invalid_argument(
        "Correction monitor observation is invalid.");
  if (!samples_.empty() &&
      (keyframe_id <= samples_.back().keyframe_id ||
       timestamp <= samples_.back().timestamp ||
       cumulative_distance_m < samples_.back().cumulative_distance_m))
    throw std::logic_error(
        "Correction monitor observations must be ordered.");

  Sample sample;
  sample.keyframe_id = keyframe_id;
  sample.timestamp = timestamp;
  sample.cumulative_distance_m = cumulative_distance_m;
  sample.local_position = aligned_local_position;
  sample.rtk_position = rtk_position;
  sample.residual = aligned_local_position - rtk_position;

  Result result;
  result.keyframe_id = keyframe_id;
  result.residual_m = sample.residual.norm();
  std::uint64_t baseline_keyframe_id = 0;
  const Sample *baseline = nullptr;
  for (auto iterator = samples_.rbegin(); iterator != samples_.rend();
       ++iterator)
  {
    if (cumulative_distance_m - iterator->cumulative_distance_m + 1.0e-9 >=
        options_.minimum_baseline_m)
    {
      baseline = &*iterator;
      break;
    }
  }

  const CorrectionRegime previous_regime = regime_;
  if (baseline != nullptr)
  {
    result.baseline_available = true;
    baseline_keyframe_id = baseline->keyframe_id;
    result.baseline_m =
        cumulative_distance_m - baseline->cumulative_distance_m;
    const Eigen::Vector3d residual_delta =
        sample.residual - baseline->residual;
    result.correction_gradient_m_per_m =
        residual_delta.norm() / result.baseline_m;
    result.planar_gradient_m_per_m =
        residual_delta.head<2>().norm() / result.baseline_m;
    result.vertical_gradient_m_per_m =
        std::abs(residual_delta.z()) / result.baseline_m;

    const Eigen::Vector3d local_chord =
        sample.local_position - baseline->local_position;
    const Eigen::Vector3d rtk_chord =
        sample.rtk_position - baseline->rtk_position;
    if (local_chord.norm() > 1.0e-6)
      result.chord_scale_ratio = rtk_chord.norm() / local_chord.norm();
    if (local_chord.head<2>().norm() > 1.0 &&
        rtk_chord.head<2>().norm() > 1.0)
    {
      result.heading_disagreement_deg = kRadiansToDegrees * WrapRadians(
          std::atan2(rtk_chord.y(), rtk_chord.x()) -
          std::atan2(local_chord.y(), local_chord.x()));
    }

    if (regime_ != CorrectionRegime::kRelocalizationRequired)
    {
      if (result.correction_gradient_m_per_m >=
          options_.relocalization_gradient_m_per_m)
        ++consecutive_relocalization_evidence_;
      else
        consecutive_relocalization_evidence_ = 0;

      if (consecutive_relocalization_evidence_ >=
          options_.relocalization_consecutive_observations)
        regime_ = CorrectionRegime::kRelocalizationRequired;
      else if (result.correction_gradient_m_per_m >
               options_.elastic_gradient_limit_m_per_m)
        regime_ = CorrectionRegime::kDegraded;
      else
        regime_ = CorrectionRegime::kElastic;
    }
  }

  result.regime = regime_;
  result.state_changed = regime_ != previous_regime;
  result.elastic_correction_allowed =
      regime_ == CorrectionRegime::kElastic ||
      regime_ == CorrectionRegime::kDegraded;
  samples_.push_back(sample);

  ++statistics_.observations;
  switch (regime_)
  {
    case CorrectionRegime::kWarmup:
      ++statistics_.warmup;
      break;
    case CorrectionRegime::kElastic:
      ++statistics_.elastic;
      break;
    case CorrectionRegime::kDegraded:
      ++statistics_.degraded;
      break;
    case CorrectionRegime::kRelocalizationRequired:
      ++statistics_.relocalization_required;
      break;
  }
  statistics_.maximum_gradient_m_per_m = std::max(
      statistics_.maximum_gradient_m_per_m,
      result.correction_gradient_m_per_m);
  WriteCsv(result, sample, baseline_keyframe_id);
  return result;
}

void CorrectionFeasibilityMonitor::WriteCsv(
    const Result &result, const Sample &sample,
    std::uint64_t baseline_keyframe_id)
{
  if (!csv_stream_.is_open()) return;
  csv_stream_ << std::setprecision(17) << sample.keyframe_id << ','
              << sample.timestamp << ','
              << CorrectionRegimeToString(result.regime) << ','
              << static_cast<int>(result.state_changed) << ','
              << static_cast<int>(result.elastic_correction_allowed) << ','
              << result.residual_m << ',';
  if (result.baseline_available)
    csv_stream_ << baseline_keyframe_id << ',' << result.baseline_m << ',';
  else
    csv_stream_ << ",,";
  csv_stream_ << result.correction_gradient_m_per_m << ','
              << result.planar_gradient_m_per_m << ','
              << result.vertical_gradient_m_per_m << ','
              << result.chord_scale_ratio << ','
              << result.heading_disagreement_deg << ','
              << sample.local_position.x() << ','
              << sample.local_position.y() << ','
              << sample.local_position.z() << ','
              << sample.rtk_position.x() << ','
              << sample.rtk_position.y() << ','
              << sample.rtk_position.z() << '\n';
  csv_stream_.flush();
}

}  // namespace my_livo::backend
