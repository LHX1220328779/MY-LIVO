#include "backend/low_frequency_correction_field.h"

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
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kDegreesToRadians = kPi / 180.0;

double Clamp(double value, double magnitude)
{
  return std::clamp(value, -magnitude, magnitude);
}

double WrapRadians(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double QuinticSmoothstep(double value)
{
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

Eigen::Vector2d LimitNorm(const Eigen::Vector2d &value, double limit)
{
  const double norm = value.norm();
  if (norm <= limit || norm <= 1.0e-15) return value;
  return value * (limit / norm);
}
}  // namespace

const char *CorrectionFieldDecisionToString(
    CorrectionFieldDecision decision)
{
  switch (decision)
  {
    case CorrectionFieldDecision::kDisabled:
      return "disabled";
    case CorrectionFieldDecision::kAnchor:
      return "anchor";
    case CorrectionFieldDecision::kWarmup:
      return "monitor_warmup";
    case CorrectionFieldDecision::kSpacing:
      return "knot_spacing";
    case CorrectionFieldDecision::kUpdated:
      return "updated";
    case CorrectionFieldDecision::kFrozenRelocalization:
      return "frozen_relocalization";
  }
  return "unknown";
}

LowFrequencyCorrectionField::LowFrequencyCorrectionField(
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
          "Cannot open low-frequency correction CSV: " + path.string());
    csv_stream_
        << "keyframe_id,timestamp,regime,decision,distance_m,knot_count,"
           "field_changed,frozen,correction_x,correction_y,correction_z,"
           "correction_yaw_deg,elastic_distance_m,elastic_stiffness,"
           "radial_force_proxy_m,residual_before_m,residual_after_m,"
           "planar_step_m,vertical_step_m,yaw_step_deg\n";
    csv_stream_.flush();
  }
}

void LowFrequencyCorrectionField::ValidateOptions(const Options &options)
{
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(options.knot_spacing_m) ||
      !positive(options.spatial_low_pass_length_m) ||
      !std::isfinite(options.elastic_soft_radius_m) ||
      options.elastic_soft_radius_m < 0.0 ||
      !positive(options.elastic_full_radius_m) ||
      options.elastic_full_radius_m <= options.elastic_soft_radius_m ||
      !std::isfinite(options.elastic_minimum_stiffness) ||
      options.elastic_minimum_stiffness < 0.0 ||
      !positive(options.elastic_maximum_stiffness) ||
      options.elastic_maximum_stiffness <
          options.elastic_minimum_stiffness ||
      options.elastic_maximum_stiffness > 1.0 ||
      !positive(options.maximum_planar_gradient_m_per_m) ||
      !positive(options.maximum_vertical_gradient_m_per_m) ||
      !positive(options.maximum_yaw_gradient_deg_per_m) ||
      !positive(options.maximum_planar_update_m) ||
      !positive(options.maximum_vertical_update_m) ||
      !positive(options.maximum_yaw_update_deg))
    throw std::invalid_argument(
        "Low-frequency correction-field limits must be finite and positive.");
}

double LowFrequencyCorrectionField::ElasticStiffness(
    double distance_m) const
{
  if (!std::isfinite(distance_m) || distance_m < 0.0)
    throw std::invalid_argument("Elastic distance must be finite and non-negative.");
  const double normalized =
      (distance_m - options_.elastic_soft_radius_m) /
      (options_.elastic_full_radius_m - options_.elastic_soft_radius_m);
  return options_.elastic_minimum_stiffness +
      (options_.elastic_maximum_stiffness -
       options_.elastic_minimum_stiffness) *
          QuinticSmoothstep(normalized);
}

void LowFrequencyCorrectionField::AccumulateTarget(
    const Eigen::Vector3d &target, const Eigen::Matrix3d &covariance)
{
  for (int axis = 0; axis < 3; ++axis)
  {
    const double variance = covariance(axis, axis);
    if (!std::isfinite(variance) || variance <= 0.0)
      throw std::invalid_argument(
          "Correction-field covariance diagonal must be positive.");
    const double weight = 1.0 / variance;
    pending_weighted_target_[axis] += weight * target[axis];
    pending_weight_[axis] += weight;
  }
}

void LowFrequencyCorrectionField::ClearPending()
{
  pending_weighted_target_.setZero();
  pending_weight_.setZero();
}

LowFrequencyCorrectionField::UpdateResult
LowFrequencyCorrectionField::AddObservation(
    std::uint64_t keyframe_id, double timestamp,
    double cumulative_distance_m, const Eigen::Vector3d &base_position,
    const Eigen::Vector3d &rtk_position,
    const Eigen::Matrix3d &position_covariance,
    CorrectionRegime regime)
{
  if (!std::isfinite(timestamp) || !std::isfinite(cumulative_distance_m) ||
      cumulative_distance_m < 0.0 || !base_position.allFinite() ||
      !rtk_position.allFinite() || !position_covariance.allFinite())
    throw std::invalid_argument("Correction-field observation is invalid.");
  if (!knots_.empty() &&
      (keyframe_id <= knots_.back().keyframe_id ||
       timestamp <= knots_.back().timestamp ||
       cumulative_distance_m < knots_.back().distance_m))
    throw std::logic_error(
        "Correction-field observations must be ordered after its last knot.");

  UpdateResult result;
  result.keyframe_id = keyframe_id;
  result.cumulative_distance_m = cumulative_distance_m;
  const Correction current = Evaluate(cumulative_distance_m);
  const Eigen::Vector3d elastic_error =
      base_position + current.displacement - rtk_position;
  result.elastic_distance_m = elastic_error.norm();
  result.elastic_stiffness = ElasticStiffness(result.elastic_distance_m);
  result.radial_force_proxy_m =
      result.elastic_stiffness * result.elastic_distance_m;
  result.residual_before_m = result.elastic_distance_m;
  ++statistics_.observations;

  if (!options_.enabled)
  {
    result.decision = CorrectionFieldDecision::kDisabled;
  }
  else if (frozen_ || regime == CorrectionRegime::kRelocalizationRequired)
  {
    frozen_ = true;
    ClearPending();
    result.decision = CorrectionFieldDecision::kFrozenRelocalization;
    result.frozen = true;
    ++statistics_.frozen_observations;
  }
  else if (knots_.empty())
  {
    Knot anchor;
    anchor.keyframe_id = keyframe_id;
    anchor.timestamp = timestamp;
    anchor.distance_m = cumulative_distance_m;
    anchor.source_base_position = base_position;
    anchor.source_rtk_position = rtk_position;
    knots_.push_back(anchor);  // Gauge constraint: C(s0) = identity.
    result.decision = CorrectionFieldDecision::kAnchor;
  }
  else
  {
    AccumulateTarget(rtk_position - base_position, position_covariance);
    const double spacing = cumulative_distance_m - knots_.back().distance_m;
    if (regime == CorrectionRegime::kWarmup)
    {
      result.decision = CorrectionFieldDecision::kWarmup;
    }
    else if (spacing + 1.0e-9 < options_.knot_spacing_m)
    {
      result.decision = CorrectionFieldDecision::kSpacing;
    }
    else
    {
      const Knot &previous = knots_.back();
      Correction next = previous.correction;
      const Eigen::Vector3d target =
          pending_weighted_target_.cwiseQuotient(pending_weight_);
      const double spatial_gain =
          1.0 - std::exp(-spacing / options_.spatial_low_pass_length_m);
      const double elastic_gain =
          spatial_gain * result.elastic_stiffness;

      Eigen::Vector2d planar_step =
          elastic_gain *
          (target.head<2>() - next.displacement.head<2>());
      // smoothstep's maximum derivative is 1.5 * delta / knot spacing.
      const double planar_limit = std::min(
          options_.maximum_planar_update_m,
          options_.maximum_planar_gradient_m_per_m * spacing / 1.5);
      planar_step = LimitNorm(planar_step, planar_limit);
      next.displacement.head<2>() += planar_step;

      double vertical_step =
          elastic_gain * (target.z() - next.displacement.z());
      const double vertical_limit = std::min(
          options_.maximum_vertical_update_m,
          options_.maximum_vertical_gradient_m_per_m * spacing / 1.5);
      vertical_step = Clamp(vertical_step, vertical_limit);
      next.displacement.z() += vertical_step;

      double desired_yaw = previous.correction.yaw_rad;
      const Eigen::Vector2d local_chord =
          base_position.head<2>() -
          previous.source_base_position.head<2>();
      const Eigen::Vector2d rtk_chord =
          rtk_position.head<2>() -
          previous.source_rtk_position.head<2>();
      if (local_chord.norm() >= 5.0 && rtk_chord.norm() >= 5.0)
        desired_yaw = WrapRadians(
            std::atan2(rtk_chord.y(), rtk_chord.x()) -
            std::atan2(local_chord.y(), local_chord.x()));
      double yaw_step = elastic_gain * WrapRadians(
          desired_yaw - previous.correction.yaw_rad);
      const double yaw_limit = kDegreesToRadians * std::min(
          options_.maximum_yaw_update_deg,
          options_.maximum_yaw_gradient_deg_per_m * spacing / 1.5);
      yaw_step = Clamp(yaw_step, yaw_limit);
      next.yaw_rad = WrapRadians(next.yaw_rad + yaw_step);

      Knot knot;
      knot.keyframe_id = keyframe_id;
      knot.timestamp = timestamp;
      knot.distance_m = cumulative_distance_m;
      knot.source_base_position = base_position;
      knot.source_rtk_position = rtk_position;
      knot.correction = next;
      knots_.push_back(knot);
      ClearPending();

      result.decision = CorrectionFieldDecision::kUpdated;
      result.field_changed = true;
      result.planar_step_m = planar_step.norm();
      result.vertical_step_m = std::abs(vertical_step);
      result.yaw_step_deg = std::abs(yaw_step) * kRadiansToDegrees;
      ++statistics_.updates;
    }
  }

  statistics_.knots = knots_.size();
  result.knot_count = knots_.size();
  result.frozen = frozen_;
  result.correction = Evaluate(cumulative_distance_m);
  Pose3d base_pose(Eigen::Quaterniond::Identity(), base_position);
  result.residual_after_m =
      (Apply(cumulative_distance_m, base_pose).translation - rtk_position)
          .norm();
  WriteCsv(timestamp, regime, result);
  return result;
}

LowFrequencyCorrectionField::Correction
LowFrequencyCorrectionField::Evaluate(double cumulative_distance_m) const
{
  if (!std::isfinite(cumulative_distance_m))
    throw std::invalid_argument("Correction-field query is not finite.");
  if (knots_.empty() || cumulative_distance_m <= knots_.front().distance_m)
    return Correction();
  if (cumulative_distance_m >= knots_.back().distance_m)
    return knots_.back().correction;

  const auto upper = std::upper_bound(
      knots_.begin(), knots_.end(), cumulative_distance_m,
      [](double distance, const Knot &knot) {
        return distance < knot.distance_m;
      });
  const Knot &right = *upper;
  const Knot &left = *(upper - 1);
  const double u = (cumulative_distance_m - left.distance_m) /
      (right.distance_m - left.distance_m);
  const double weight = u * u * (3.0 - 2.0 * u);
  Correction correction;
  correction.displacement =
      (1.0 - weight) * left.correction.displacement +
      weight * right.correction.displacement;
  correction.yaw_rad = WrapRadians(
      left.correction.yaw_rad + weight * WrapRadians(
          right.correction.yaw_rad - left.correction.yaw_rad));
  return correction;
}

Pose3d LowFrequencyCorrectionField::Apply(
    double cumulative_distance_m, const Pose3d &base_pose) const
{
  if (!base_pose.isFinite())
    throw std::invalid_argument("Correction-field base pose is invalid.");
  const Correction correction = Evaluate(cumulative_distance_m);
  const Eigen::Quaterniond yaw(
      Eigen::AngleAxisd(correction.yaw_rad, Eigen::Vector3d::UnitZ()));
  return Pose3d(yaw * base_pose.rotation,
                base_pose.translation + correction.displacement);
}

void LowFrequencyCorrectionField::WriteCsv(
    double timestamp, CorrectionRegime regime,
    const UpdateResult &result)
{
  if (!csv_stream_.is_open()) return;
  csv_stream_ << std::setprecision(17) << result.keyframe_id << ','
              << timestamp << ',' << CorrectionRegimeToString(regime) << ','
              << CorrectionFieldDecisionToString(result.decision) << ','
              << result.cumulative_distance_m << ',' << result.knot_count
              << ',' << static_cast<int>(result.field_changed) << ','
              << static_cast<int>(result.frozen) << ','
              << result.correction.displacement.x() << ','
              << result.correction.displacement.y() << ','
              << result.correction.displacement.z() << ','
              << result.correction.yaw_rad * kRadiansToDegrees << ','
              << result.elastic_distance_m << ','
              << result.elastic_stiffness << ','
              << result.radial_force_proxy_m << ','
              << result.residual_before_m << ',' << result.residual_after_m
              << ',' << result.planar_step_m << ',' << result.vertical_step_m
              << ',' << result.yaw_step_deg << '\n';
  csv_stream_.flush();
}

}  // namespace my_livo::backend
