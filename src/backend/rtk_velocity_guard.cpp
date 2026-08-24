#include "backend/rtk_velocity_guard.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace my_livo::backend
{
namespace
{
Eigen::Vector2d LimitNorm(const Eigen::Vector2d &value, double limit)
{
  const double norm = value.norm();
  if (norm <= limit || norm <= 1.0e-15) return value;
  return value * (limit / norm);
}
}  // namespace

const char *RtkVelocityGuardDecisionToString(
    RtkVelocityGuardDecision decision)
{
  switch (decision)
  {
    case RtkVelocityGuardDecision::kWarmup:
      return "warmup";
    case RtkVelocityGuardDecision::kMonitoring:
      return "monitoring";
    case RtkVelocityGuardDecision::kActivated:
      return "activated";
    case RtkVelocityGuardDecision::kCorrected:
      return "corrected";
    case RtkVelocityGuardDecision::kHealthyVerticalAiding:
      return "healthy_vertical_aiding";
    case RtkVelocityGuardDecision::kRecovered:
      return "recovered";
  }
  return "unknown";
}

const char *RtkVelocitySourceToString(RtkVelocitySource source)
{
  switch (source)
  {
    case RtkVelocitySource::kUnavailable:
      return "unavailable";
    case RtkVelocitySource::kReceiverTwist:
      return "receiver_twist";
    case RtkVelocitySource::kPositionDifference:
      return "position_difference";
  }
  return "unknown";
}

RtkVelocityGuard::RtkVelocityGuard(const Options &options)
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
          "Cannot open RTK velocity-guard CSV: " + path.string());
    csv_stream_
        << "keyframe_id,timestamp,regime,decision,velocity_source,"
           "baseline_available,"
           "active,state_changed,correction_applied,interval_sec,"
           "raw_rtk_vx,raw_rtk_vy,raw_rtk_vz,filtered_rtk_vx,"
           "filtered_rtk_vy,filtered_rtk_vz,lio_before_vx,lio_before_vy,"
           "lio_before_vz,lio_after_vx,lio_after_vy,lio_after_vz,"
           "velocity_error_mps,speed_ratio,applied_gain,correction_x,"
           "correction_y,correction_z,emergency_restart_required,"
           "emergency_restart_state_changed,emergency_restart_evidence,"
           "recovery_tracking\n";
    csv_stream_.flush();
  }
}

void RtkVelocityGuard::ValidateOptions(const Options &options)
{
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(options.low_pass_time_constant_sec) ||
      !positive(options.receiver_low_pass_time_constant_sec) ||
      !positive(options.maximum_receiver_position_difference_error_mps) ||
      !positive(options.activation_velocity_error_mps) ||
      !std::isfinite(options.activation_speed_ratio) ||
      options.activation_speed_ratio <= 1.0 ||
      options.activation_consecutive_observations <= 0 ||
      !positive(options.recovery_velocity_error_mps) ||
      options.recovery_velocity_error_mps >=
          options.activation_velocity_error_mps ||
      options.recovery_consecutive_observations <= 0 ||
      !positive(options.correction_time_constant_sec) ||
      !positive(options.maximum_planar_correction_mps) ||
      !positive(options.maximum_vertical_correction_mps) ||
      !positive(options.maximum_planar_correction_acceleration_mps2) ||
      !positive(options.maximum_vertical_correction_acceleration_mps2) ||
      !positive(options.healthy_vertical_activation_error_mps) ||
      options.healthy_vertical_activation_observations <= 0 ||
      !positive(options.healthy_vertical_time_constant_sec) ||
      !positive(options.healthy_vertical_maximum_acceleration_mps2) ||
      !positive(options.recovery_tracking_time_constant_sec) ||
      !positive(options.recovery_tracking_planar_acceleration_mps2) ||
      !positive(options.recovery_tracking_vertical_acceleration_mps2) ||
      !positive(options.emergency_restart_velocity_error_mps) ||
      options.emergency_restart_velocity_error_mps <=
          options.activation_velocity_error_mps ||
      options.emergency_restart_consecutive_observations <= 0)
    throw std::invalid_argument("RTK velocity-guard options are invalid.");
}

RtkVelocityGuard::Result RtkVelocityGuard::AddObservation(
    std::uint64_t keyframe_id, double timestamp,
    const Eigen::Vector3d &rtk_position,
    const Eigen::Vector3d &lio_velocity, CorrectionRegime regime,
    const std::optional<Eigen::Vector3d> &receiver_velocity)
{
  if (!std::isfinite(timestamp) || !rtk_position.allFinite() ||
      !lio_velocity.allFinite() ||
      (receiver_velocity && !receiver_velocity->allFinite()))
    throw std::invalid_argument("RTK velocity-guard input is invalid.");
  if (have_previous_ &&
      (keyframe_id <= previous_keyframe_id_ ||
       timestamp <= previous_timestamp_))
    throw std::logic_error(
        "RTK velocity-guard observations must be strictly ordered.");

  Result result;
  result.keyframe_id = keyframe_id;
  result.timestamp = timestamp;
  result.lio_velocity_before = lio_velocity;
  result.lio_velocity_after = lio_velocity;
  result.active = active_;
  result.recovery_tracking = recovery_tracking_;

  if (!have_previous_)
  {
    have_previous_ = true;
    previous_keyframe_id_ = keyframe_id;
    previous_timestamp_ = timestamp;
    previous_rtk_position_ = rtk_position;
    WriteCsv(timestamp, regime, result);
    return result;
  }

  result.baseline_available = true;
  result.interval_sec = timestamp - previous_timestamp_;
  const Eigen::Vector3d position_difference_velocity =
      (rtk_position - previous_rtk_position_) / result.interval_sec;
  const bool receiver_velocity_consistent = receiver_velocity &&
      (*receiver_velocity - position_difference_velocity).norm() <=
          options_.maximum_receiver_position_difference_error_mps;
  result.velocity_source = receiver_velocity_consistent
      ? RtkVelocitySource::kReceiverTwist
      : RtkVelocitySource::kPositionDifference;
  result.raw_rtk_velocity = receiver_velocity_consistent
      ? *receiver_velocity
      : position_difference_velocity;
  if (!have_filtered_velocity_)
  {
    filtered_rtk_velocity_ = result.raw_rtk_velocity;
    have_filtered_velocity_ = true;
  }
  else
  {
    const double time_constant = receiver_velocity_consistent
        ? options_.receiver_low_pass_time_constant_sec
        : options_.low_pass_time_constant_sec;
    const double alpha = 1.0 - std::exp(
        -result.interval_sec / time_constant);
    filtered_rtk_velocity_ +=
        alpha * (result.raw_rtk_velocity - filtered_rtk_velocity_);
  }
  result.filtered_rtk_velocity = filtered_rtk_velocity_;
  result.decision = RtkVelocityGuardDecision::kMonitoring;
  result.velocity_error_mps =
      (lio_velocity - filtered_rtk_velocity_).norm();
  const double lio_speed = lio_velocity.norm();
  const double rtk_speed = filtered_rtk_velocity_.norm();
  const double slower_speed = std::min(lio_speed, rtk_speed);
  const double faster_speed = std::max(lio_speed, rtk_speed);
  result.speed_ratio = faster_speed <= 0.2
      ? 1.0
      : faster_speed / std::max(0.2, slower_speed);

  if (options_.enabled)
  {
    const bool failure_regime =
        regime == CorrectionRegime::kDegraded ||
        regime == CorrectionRegime::kRelocalizationRequired;
    const bool evidence = failure_regime &&
        result.velocity_error_mps >=
            options_.activation_velocity_error_mps &&
        result.speed_ratio >= options_.activation_speed_ratio;
    if (!active_)
    {
      activation_evidence_ = evidence ? activation_evidence_ + 1 : 0;
      const bool severe_relocalization =
          regime == CorrectionRegime::kRelocalizationRequired &&
          result.velocity_error_mps >=
              2.0 * options_.activation_velocity_error_mps;
      if (severe_relocalization ||
          activation_evidence_ >=
              options_.activation_consecutive_observations)
      {
        active_ = true;
        result.state_changed = true;
        result.decision = RtkVelocityGuardDecision::kActivated;
        recovery_evidence_ = 0;
      }
    }

    // A persistent vertical velocity bias is the observed source of the
    // long-term altitude drift. Correct it at low rate while the global layer
    // is still elastic/degraded. This path requires receiver twist to agree
    // with independent RTK position differencing and cannot inject RTK
    // position or attitude noise into the frontend.
    const bool healthy_vertical_candidate =
        options_.healthy_vertical_aiding_enabled && !active_ &&
        regime != CorrectionRegime::kRelocalizationRequired &&
        result.velocity_source == RtkVelocitySource::kReceiverTwist &&
        std::abs(lio_velocity.z() - filtered_rtk_velocity_.z()) >=
            options_.healthy_vertical_activation_error_mps;
    healthy_vertical_evidence_ = healthy_vertical_candidate
        ? healthy_vertical_evidence_ + 1 : 0;
    if (healthy_vertical_evidence_ >=
        options_.healthy_vertical_activation_observations)
    {
      result.applied_gain = 1.0 - std::exp(
          -result.interval_sec /
          options_.healthy_vertical_time_constant_sec);
      const double requested = result.applied_gain *
          (filtered_rtk_velocity_.z() - lio_velocity.z());
      const double limit =
          options_.healthy_vertical_maximum_acceleration_mps2 *
          result.interval_sec;
      result.applied_correction.z() =
          std::clamp(requested, -limit, limit);
      result.lio_velocity_after.z() =
          lio_velocity.z() + result.applied_correction.z();
      result.correction_applied =
          std::abs(result.applied_correction.z()) > 1.0e-12;
      if (result.correction_applied)
        result.decision =
            RtkVelocityGuardDecision::kHealthyVerticalAiding;
    }

    if (active_)
    {
      // A few low-error samples are not proof that the rebuilt segment is
      // geometrically stable. During quarantined recovery only the 4DOF
      // window is allowed to release the stronger velocity tracker.
      if (!recovery_tracking_)
      {
        if (result.velocity_error_mps <=
            options_.recovery_velocity_error_mps)
          ++recovery_evidence_;
        else
          recovery_evidence_ = 0;
        if (recovery_evidence_ >=
            options_.recovery_consecutive_observations)
        {
          active_ = false;
          activation_evidence_ = 0;
          result.state_changed = true;
          result.decision = RtkVelocityGuardDecision::kRecovered;
        }
      }
      else
      {
        recovery_evidence_ = 0;
      }
      if (active_)
      {
        const double correction_time_constant = recovery_tracking_
            ? options_.recovery_tracking_time_constant_sec
            : options_.correction_time_constant_sec;
        result.applied_gain = 1.0 - std::exp(
            -result.interval_sec / correction_time_constant);
        Eigen::Vector3d correction = result.applied_gain *
            (filtered_rtk_velocity_ - lio_velocity);
        const double planar_acceleration = recovery_tracking_
            ? options_.recovery_tracking_planar_acceleration_mps2
            : options_.maximum_planar_correction_acceleration_mps2;
        const double vertical_acceleration = recovery_tracking_
            ? options_.recovery_tracking_vertical_acceleration_mps2
            : options_.maximum_vertical_correction_acceleration_mps2;
        const double planar_limit = std::min(
            options_.maximum_planar_correction_mps,
            planar_acceleration * result.interval_sec);
        const double vertical_limit = std::min(
            options_.maximum_vertical_correction_mps,
            vertical_acceleration * result.interval_sec);
        correction.head<2>() = LimitNorm(
            correction.head<2>(), planar_limit);
        correction.z() = std::clamp(
            correction.z(), -vertical_limit, vertical_limit);
        result.applied_correction = correction;
        result.lio_velocity_after = lio_velocity + correction;
        result.correction_applied = correction.norm() > 1.0e-12;
        if (!result.state_changed)
          result.decision = RtkVelocityGuardDecision::kCorrected;
      }
    }
  }

  const double post_velocity_error_mps =
      (result.lio_velocity_after - filtered_rtk_velocity_).norm();
  if (options_.enabled && options_.emergency_restart_enabled &&
      !emergency_restart_acknowledged_ &&
      !emergency_restart_required_)
  {
    const bool emergency_evidence =
        regime == CorrectionRegime::kRelocalizationRequired && active_ &&
        result.velocity_source == RtkVelocitySource::kReceiverTwist &&
        post_velocity_error_mps >=
            options_.emergency_restart_velocity_error_mps;
    emergency_restart_evidence_ = emergency_evidence
        ? emergency_restart_evidence_ + 1 : 0;
    if (emergency_restart_evidence_ >=
        options_.emergency_restart_consecutive_observations)
    {
      emergency_restart_required_ = true;
      emergency_restart_keyframe_id_ = keyframe_id;
      result.emergency_restart_state_changed = true;
    }
  }
  result.emergency_restart_required = emergency_restart_required_;
  result.emergency_restart_evidence = emergency_restart_evidence_;

  result.active = active_;
  result.recovery_tracking = recovery_tracking_;
  previous_keyframe_id_ = keyframe_id;
  previous_timestamp_ = timestamp;
  previous_rtk_position_ = rtk_position;
  WriteCsv(timestamp, regime, result);
  return result;
}

void RtkVelocityGuard::AcknowledgeEmergencyRestart(
    std::uint64_t keyframe_id)
{
  if (!emergency_restart_required_ ||
      keyframe_id != emergency_restart_keyframe_id_)
    throw std::logic_error(
        "RTK velocity emergency restart acknowledgement is invalid.");
  emergency_restart_required_ = false;
  emergency_restart_acknowledged_ = true;
  emergency_restart_evidence_ = 0;
  recovery_tracking_ = true;
}

void RtkVelocityGuard::CompleteRecoveryTracking(
    std::uint64_t keyframe_id)
{
  if (!recovery_tracking_ || !have_previous_ ||
      keyframe_id != previous_keyframe_id_)
    throw std::logic_error(
        "RTK recovery tracking completion is invalid.");
  recovery_tracking_ = false;
  active_ = false;
  activation_evidence_ = 0;
  recovery_evidence_ = 0;
  emergency_restart_acknowledged_ = false;
  emergency_restart_evidence_ = 0;
  emergency_restart_keyframe_id_ = 0;
}

void RtkVelocityGuard::WriteCsv(
    double timestamp, CorrectionRegime regime, const Result &result)
{
  if (!csv_stream_.is_open()) return;
  csv_stream_ << std::setprecision(17) << result.keyframe_id << ','
              << timestamp << ',' << CorrectionRegimeToString(regime) << ','
              << RtkVelocityGuardDecisionToString(result.decision) << ','
              << RtkVelocitySourceToString(result.velocity_source) << ','
              << static_cast<int>(result.baseline_available) << ','
              << static_cast<int>(result.active) << ','
              << static_cast<int>(result.state_changed) << ','
              << static_cast<int>(result.correction_applied) << ','
              << result.interval_sec << ',' << result.raw_rtk_velocity.x()
              << ',' << result.raw_rtk_velocity.y() << ','
              << result.raw_rtk_velocity.z() << ','
              << result.filtered_rtk_velocity.x() << ','
              << result.filtered_rtk_velocity.y() << ','
              << result.filtered_rtk_velocity.z() << ','
              << result.lio_velocity_before.x() << ','
              << result.lio_velocity_before.y() << ','
              << result.lio_velocity_before.z() << ','
              << result.lio_velocity_after.x() << ','
              << result.lio_velocity_after.y() << ','
              << result.lio_velocity_after.z() << ','
              << result.velocity_error_mps << ',' << result.speed_ratio << ','
              << result.applied_gain << ',' << result.applied_correction.x() << ','
              << result.applied_correction.y() << ','
              << result.applied_correction.z() << ','
              << static_cast<int>(result.emergency_restart_required) << ','
              << static_cast<int>(
                     result.emergency_restart_state_changed) << ','
              << result.emergency_restart_evidence << ','
              << static_cast<int>(result.recovery_tracking) << '\n';
  csv_stream_.flush();
}

}  // namespace my_livo::backend
