#include "backend/rigid_relocalization_monitor.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace my_livo::backend
{

const char *RigidRelocalizationDecisionToString(
    RigidRelocalizationDecision decision)
{
  switch (decision)
  {
    case RigidRelocalizationDecision::kMonitoring:
      return "monitoring";
    case RigidRelocalizationDecision::kWaitingVelocity:
      return "waiting_velocity";
    case RigidRelocalizationDecision::kCollectingBaseline:
      return "collecting_baseline";
    case RigidRelocalizationDecision::kScaleMismatch:
      return "scale_mismatch";
    case RigidRelocalizationDecision::kFitRejected:
      return "fit_rejected";
    case RigidRelocalizationDecision::kCandidate:
      return "candidate";
    case RigidRelocalizationDecision::kReady:
      return "ready";
    case RigidRelocalizationDecision::kRestartRequired:
      return "restart_required";
  }
  return "unknown";
}

const char *RelocalizationGeometryFailureToString(
    RelocalizationGeometryFailure failure)
{
  switch (failure)
  {
    case RelocalizationGeometryFailure::kNone:
      return "none";
    case RelocalizationGeometryFailure::kWeakGeometry:
      return "weak_geometry";
    case RelocalizationGeometryFailure::kSystematicScale:
      return "systematic_scale";
    case RelocalizationGeometryFailure::kSystematicScaleWeakGeometry:
      return "systematic_scale_weak_geometry";
    case RelocalizationGeometryFailure::kAttitudeMisalignment:
      return "attitude_misalignment";
    case RelocalizationGeometryFailure::kScaleAndAttitude:
      return "scale_and_attitude";
    case RelocalizationGeometryFailure::kNonRigid:
      return "nonrigid";
  }
  return "unknown";
}

RigidRelocalizationMonitor::RigidRelocalizationMonitor(
    const Options &options) : options_(options)
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
          "Cannot open rigid-relocalization CSV: " + path.string());
    csv_stream_
        << "keyframe_id,timestamp,regime,decision,guard_active,"
           "post_velocity_error_mps,window_size,window_start_keyframe_id,"
           "local_path_length_m,"
           "rtk_path_length_m,path_scale_ratio,planar_rms_m,"
           "vertical_rms_m,transform_x,transform_y,transform_z,"
           "transform_yaw_deg,ready,state_changed,cumulative_distance_m,"
           "frontend_restart_required,restart_state_changed,"
           "consecutive_structural_rejections,"
           "structural_rejection_span_m,restart_reason,"
           "full_rotation_observable,geometry_condition_ratio,"
           "se3_nonyaw_rotation_deg,se3_rms_m,similarity_scale,"
           "similarity_rms_m,geometry_failure\n";
    csv_stream_.flush();
  }
}

void RigidRelocalizationMonitor::ValidateOptions(const Options &options)
{
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(options.maximum_post_velocity_error_mps) ||
      options.minimum_observations < 3 ||
      !positive(options.minimum_path_length_m) ||
      !positive(options.maximum_window_length_m) ||
      options.maximum_window_length_m < options.minimum_path_length_m ||
      !positive(options.maximum_path_scale_error) ||
      options.maximum_path_scale_error >= 1.0 ||
      !positive(options.maximum_planar_rms_m) ||
      !positive(options.maximum_vertical_rms_m) ||
      options.required_consecutive_candidates <= 0 ||
      options.restart_after_consecutive_structural_rejections <= 0 ||
      !positive(options.restart_minimum_rejection_span_m) ||
      !positive(options.diagnostic_minimum_condition_ratio) ||
      options.diagnostic_minimum_condition_ratio >= 1.0 ||
      !positive(options.diagnostic_minimum_scale_error) ||
      options.diagnostic_minimum_scale_error >= 1.0 ||
      !positive(options.diagnostic_minimum_nonyaw_rotation_deg) ||
      !positive(options.diagnostic_maximum_similarity_rms_m))
    throw std::invalid_argument(
        "Rigid-relocalization monitor options are invalid.");
}

void RigidRelocalizationMonitor::ClearWindow()
{
  window_.clear();
  consecutive_candidates_ = 0;
  if (!frontend_restart_required_)
  {
    consecutive_structural_rejections_ = 0;
    first_structural_rejection_distance_m_ = 0.0;
    restart_reason_ = RigidRelocalizationDecision::kMonitoring;
  }
}

void RigidRelocalizationMonitor::Fit(
    std::size_t begin_index, Result *result) const
{
  Eigen::Vector2d local_centroid = Eigen::Vector2d::Zero();
  Eigen::Vector2d rtk_centroid = Eigen::Vector2d::Zero();
  double local_z_mean = 0.0;
  double rtk_z_mean = 0.0;
  for (std::size_t index = begin_index; index < window_.size(); ++index)
  {
    const auto &sample = window_[index];
    local_centroid += sample.local_position.head<2>();
    rtk_centroid += sample.rtk_position.head<2>();
    local_z_mean += sample.local_position.z();
    rtk_z_mean += sample.rtk_position.z();
  }
  const double count = static_cast<double>(window_.size() - begin_index);
  local_centroid /= count;
  rtk_centroid /= count;
  local_z_mean /= count;
  rtk_z_mean /= count;

  double dot_sum = 0.0;
  double cross_sum = 0.0;
  for (std::size_t index = begin_index; index < window_.size(); ++index)
  {
    const auto &sample = window_[index];
    const Eigen::Vector2d local =
        sample.local_position.head<2>() - local_centroid;
    const Eigen::Vector2d rtk =
        sample.rtk_position.head<2>() - rtk_centroid;
    dot_sum += local.dot(rtk);
    cross_sum += local.x() * rtk.y() - local.y() * rtk.x();
  }
  result->yaw_rad = std::atan2(cross_sum, dot_sum);
  const Eigen::Matrix2d rotation = Eigen::Rotation2Dd(
      result->yaw_rad).toRotationMatrix();
  result->translation.head<2>() =
      rtk_centroid - rotation * local_centroid;
  result->translation.z() = rtk_z_mean - local_z_mean;

  double planar_squared_sum = 0.0;
  double vertical_squared_sum = 0.0;
  for (std::size_t index = begin_index; index < window_.size(); ++index)
  {
    const auto &sample = window_[index];
    const Eigen::Vector2d planar_residual =
        rotation * sample.local_position.head<2>() +
        result->translation.head<2>() - sample.rtk_position.head<2>();
    const double vertical_residual = sample.local_position.z() +
        result->translation.z() - sample.rtk_position.z();
    planar_squared_sum += planar_residual.squaredNorm();
    vertical_squared_sum += vertical_residual * vertical_residual;
  }
  result->planar_rms_m = std::sqrt(planar_squared_sum / count);
  result->vertical_rms_m = std::sqrt(vertical_squared_sum / count);

  Eigen::Vector3d local_centroid_3d = Eigen::Vector3d::Zero();
  Eigen::Vector3d rtk_centroid_3d = Eigen::Vector3d::Zero();
  for (std::size_t index = begin_index; index < window_.size(); ++index)
  {
    const auto &sample = window_[index];
    local_centroid_3d += sample.local_position;
    rtk_centroid_3d += sample.rtk_position;
  }
  local_centroid_3d /= count;
  rtk_centroid_3d /= count;
  Eigen::Matrix3d cross_covariance = Eigen::Matrix3d::Zero();
  double local_squared_sum = 0.0;
  for (std::size_t index = begin_index; index < window_.size(); ++index)
  {
    const auto &sample = window_[index];
    const Eigen::Vector3d local =
        sample.local_position - local_centroid_3d;
    const Eigen::Vector3d rtk = sample.rtk_position - rtk_centroid_3d;
    cross_covariance += local * rtk.transpose();
    local_squared_sum += local.squaredNorm();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      cross_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0)
    reflection(2, 2) = -1.0;
  const Eigen::Matrix3d rotation_3d =
      svd.matrixV() * reflection * svd.matrixU().transpose();
  const double singular_sum =
      (svd.singularValues().array() * reflection.diagonal().array()).sum();
  result->similarity_scale = local_squared_sum > 1.0e-12
      ? singular_sum / local_squared_sum
      : 1.0;
  result->geometry_condition_ratio = svd.singularValues().x() > 1.0e-12
      ? svd.singularValues().y() / svd.singularValues().x()
      : 0.0;
  result->full_rotation_observable =
      result->geometry_condition_ratio >=
      options_.diagnostic_minimum_condition_ratio;
  const Eigen::Matrix3d yaw_rotation =
      Eigen::AngleAxisd(result->yaw_rad, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  result->se3_nonyaw_rotation_deg = Eigen::Quaterniond(
      yaw_rotation.transpose() * rotation_3d).angularDistance(
          Eigen::Quaterniond::Identity()) * 180.0 / std::acos(-1.0);
  const Eigen::Vector3d se3_translation =
      rtk_centroid_3d - rotation_3d * local_centroid_3d;
  const Eigen::Vector3d sim3_translation = rtk_centroid_3d -
      result->similarity_scale * rotation_3d * local_centroid_3d;
  double se3_squared_sum = 0.0;
  double sim3_squared_sum = 0.0;
  for (const auto &sample : window_)
  {
    se3_squared_sum += (rotation_3d * sample.local_position +
        se3_translation - sample.rtk_position).squaredNorm();
    sim3_squared_sum += (result->similarity_scale * rotation_3d *
        sample.local_position + sim3_translation -
        sample.rtk_position).squaredNorm();
  }
  result->se3_rms_m = std::sqrt(se3_squared_sum / count);
  result->similarity_rms_m = std::sqrt(sim3_squared_sum / count);
}

RigidRelocalizationMonitor::Result
RigidRelocalizationMonitor::AddObservation(
    std::uint64_t keyframe_id, double timestamp, double distance_m,
    const Eigen::Vector3d &local_position,
    const Eigen::Vector3d &rtk_position, CorrectionRegime regime,
    bool velocity_baseline_available, bool guard_active,
    double post_velocity_error_mps)
{
  if (!std::isfinite(timestamp) || !std::isfinite(distance_m) ||
      distance_m < 0.0 || !local_position.allFinite() ||
      !rtk_position.allFinite() ||
      (velocity_baseline_available &&
       !std::isfinite(post_velocity_error_mps)))
    throw std::invalid_argument(
        "Rigid-relocalization monitor input is invalid.");
  if (have_previous_input_ &&
      (keyframe_id <= previous_keyframe_id_ ||
       timestamp <= previous_timestamp_))
    throw std::logic_error(
        "Rigid-relocalization observations must be strictly ordered.");
  have_previous_input_ = true;
  previous_keyframe_id_ = keyframe_id;
  previous_timestamp_ = timestamp;

  Result result;
  result.keyframe_id = keyframe_id;
  result.guard_active = guard_active;
  result.post_velocity_error_mps = velocity_baseline_available
      ? post_velocity_error_mps
      : 0.0;
  result.ready = ready_;
  result.cumulative_distance_m = distance_m;
  const auto sync_restart_state = [&]() {
    result.frontend_restart_required = frontend_restart_required_;
    result.restart_reason = restart_reason_;
    result.consecutive_structural_rejections =
        consecutive_structural_rejections_;
    result.structural_rejection_span_m =
        consecutive_structural_rejections_ > 0
        ? distance_m - first_structural_rejection_distance_m_
        : 0.0;
  };
  sync_restart_state();

  if (!options_.enabled ||
      regime != CorrectionRegime::kRelocalizationRequired)
  {
    ClearWindow();
    sync_restart_state();
    WriteCsv(timestamp, regime, result);
    return result;
  }

  if (!velocity_baseline_available ||
      post_velocity_error_mps >
          options_.maximum_post_velocity_error_mps)
  {
    ClearWindow();
    sync_restart_state();
    result.decision = frontend_restart_required_
        ? RigidRelocalizationDecision::kRestartRequired
        : RigidRelocalizationDecision::kWaitingVelocity;
    WriteCsv(timestamp, regime, result);
    return result;
  }

  window_.push_back({keyframe_id, distance_m, local_position, rtk_position});
  while (window_.size() > 1 &&
         window_.back().distance_m - window_.front().distance_m >
             options_.maximum_window_length_m)
    window_.pop_front();

  result.window_size = window_.size();
  if (!window_.empty())
    result.window_start_keyframe_id = window_.front().keyframe_id;
  for (std::size_t index = 1; index < window_.size(); ++index)
  {
    result.local_path_length_m +=
        (window_[index].local_position -
         window_[index - 1].local_position).norm();
    result.rtk_path_length_m +=
        (window_[index].rtk_position -
         window_[index - 1].rtk_position).norm();
  }
  if (result.local_path_length_m > 1.0e-12)
    result.path_scale_ratio =
        result.rtk_path_length_m / result.local_path_length_m;

  if (window_.size() <
          static_cast<std::size_t>(options_.minimum_observations) ||
      std::min(result.local_path_length_m,
               result.rtk_path_length_m) <
          options_.minimum_path_length_m)
  {
    consecutive_candidates_ = 0;
    if (!frontend_restart_required_)
    {
      consecutive_structural_rejections_ = 0;
      first_structural_rejection_distance_m_ = 0.0;
      restart_reason_ = RigidRelocalizationDecision::kMonitoring;
    }
    sync_restart_state();
    result.decision = frontend_restart_required_
        ? RigidRelocalizationDecision::kRestartRequired
        : RigidRelocalizationDecision::kCollectingBaseline;
    WriteCsv(timestamp, regime, result);
    return result;
  }

  Fit(0, &result);
  if (options_.select_best_valid_suffix)
  {
    double best_score = std::max(
        result.planar_rms_m / options_.maximum_planar_rms_m,
        result.vertical_rms_m / options_.maximum_vertical_rms_m);
    for (std::size_t begin = 1;
         begin + static_cast<std::size_t>(options_.minimum_observations) <=
             window_.size(); ++begin)
    {
      Result candidate = result;
      candidate.window_size = window_.size() - begin;
      candidate.window_start_keyframe_id = window_[begin].keyframe_id;
      candidate.local_path_length_m = 0.0;
      candidate.rtk_path_length_m = 0.0;
      for (std::size_t index = begin + 1; index < window_.size(); ++index)
      {
        candidate.local_path_length_m +=
            (window_[index].local_position -
             window_[index - 1].local_position).norm();
        candidate.rtk_path_length_m +=
            (window_[index].rtk_position -
             window_[index - 1].rtk_position).norm();
      }
      if (std::min(candidate.local_path_length_m,
                   candidate.rtk_path_length_m) <
          options_.minimum_path_length_m)
        continue;
      candidate.path_scale_ratio = candidate.local_path_length_m > 1.0e-12
          ? candidate.rtk_path_length_m / candidate.local_path_length_m
          : 1.0;
      if (std::abs(candidate.path_scale_ratio - 1.0) >
          options_.maximum_path_scale_error)
        continue;
      Fit(begin, &candidate);
      const double score = std::max(
          candidate.planar_rms_m / options_.maximum_planar_rms_m,
          candidate.vertical_rms_m / options_.maximum_vertical_rms_m);
      if (score < best_score)
      {
        best_score = score;
        result = candidate;
      }
    }
  }
  RigidRelocalizationDecision structural_failure =
      RigidRelocalizationDecision::kMonitoring;
  if (std::abs(result.path_scale_ratio - 1.0) >
      options_.maximum_path_scale_error)
  {
    consecutive_candidates_ = 0;
    structural_failure = RigidRelocalizationDecision::kScaleMismatch;
  }
  else if (result.planar_rms_m > options_.maximum_planar_rms_m ||
           result.vertical_rms_m > options_.maximum_vertical_rms_m)
  {
    consecutive_candidates_ = 0;
    structural_failure = RigidRelocalizationDecision::kFitRejected;
  }

  if (structural_failure != RigidRelocalizationDecision::kMonitoring)
  {
    const bool similarity_explains_window =
        result.similarity_rms_m <=
        options_.diagnostic_maximum_similarity_rms_m;
    const bool has_scale_error =
        std::abs(result.similarity_scale - 1.0) >=
        options_.diagnostic_minimum_scale_error;
    const bool has_nonyaw_rotation = result.full_rotation_observable &&
        result.se3_nonyaw_rotation_deg >=
            options_.diagnostic_minimum_nonyaw_rotation_deg;
    if (!similarity_explains_window)
      result.geometry_failure = RelocalizationGeometryFailure::kNonRigid;
    else if (has_scale_error && !result.full_rotation_observable)
      result.geometry_failure =
          RelocalizationGeometryFailure::kSystematicScaleWeakGeometry;
    else if (has_scale_error && has_nonyaw_rotation)
      result.geometry_failure =
          RelocalizationGeometryFailure::kScaleAndAttitude;
    else if (has_scale_error)
      result.geometry_failure =
          RelocalizationGeometryFailure::kSystematicScale;
    else if (!result.full_rotation_observable)
      result.geometry_failure =
          RelocalizationGeometryFailure::kWeakGeometry;
    else if (has_nonyaw_rotation)
      result.geometry_failure =
          RelocalizationGeometryFailure::kAttitudeMisalignment;
    else
      result.geometry_failure = RelocalizationGeometryFailure::kNonRigid;
  }

  if (ready_)
  {
    result.decision = RigidRelocalizationDecision::kReady;
  }
  else if (structural_failure != RigidRelocalizationDecision::kMonitoring)
  {
    if (consecutive_structural_rejections_ == 0)
      first_structural_rejection_distance_m_ = distance_m;
    ++consecutive_structural_rejections_;
    restart_reason_ = structural_failure;
    result.structural_rejection_span_m =
        distance_m - first_structural_rejection_distance_m_;
    if (!frontend_restart_required_ &&
        consecutive_structural_rejections_ >=
            options_.restart_after_consecutive_structural_rejections &&
        result.structural_rejection_span_m >=
            options_.restart_minimum_rejection_span_m)
    {
      frontend_restart_required_ = true;
      result.restart_state_changed = true;
    }
    result.decision = frontend_restart_required_
        ? RigidRelocalizationDecision::kRestartRequired
        : structural_failure;
  }
  else if (frontend_restart_required_)
  {
    result.decision = RigidRelocalizationDecision::kRestartRequired;
  }
  else
  {
    consecutive_structural_rejections_ = 0;
    first_structural_rejection_distance_m_ = 0.0;
    restart_reason_ = RigidRelocalizationDecision::kMonitoring;
    ++consecutive_candidates_;
    result.decision = RigidRelocalizationDecision::kCandidate;
    if (!ready_ && consecutive_candidates_ >=
                       options_.required_consecutive_candidates)
    {
      ready_ = true;
      result.state_changed = true;
    }
    if (ready_) result.decision = RigidRelocalizationDecision::kReady;
  }
  result.ready = ready_;
  sync_restart_state();
  WriteCsv(timestamp, regime, result);
  return result;
}

void RigidRelocalizationMonitor::AcknowledgeFrontendRestart(
    std::uint64_t keyframe_id)
{
  if (!frontend_restart_required_)
    throw std::logic_error(
        "Cannot acknowledge a frontend restart that was not requested.");
  if (!have_previous_input_ || keyframe_id != previous_keyframe_id_)
    throw std::logic_error(
        "Frontend restart must acknowledge the triggering observation.");
  frontend_restart_required_ = false;
  restart_reason_ = RigidRelocalizationDecision::kMonitoring;
  consecutive_structural_rejections_ = 0;
  first_structural_rejection_distance_m_ = 0.0;
  ready_ = false;
  ClearWindow();
}

void RigidRelocalizationMonitor::WriteCsv(
    double timestamp, CorrectionRegime regime, const Result &result)
{
  if (!csv_stream_.is_open()) return;
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  csv_stream_ << std::setprecision(17) << result.keyframe_id << ','
              << timestamp << ',' << CorrectionRegimeToString(regime) << ','
              << RigidRelocalizationDecisionToString(result.decision) << ','
              << static_cast<int>(result.guard_active) << ','
              << result.post_velocity_error_mps << ',' << result.window_size
              << ',' << result.window_start_keyframe_id << ','
              << result.local_path_length_m << ','
              << result.rtk_path_length_m << ',' << result.path_scale_ratio
              << ',' << result.planar_rms_m << ',' << result.vertical_rms_m
              << ',' << result.translation.x() << ','
              << result.translation.y() << ',' << result.translation.z()
              << ',' << result.yaw_rad * kRadiansToDegrees << ','
              << static_cast<int>(result.ready) << ','
              << static_cast<int>(result.state_changed) << ','
              << result.cumulative_distance_m << ','
              << static_cast<int>(result.frontend_restart_required) << ','
              << static_cast<int>(result.restart_state_changed) << ','
              << result.consecutive_structural_rejections << ','
              << result.structural_rejection_span_m << ','
              << RigidRelocalizationDecisionToString(result.restart_reason)
              << ',' << static_cast<int>(result.full_rotation_observable)
              << ',' << result.geometry_condition_ratio << ','
              << result.se3_nonyaw_rotation_deg << ',' << result.se3_rms_m
              << ',' << result.similarity_scale << ','
              << result.similarity_rms_m << ','
              << RelocalizationGeometryFailureToString(
                     result.geometry_failure)
              << '\n';
  csv_stream_.flush();
}

}  // namespace my_livo::backend
