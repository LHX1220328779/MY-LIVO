#include "backend/global_pose_layer.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace my_livo::backend
{
namespace
{
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

void EnsureParentDirectory(const std::string &path)
{
  const std::filesystem::path file_path(path);
  if (file_path.has_parent_path())
    std::filesystem::create_directories(file_path.parent_path());
}

void WriteTrajectoryHeader(std::ostream &stream)
{
  stream << "id,timestamp,local_tx,local_ty,local_tz,local_qx,local_qy,"
            "local_qz,local_qw,global_tx,global_ty,global_tz,global_qx,"
            "global_qy,global_qz,global_qw,correction_translation_m,"
            "correction_angle_deg,map_eligible\n";
}

void WriteTrajectoryRow(std::ostream &stream, const Keyframe &keyframe,
                        const Pose3d &local_pose,
                        const Pose3d &global_pose, bool map_eligible)
{
  const Eigen::Quaterniond correction_rotation =
      global_pose.rotation * local_pose.rotation.conjugate();
  stream << std::setprecision(17) << keyframe.id() << ','
         << keyframe.timestamp() << ',' << local_pose.translation.x() << ','
         << local_pose.translation.y() << ',' << local_pose.translation.z()
         << ',' << local_pose.rotation.x() << ',' << local_pose.rotation.y()
         << ',' << local_pose.rotation.z() << ',' << local_pose.rotation.w()
         << ',' << global_pose.translation.x() << ','
         << global_pose.translation.y() << ','
         << global_pose.translation.z() << ',' << global_pose.rotation.x()
         << ',' << global_pose.rotation.y() << ',' << global_pose.rotation.z()
         << ',' << global_pose.rotation.w() << ','
         << (global_pose.translation - local_pose.translation).norm() << ','
         << correction_rotation.angularDistance(Eigen::Quaterniond::Identity())
                * 180.0 / std::acos(-1.0)
         << ',' << static_cast<int>(map_eligible) << '\n';
}
}  // namespace

GlobalPoseLayer::GlobalPoseLayer(const Options &options) : options_(options)
{
  if (!options_.T_global_slam.isFinite())
    throw std::invalid_argument("Global-pose A0 must be finite.");
  InitializeTrajectoryCsv();
  if (!options_.observation_csv_path.empty())
  {
    EnsureParentDirectory(options_.observation_csv_path);
    observation_stream_.open(options_.observation_csv_path,
                             std::ios::out | std::ios::trunc);
    if (!observation_stream_.is_open())
      throw std::runtime_error("Cannot open global RTK observation CSV: " +
                               options_.observation_csv_path);
    observation_stream_
        << "keyframe_id,timestamp,health,x,y,z,sigma_x,sigma_y,sigma_z\n";
  }
  correction_monitor_ =
      std::make_unique<CorrectionFeasibilityMonitor>(
          options_.correction_monitor);
  correction_field_ = std::make_unique<LowFrequencyCorrectionField>(
      options_.correction_field);
  regularized_field_ = std::make_unique<RegularizedCorrectionField4d>(
      options_.regularized_field);
  acceptance_monitor_ =
      std::make_unique<ElasticSegmentAcceptanceMonitor>(
          options_.elastic_acceptance);
  relocalization_monitor_ =
      std::make_unique<RigidRelocalizationMonitor>(
          options_.relocalization_monitor);
  recovery_monitor_ =
      std::make_unique<RigidRelocalizationMonitor>(
          options_.recovery_monitor);
  if (!options_.regularized_field_csv_path.empty())
  {
    EnsureParentDirectory(options_.regularized_field_csv_path);
    regularized_field_stream_.open(
        options_.regularized_field_csv_path, std::ios::out | std::ios::trunc);
    if (!regularized_field_stream_.is_open())
      throw std::runtime_error(
          "Cannot open regularized correction-field CSV: " +
          options_.regularized_field_csv_path);
    regularized_field_stream_
        << "keyframe_id,timestamp,segment_id,recovery_segment,decision,"
           "distance_m,knot_count,elastic_distance_m,elastic_stiffness,"
           "radial_force_proxy_m,"
           "correction_x,correction_y,correction_z,correction_yaw_deg,"
           "gradient_m_per_m,yaw_gradient_deg_per_m,curvature_per_m,"
           "residual_after_m,elastic_probation,position_observation\n";
    regularized_field_stream_.flush();
  }
  if (!options_.elastic_acceptance_csv_path.empty())
  {
    EnsureParentDirectory(options_.elastic_acceptance_csv_path);
    elastic_acceptance_stream_.open(
        options_.elastic_acceptance_csv_path,
        std::ios::out | std::ios::trunc);
    if (!elastic_acceptance_stream_.is_open())
      throw std::runtime_error(
          "Cannot open elastic-segment acceptance CSV: " +
          options_.elastic_acceptance_csv_path);
    elastic_acceptance_stream_
        << "keyframe_id,timestamp,decision,window_start_keyframe_id,"
           "window_size,valid_samples,valid_ratio,window_path_m,"
           "position_residual_m,yaw_residual_deg,"
           "post_velocity_error_mps,field_gradient_m_per_m,"
           "yaw_gradient_deg_per_m,accepted,state_changed,"
           "trusted_start_keyframe_id\n";
    elastic_acceptance_stream_.flush();
  }
}

Pose3d GlobalPoseLayer::BaseGlobal(const Pose3d &T_slam_body) const
{
  return options_.T_global_slam * T_slam_body;
}

Pose3d GlobalPoseLayer::SegmentNominal(
    std::size_t index, const Pose3d &T_slam_body) const
{
  const Pose3d base_pose = BaseGlobal(T_slam_body);
  const EmergencySegmentAnchor *active_anchor = nullptr;
  std::size_t active_anchor_index = 0;
  for (std::size_t anchor_index = 0;
       anchor_index < emergency_segment_anchors_.size(); ++anchor_index)
  {
    const auto &anchor = emergency_segment_anchors_[anchor_index];
    if (anchor.start_keyframe_id > index) break;
    active_anchor = &anchor;
    active_anchor_index = anchor_index;
  }
  if (!active_anchor)
    return base_pose;

  const auto provisional_pose = [this, &base_pose](
      const EmergencySegmentAnchor &anchor) {
    const std::size_t trigger_index =
        static_cast<std::size_t>(anchor.trigger_keyframe_id);
    const Pose3d trigger_base = BaseGlobal(
        local_slam_poses_.at(trigger_index));
    // The trigger belongs to the preceding segment, so this recursion cannot
    // select the provisional anchor whose start is trigger+1.
    const Pose3d trigger_global = ToGlobal(
        trigger_index, local_slam_poses_.at(trigger_index));
    const Eigen::Matrix3d carried_correction =
        (trigger_global.rotation * trigger_base.rotation.conjugate())
            .toRotationMatrix();
    const double carried_yaw = std::atan2(
        carried_correction(1, 0), carried_correction(0, 0));
    const Eigen::Quaterniond yaw_rotation(
        Eigen::AngleAxisd(carried_yaw, Eigen::Vector3d::UnitZ()));
    return Pose3d(
        yaw_rotation * base_pose.rotation,
        anchor.rtk_anchor_position + yaw_rotation *
            (base_pose.translation - trigger_base.translation));
  };
  if (!active_anchor->fitted)
    return provisional_pose(*active_anchor);

  const Eigen::Quaterniond position_yaw_rotation(
      Eigen::AngleAxisd(active_anchor->position_yaw_rad,
                        Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond orientation_yaw_rotation(
      Eigen::AngleAxisd(active_anchor->orientation_yaw_rad,
                        Eigen::Vector3d::UnitZ()));
  const Pose3d fitted_pose(
      orientation_yaw_rotation * base_pose.rotation,
      position_yaw_rotation * base_pose.translation +
          active_anchor->translation);
  (void)active_anchor_index;
  return fitted_pose;
}

Pose3d GlobalPoseLayer::ToGlobal(
    std::size_t index, const Pose3d &T_slam_body) const
{
  const Pose3d nominal_pose = SegmentNominal(index, T_slam_body);
  const EmergencySegmentAnchor *active_anchor = nullptr;
  for (const auto &anchor : emergency_segment_anchors_)
  {
    if (anchor.start_keyframe_id > index) break;
    active_anchor = &anchor;
  }
  if (!active_anchor)
    return regularized_field_->Apply(
        cumulative_distance_m_.at(index), nominal_pose);
  if (active_anchor->fitted && active_anchor->elastic_field)
    return active_anchor->elastic_field->Apply(
        cumulative_distance_m_.at(index), nominal_pose);
  return nominal_pose;
}

void GlobalPoseLayer::InitializeTrajectoryCsv()
{
  if (options_.trajectory_csv_path.empty()) return;
  EnsureParentDirectory(options_.trajectory_csv_path);
  trajectory_stream_.open(options_.trajectory_csv_path,
                          std::ios::out | std::ios::trunc);
  if (!trajectory_stream_.is_open())
    throw std::runtime_error("Cannot open global trajectory CSV: " +
                             options_.trajectory_csv_path);
  WriteTrajectoryHeader(trajectory_stream_);
  trajectory_stream_.flush();
}

void GlobalPoseLayer::AppendTrajectoryRow(const Keyframe &keyframe,
                                          const Pose3d &local_pose,
                                          const Pose3d &global_pose)
{
  if (!trajectory_stream_.is_open()) return;
  WriteTrajectoryRow(trajectory_stream_, keyframe, local_pose, global_pose,
                     global_map_eligible_.at(keyframe.id()) != 0U);
  trajectory_stream_.flush();
}

void GlobalPoseLayer::AppendKeyframe(const Keyframe::Ptr &keyframe)
{
  if (!keyframe)
    throw std::invalid_argument("Cannot append a null global keyframe.");
  std::lock_guard<std::mutex> lock(mutex_);
  if (keyframe->id() != global_poses_.size())
    throw std::logic_error(
        "Global-pose keyframe IDs must be contiguous from zero.");
  const Pose3d local_pose = keyframe->T_slam_body();
  if (!local_pose.isFinite())
    throw std::invalid_argument("Global-pose input or output is not finite.");
  local_slam_poses_.push_back(local_pose);
  keyframes_.push_back(keyframe);
  const double cumulative_distance = cumulative_distance_m_.empty()
      ? 0.0
      : cumulative_distance_m_.back() +
            (local_pose.translation -
             local_slam_poses_[local_slam_poses_.size() - 2].translation)
                .norm();
  cumulative_distance_m_.push_back(cumulative_distance);
  global_map_eligible_.push_back(
      quarantine_start_keyframe_id_.has_value() &&
              !recovery_segment_accepted_
          ? 0U : 1U);
  const Pose3d global_pose = ToGlobal(keyframe->id(), local_pose);
  if (!global_pose.isFinite())
    throw std::invalid_argument("Global-pose output is not finite.");
  global_poses_.push_back(global_pose);
  keyframe->set_T_global_body(global_pose);
  statistics_.keyframes = global_poses_.size();
  AppendTrajectoryRow(*keyframe, local_pose, global_pose);
}

void GlobalPoseLayer::RewriteTrajectoryLocked(
    const std::vector<Keyframe::Ptr> &keyframes,
    const std::vector<Pose3d> &local_slam_poses)
{
  if (options_.trajectory_csv_path.empty()) return;
  if (trajectory_stream_.is_open()) trajectory_stream_.close();
  const std::string temporary_path = options_.trajectory_csv_path + ".tmp";
  {
    std::ofstream temporary_stream(temporary_path,
                                   std::ios::out | std::ios::trunc);
    if (!temporary_stream.is_open())
      throw std::runtime_error("Cannot open temporary global trajectory CSV.");
    WriteTrajectoryHeader(temporary_stream);
    for (std::size_t index = 0; index < keyframes.size(); ++index)
      WriteTrajectoryRow(temporary_stream, *keyframes[index],
                         local_slam_poses[index], global_poses_[index],
                         global_map_eligible_.at(index) != 0U);
  }
  std::filesystem::rename(temporary_path, options_.trajectory_csv_path);
  trajectory_stream_.open(options_.trajectory_csv_path,
                          std::ios::out | std::ios::app);
  if (!trajectory_stream_.is_open())
    throw std::runtime_error("Cannot reopen global trajectory CSV.");
}

void GlobalPoseLayer::SynchronizeLocalPoses(
    const std::vector<Keyframe::Ptr> &keyframes,
    const std::vector<Pose3d> &local_slam_poses)
{
  if (keyframes.empty() || keyframes.size() != local_slam_poses.size())
    throw std::invalid_argument(
        "Global synchronization requires equal non-empty snapshots.");
  std::lock_guard<std::mutex> lock(mutex_);
  if (keyframes.size() != global_poses_.size() ||
      keyframes.size() != keyframes_.size())
    throw std::logic_error(
        "Global synchronization size differs from appended keyframes.");
  bool local_poses_changed = false;
  for (std::size_t index = 0; index < keyframes.size(); ++index)
  {
    if (!keyframes[index] || keyframes[index]->id() != index ||
        !local_slam_poses[index].isFinite())
      throw std::invalid_argument(
          "Global synchronization contains an invalid keyframe.");
    local_poses_changed = local_poses_changed ||
        (local_slam_poses_[index].translation -
         local_slam_poses[index].translation).norm() > 1.0e-9 ||
        local_slam_poses_[index].rotation.angularDistance(
            local_slam_poses[index].rotation) > 1.0e-10;
    if (keyframes_[index] != keyframes[index])
      throw std::logic_error("Global keyframe identity changed.");
    local_slam_poses_[index] = local_slam_poses[index];
  }
  if (local_poses_changed)
  {
    RecomputeCumulativeDistanceLocked();
    RebuildGlobalCorrectionLocked();
  }
  RecomputeGlobalPosesLocked();
  ++statistics_.full_synchronizations;
  RewriteTrajectoryLocked(keyframes, local_slam_poses);
}

GlobalPoseLayer::RtkUpdate GlobalPoseLayer::AddRtkObservation(
    std::uint64_t keyframe_id, const RtkObservation &observation)
{
  if (!std::isfinite(observation.timestamp) ||
      !observation.position.allFinite() ||
      !observation.position_covariance.allFinite())
    throw std::invalid_argument("Global RTK observation is not finite.");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(
      observation.position_covariance);
  if (eigensolver.info() != Eigen::Success ||
      eigensolver.eigenvalues().minCoeff() <= 0.0)
    throw std::invalid_argument(
        "Global RTK observation covariance must be positive definite.");
  std::lock_guard<std::mutex> lock(mutex_);
  if (keyframe_id >= global_poses_.size())
    throw std::invalid_argument(
        "Global RTK observation references an unknown keyframe.");
  if (!selected_observations_.empty() &&
      keyframe_id <= selected_observations_.back().keyframe_id)
    throw std::logic_error(
        "Global RTK observation IDs must be strictly increasing.");
  selected_observations_.push_back({keyframe_id, observation});
  ++statistics_.rtk_observations;
  if (observation_stream_.is_open())
  {
    observation_stream_ << std::setprecision(17) << keyframe_id << ','
                        << observation.timestamp << ',' << observation.health
                        << ',' << observation.position.x() << ','
                        << observation.position.y() << ','
                        << observation.position.z() << ','
                        << std::sqrt(observation.position_covariance(0, 0))
                        << ','
                        << std::sqrt(observation.position_covariance(1, 1))
                        << ','
                        << std::sqrt(observation.position_covariance(2, 2))
                        << '\n';
    observation_stream_.flush();
  }
  RtkUpdate update;
  const Pose3d base_pose = BaseGlobal(
      local_slam_poses_.at(keyframe_id));
  update.feasibility = correction_monitor_->AddObservation(
      keyframe_id, observation.timestamp,
      cumulative_distance_m_.at(keyframe_id),
      base_pose.translation,
      observation.position);
  if (update.feasibility.regime ==
      CorrectionRegime::kRelocalizationRequired)
    update.quarantine_started = StartQuarantineLocked(keyframe_id);
  update.correction_field = correction_field_->AddObservation(
      keyframe_id, observation.timestamp,
      cumulative_distance_m_.at(keyframe_id), base_pose.translation,
      observation.position, observation.position_covariance,
      update.feasibility.regime);
  selected_observations_.back().regime = update.feasibility.regime;
  if (update.correction_field.field_changed)
  {
    ++statistics_.correction_field_updates;
    RecomputeGlobalPosesLocked();
  }
  if (update.correction_field.field_changed || update.quarantine_started)
  {
    RewriteTrajectoryLocked(keyframes_, local_slam_poses_);
  }
  return update;
}

bool GlobalPoseLayer::StartQuarantineLocked(std::uint64_t keyframe_id)
{
  if (keyframe_id >= global_map_eligible_.size())
    throw std::logic_error("Global-map quarantine references an unknown pose.");
  if (quarantine_start_keyframe_id_ &&
      *quarantine_start_keyframe_id_ <= keyframe_id)
    return false;
  quarantine_start_keyframe_id_ = keyframe_id;
  for (std::size_t index = keyframe_id;
       index < global_map_eligible_.size(); ++index)
    global_map_eligible_[index] = 0U;
  return true;
}

void GlobalPoseLayer::ResetQuarantineLocked()
{
  quarantine_start_keyframe_id_.reset();
  global_map_eligible_.assign(keyframes_.size(), 1U);
}

RigidRelocalizationMonitor::Result
GlobalPoseLayer::AddRelocalizationEvidence(
    std::uint64_t keyframe_id, bool velocity_baseline_available,
    bool guard_active, double post_velocity_error_mps)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (selected_observations_.empty() ||
      selected_observations_.back().keyframe_id != keyframe_id)
    throw std::logic_error(
        "Relocalization evidence must follow its selected RTK observation.");
  auto &selected = selected_observations_.back();
  if (selected.has_relocalization_evidence)
    throw std::logic_error(
        "Relocalization evidence was added twice for one observation.");
  selected.has_relocalization_evidence = true;
  selected.velocity_baseline_available = velocity_baseline_available;
  selected.guard_active = guard_active;
  selected.post_velocity_error_mps = post_velocity_error_mps;
  return relocalization_monitor_->AddObservation(
      keyframe_id, selected.observation.timestamp,
      cumulative_distance_m_.at(keyframe_id),
      BaseGlobal(local_slam_poses_.at(keyframe_id)).translation,
      selected.observation.position, selected.regime,
      velocity_baseline_available, guard_active,
      post_velocity_error_mps);
}

void GlobalPoseLayer::AcknowledgeFrontendRestart(
    std::uint64_t keyframe_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (selected_observations_.empty() ||
      selected_observations_.back().keyframe_id != keyframe_id ||
      !selected_observations_.back().has_relocalization_evidence)
    throw std::logic_error(
        "Frontend restart must follow its relocalization evidence.");
  auto &selected = selected_observations_.back();
  if (selected.frontend_restart_acknowledged)
    throw std::logic_error("Frontend restart was acknowledged twice.");
  relocalization_monitor_->AcknowledgeFrontendRestart(keyframe_id);
  selected.frontend_restart_acknowledged = true;
}

void GlobalPoseLayer::StartEmergencyGlobalSegment(
    std::uint64_t trigger_keyframe_id,
    const Eigen::Vector3d &rtk_anchor_position)
{
  if (!rtk_anchor_position.allFinite())
    throw std::invalid_argument(
        "Emergency global-segment RTK anchor is not finite.");
  std::lock_guard<std::mutex> lock(mutex_);
  if (trigger_keyframe_id >= global_poses_.size())
    throw std::invalid_argument(
        "Emergency global segment references an unknown keyframe.");
  if (!emergency_segment_anchors_.empty() &&
      trigger_keyframe_id <=
          emergency_segment_anchors_.back().trigger_keyframe_id)
    throw std::logic_error(
        "Emergency global segment boundaries must be strictly ordered.");
  // A provisional quarantined segment must be continuous with the trusted
  // predecessor. RTK is evidence for the later robust 4DOF fit, not an
  // instantaneous anchor that may teleport the path by many metres.
  emergency_segment_anchors_.push_back(EmergencySegmentAnchor{
      trigger_keyframe_id, trigger_keyframe_id + 1U,
      0U, 0U, global_poses_.at(trigger_keyframe_id).translation});
  recovery_observations_.clear();
  recovery_monitor_ =
      std::make_unique<RigidRelocalizationMonitor>(
          options_.recovery_monitor);
  recovery_monitor_active_ = true;
  recovery_elastic_probation_ = false;
  recovery_segment_accepted_ = false;
  acceptance_monitor_->Reset();
  ++statistics_.emergency_segments;
  // The failed tail and the new recovery segment stay out of the trusted map
  // until a fresh no-scale 4DOF gate accepts the segment.
  StartQuarantineLocked(trigger_keyframe_id);
  RecomputeGlobalPosesLocked();
  RewriteTrajectoryLocked(keyframes_, local_slam_poses_);
}

GlobalPoseLayer::RecoveryUpdate GlobalPoseLayer::AddRecoveryObservation(
    std::uint64_t keyframe_id, const RtkObservation &observation,
    bool velocity_baseline_available, bool guard_active,
    double post_velocity_error_mps)
{
  RecoveryUpdate update;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!recovery_monitor_active_) return update;
  update.elastic_probation = recovery_elastic_probation_;
  if (keyframe_id >= local_slam_poses_.size())
    throw std::invalid_argument(
        "Recovery observation references an unknown keyframe.");
  if (!recovery_observations_.empty() &&
      keyframe_id <= recovery_observations_.back().keyframe_id)
    throw std::logic_error(
        "Recovery observations must be strictly ordered.");
  recovery_observations_.push_back(RecoveryObservation{
      keyframe_id, observation, velocity_baseline_available,
      guard_active, post_velocity_error_mps});
  update.observation_used = true;
  const Pose3d base_pose = BaseGlobal(local_slam_poses_.at(keyframe_id));
  update.relocalization = recovery_monitor_->AddObservation(
      keyframe_id, observation.timestamp,
      cumulative_distance_m_.at(keyframe_id), base_pose.translation,
      observation.position, CorrectionRegime::kRelocalizationRequired,
      velocity_baseline_available, guard_active,
      post_velocity_error_mps);
  if (recovery_elastic_probation_)
  {
    update.elastic_probation = true;
    return update;
  }
  if (!update.relocalization.state_changed ||
      !update.relocalization.ready)
    return update;

  EmergencySegmentAnchor fitted_anchor;
  if (emergency_segment_anchors_.empty() ||
      emergency_segment_anchors_.back().fitted)
    throw std::logic_error(
        "Recovery fit lacks its provisional segment boundary.");
  // The provisional history is immutable.  A validated fit starts a new
  // rigid segment at the current keyframe; it must never morph the whole
  // quarantined baseline that was needed to estimate this transform.
  fitted_anchor.trigger_keyframe_id =
      emergency_segment_anchors_.back().trigger_keyframe_id;
  fitted_anchor.start_keyframe_id = update.relocalization.keyframe_id;
  fitted_anchor.trusted_start_keyframe_id =
      update.relocalization.keyframe_id;
  fitted_anchor.transition_end_keyframe_id =
      update.relocalization.keyframe_id;
  fitted_anchor.fitted = true;
  fitted_anchor.position_yaw_rad = update.relocalization.yaw_rad;
  fitted_anchor.translation = update.relocalization.translation;
  if (fitted_anchor.start_keyframe_id == 0 ||
      fitted_anchor.start_keyframe_id >= global_poses_.size() ||
      (!emergency_segment_anchors_.empty() &&
       fitted_anchor.start_keyframe_id <
           emergency_segment_anchors_.back().start_keyframe_id))
    throw std::logic_error(
        "Recovery fit produced an invalid segment boundary.");

  // A gravity-constrained 4DOF segment is one rigid transform: the robust
  // long-baseline position yaw rotates both translations and attitudes.
  // Receiver quaternion/yaw is deliberately not a production input.
  fitted_anchor.orientation_yaw_rad = update.relocalization.yaw_rad;
  update.orientation_yaw_rad = fitted_anchor.orientation_yaw_rad;
  auto recovery_field_options = options_.regularized_field;
  recovery_field_options.anchor_first_knot_identity = true;
  fitted_anchor.elastic_field =
      std::make_shared<RegularizedCorrectionField4d>(
          recovery_field_options);
  emergency_segment_anchors_.push_back(fitted_anchor);
  for (const auto &elastic : elastic_observations_)
  {
    if (elastic.keyframe_id < fitted_anchor.start_keyframe_id ||
        elastic.keyframe_id > update.relocalization.keyframe_id)
      continue;
    const Pose3d nominal = SegmentNominal(
        elastic.keyframe_id,
        local_slam_poses_.at(elastic.keyframe_id));
    emergency_segment_anchors_.back().elastic_field->AddObservation(
        elastic.keyframe_id, elastic.observation.timestamp,
        cumulative_distance_m_.at(elastic.keyframe_id), nominal,
        elastic.observation, elastic.use_position_observation);
  }
  // Rigid ready starts elastic probation; it is not proof that the frontend
  // remains stable without velocity support.
  recovery_elastic_probation_ = true;
  recovery_segment_accepted_ = false;
  acceptance_monitor_->Reset();
  update.elastic_probation = true;
  ++statistics_.recovery_reanchors;
  RecomputeGlobalPosesLocked();
  RewriteTrajectoryLocked(keyframes_, local_slam_poses_);
  update.segment_reanchored = true;
  return update;
}

GlobalPoseLayer::ElasticUpdate GlobalPoseLayer::AddElasticObservation(
    std::uint64_t keyframe_id, const RtkObservation &observation,
    bool use_position_observation)
{
  if (!std::isfinite(observation.timestamp) ||
      !observation.position.allFinite() ||
      !observation.orientation.coeffs().allFinite())
    throw std::invalid_argument(
        "Elastic observation is not finite.");
  std::lock_guard<std::mutex> lock(mutex_);
  if (keyframe_id >= local_slam_poses_.size())
    throw std::invalid_argument(
        "Elastic observation references an unknown keyframe.");
  if (!elastic_observations_.empty() &&
      keyframe_id <= elastic_observations_.back().keyframe_id)
    throw std::logic_error(
        "Elastic observations must be strictly ordered.");
  elastic_observations_.push_back(
      {keyframe_id, observation, use_position_observation});

  ElasticUpdate update;
  update.observation_used = true;
  EmergencySegmentAnchor *active_anchor = nullptr;
  std::uint64_t segment_id = 0;
  for (std::size_t index = 0; index < emergency_segment_anchors_.size();
       ++index)
  {
    const auto &anchor = emergency_segment_anchors_[index];
    if (!anchor.fitted) ++segment_id;
    if (anchor.start_keyframe_id > keyframe_id)
      break;
    active_anchor = &emergency_segment_anchors_[index];
  }
  RegularizedCorrectionField4d *field = nullptr;
  if (!active_anchor)
  {
    // Once the feasibility monitor quarantines the initial segment, the
    // production field freezes immediately.  Continuing to pull this failed
    // interval toward RTK would convert a frontend/local-shape failure into a
    // visibly warped global map.
    if (!quarantine_start_keyframe_id_ ||
        keyframe_id < *quarantine_start_keyframe_id_)
      field = regularized_field_.get();
  }
  else if (active_anchor->fitted && active_anchor->elastic_field)
  {
    field = active_anchor->elastic_field.get();
  }
  update.recovery_segment = active_anchor != nullptr;
  update.segment_id = segment_id;

  const Pose3d nominal = SegmentNominal(
      keyframe_id, local_slam_poses_.at(keyframe_id));
  const bool provisional_quarantine = field == nullptr;
  Pose3d corrected = nominal;
  if (!provisional_quarantine)
  {
    update.field = field->AddObservation(
        keyframe_id, observation.timestamp,
        cumulative_distance_m_.at(keyframe_id), nominal, observation,
        use_position_observation);
    update.field_changed = update.field.decision ==
            RegularizedCorrectionField4d::AddDecision::kAccepted ||
        update.field.decision ==
            RegularizedCorrectionField4d::AddDecision::kTranslationFallback;
    update.evaluation = field->EvaluateWithDerivatives(
        cumulative_distance_m_.at(keyframe_id));
    corrected = field->Apply(
        cumulative_distance_m_.at(keyframe_id), nominal);
  }
  update.residual_after_m =
      (corrected.translation - observation.position).norm();
  // Acceptance checks lag to the robust position-path yaw target, never the
  // CGI-610 attitude quaternion. Receiver attitude remains diagnostic-only.
  const double yaw_lag = update.field.fitted_correction.yaw_rad -
      update.evaluation.correction.yaw_rad;
  update.yaw_residual_after_deg = std::abs(
      std::atan2(std::sin(yaw_lag), std::cos(yaw_lag))) *
      kRadiansToDegrees;
  if (recovery_elastic_probation_ && use_position_observation &&
      active_anchor && active_anchor->fitted)
  {
    if (recovery_observations_.empty() ||
        recovery_observations_.back().keyframe_id != keyframe_id)
      throw std::logic_error(
          "Elastic acceptance lacks same-keyframe velocity evidence.");
    const auto &velocity = recovery_observations_.back();
    const bool alignment_valid = update.field.decision ==
            RegularizedCorrectionField4d::AddDecision::kAccepted ||
        update.field.decision ==
            RegularizedCorrectionField4d::AddDecision::kTranslationFallback;
    update.acceptance = acceptance_monitor_->AddObservation(
        keyframe_id, cumulative_distance_m_.at(keyframe_id),
        alignment_valid ? update.residual_after_m
                        : options_.elastic_acceptance
                                  .maximum_position_residual_m + 1.0,
        update.yaw_residual_after_deg,
        velocity.post_velocity_error_mps,
        velocity.velocity_baseline_available, velocity.guard_active,
        update.evaluation.first_derivative.displacement.norm(),
        std::abs(update.evaluation.first_derivative.yaw_rad) *
            kRadiansToDegrees);
    if (elastic_acceptance_stream_.is_open())
    {
      elastic_acceptance_stream_ << std::setprecision(17)
          << keyframe_id << ',' << observation.timestamp << ','
          << ElasticSegmentAcceptanceDecisionToString(
                 update.acceptance.decision)
          << ',' << update.acceptance.window_start_keyframe_id << ','
          << update.acceptance.window_size << ','
          << update.acceptance.valid_samples << ','
          << update.acceptance.valid_ratio << ','
          << update.acceptance.window_path_length_m << ','
          << update.residual_after_m << ','
          << update.yaw_residual_after_deg << ','
          << velocity.post_velocity_error_mps << ','
          << update.evaluation.first_derivative.displacement.norm() << ','
          << std::abs(update.evaluation.first_derivative.yaw_rad) *
                 kRadiansToDegrees
          << ',' << static_cast<int>(update.acceptance.accepted) << ','
          << static_cast<int>(update.acceptance.state_changed) << ','
          << active_anchor->trusted_start_keyframe_id << '\n';
      elastic_acceptance_stream_.flush();
    }
    if (update.acceptance.state_changed)
    {
      if (active_anchor->trusted_start_keyframe_id == 0 ||
          active_anchor->trusted_start_keyframe_id > keyframe_id)
        throw std::logic_error(
            "Elastic acceptance has an invalid trusted boundary.");
      update.segment_accepted = true;
      update.trusted_start_keyframe_id =
          active_anchor->trusted_start_keyframe_id;
      recovery_elastic_probation_ = false;
      recovery_segment_accepted_ = true;
      for (std::size_t index = active_anchor->trusted_start_keyframe_id;
           index < global_map_eligible_.size(); ++index)
        global_map_eligible_[index] = 1U;
      ++statistics_.accepted_recovery_segments;
    }
  }
  if (update.field_changed)
  {
    ++statistics_.regularized_field_updates;
    RecomputeGlobalPosesLocked();
  }
  if (update.field_changed || update.segment_accepted)
    RewriteTrajectoryLocked(keyframes_, local_slam_poses_);

  if (regularized_field_stream_.is_open())
  {
    const char *decision = provisional_quarantine
        ? (active_anchor ? "provisional_recovery" : "frozen_quarantine")
        : "accepted";
    if (!provisional_quarantine && update.field.decision ==
        RegularizedCorrectionField4d::AddDecision::kStatusRejected)
      decision = "status_rejected";
    else if (!provisional_quarantine && update.field.decision ==
        RegularizedCorrectionField4d::AddDecision::kSpacingRejected)
      decision = "spacing_rejected";
    else if (!provisional_quarantine && update.field.decision ==
        RegularizedCorrectionField4d::AddDecision::kNoPositionConstraint)
      decision = "position_hold";
    else if (!provisional_quarantine && update.field.decision ==
        RegularizedCorrectionField4d::AddDecision::kAlignmentRejected)
      decision = "alignment_rejected";
    else if (!provisional_quarantine && update.field.decision ==
        RegularizedCorrectionField4d::AddDecision::kTranslationFallback)
      decision = "translation_fallback";
    regularized_field_stream_ << std::setprecision(17)
        << keyframe_id << ',' << observation.timestamp << ','
        << segment_id << ',' << static_cast<int>(update.recovery_segment)
        << ',' << decision << ','
        << cumulative_distance_m_.at(keyframe_id) << ','
        << update.field.knot_count << ','
        << update.field.elastic_distance_m << ','
        << update.field.elastic_stiffness << ','
        << update.field.elastic_stiffness * update.field.elastic_distance_m
        << ','
        << update.evaluation.correction.displacement.x() << ','
        << update.evaluation.correction.displacement.y() << ','
        << update.evaluation.correction.displacement.z() << ','
        << update.evaluation.correction.yaw_rad * 180.0 / std::acos(-1.0)
        << ',' << update.evaluation.first_derivative.displacement.norm()
        << ',' << std::abs(update.evaluation.first_derivative.yaw_rad) *
            180.0 / std::acos(-1.0)
        << ',' << update.evaluation.second_derivative.displacement.norm()
        << ',' << update.residual_after_m << ','
        << static_cast<int>(recovery_elastic_probation_) << ','
        << static_cast<int>(use_position_observation) << '\n';
    regularized_field_stream_.flush();
  }
  return update;
}

void GlobalPoseLayer::RecomputeCumulativeDistanceLocked()
{
  cumulative_distance_m_.assign(local_slam_poses_.size(), 0.0);
  for (std::size_t index = 1; index < local_slam_poses_.size(); ++index)
    cumulative_distance_m_[index] = cumulative_distance_m_[index - 1] +
        (local_slam_poses_[index].translation -
         local_slam_poses_[index - 1].translation).norm();
}

void GlobalPoseLayer::RebuildGlobalCorrectionLocked()
{
  correction_monitor_ =
      std::make_unique<CorrectionFeasibilityMonitor>(
          options_.correction_monitor);
  correction_field_ = std::make_unique<LowFrequencyCorrectionField>(
      options_.correction_field);
  relocalization_monitor_ =
      std::make_unique<RigidRelocalizationMonitor>(
          options_.relocalization_monitor);
  recovery_monitor_ =
      std::make_unique<RigidRelocalizationMonitor>(
          options_.recovery_monitor);
  ResetQuarantineLocked();
  for (auto &selected : selected_observations_)
  {
    const Pose3d base_pose = BaseGlobal(
        local_slam_poses_.at(selected.keyframe_id));
    const auto feasibility = correction_monitor_->AddObservation(
        selected.keyframe_id, selected.observation.timestamp,
        cumulative_distance_m_.at(selected.keyframe_id),
        base_pose.translation,
        selected.observation.position);
    if (feasibility.regime ==
        CorrectionRegime::kRelocalizationRequired)
      StartQuarantineLocked(selected.keyframe_id);
    correction_field_->AddObservation(
        selected.keyframe_id, selected.observation.timestamp,
        cumulative_distance_m_.at(selected.keyframe_id),
        base_pose.translation, selected.observation.position,
        selected.observation.position_covariance, feasibility.regime);
    selected.regime = feasibility.regime;
    if (selected.has_relocalization_evidence)
    {
      relocalization_monitor_->AddObservation(
          selected.keyframe_id, selected.observation.timestamp,
          cumulative_distance_m_.at(selected.keyframe_id),
          base_pose.translation, selected.observation.position,
          feasibility.regime, selected.velocity_baseline_available,
          selected.guard_active, selected.post_velocity_error_mps);
      if (selected.frontend_restart_acknowledged)
        relocalization_monitor_->AcknowledgeFrontendRestart(
            selected.keyframe_id);
    }
  }
  emergency_segment_anchors_.erase(
      std::remove_if(
          emergency_segment_anchors_.begin(),
          emergency_segment_anchors_.end(),
          [](const EmergencySegmentAnchor &anchor) {
            return anchor.fitted;
          }),
      emergency_segment_anchors_.end());
  recovery_monitor_active_ = !emergency_segment_anchors_.empty();
  recovery_elastic_probation_ = false;
  for (const auto &recovery : recovery_observations_)
  {
    const Pose3d base_pose = BaseGlobal(
        local_slam_poses_.at(recovery.keyframe_id));
    const auto result = recovery_monitor_->AddObservation(
        recovery.keyframe_id, recovery.observation.timestamp,
        cumulative_distance_m_.at(recovery.keyframe_id),
        base_pose.translation, recovery.observation.position,
        CorrectionRegime::kRelocalizationRequired,
        recovery.velocity_baseline_available, recovery.guard_active,
        recovery.post_velocity_error_mps);
    if (!recovery_elastic_probation_ &&
        result.state_changed && result.ready)
    {
      EmergencySegmentAnchor fitted_anchor;
      if (emergency_segment_anchors_.empty() ||
          emergency_segment_anchors_.back().fitted)
        throw std::logic_error(
            "Recovery replay lacks its provisional segment boundary.");
      fitted_anchor.trigger_keyframe_id =
          emergency_segment_anchors_.back().trigger_keyframe_id;
      fitted_anchor.start_keyframe_id = result.keyframe_id;
      fitted_anchor.trusted_start_keyframe_id =
          result.keyframe_id;
      fitted_anchor.transition_end_keyframe_id = result.keyframe_id;
      fitted_anchor.fitted = true;
      fitted_anchor.position_yaw_rad = result.yaw_rad;
      fitted_anchor.translation = result.translation;
      fitted_anchor.orientation_yaw_rad = result.yaw_rad;
      emergency_segment_anchors_.push_back(fitted_anchor);
      recovery_elastic_probation_ = true;
    }
  }
  RebuildRegularizedFieldsLocked();
}

void GlobalPoseLayer::RebuildRegularizedFieldsLocked()
{
  regularized_field_ = std::make_unique<RegularizedCorrectionField4d>(
      options_.regularized_field);
  auto recovery_options = options_.regularized_field;
  recovery_options.anchor_first_knot_identity = true;
  for (auto &anchor : emergency_segment_anchors_)
  {
    anchor.elastic_field = anchor.fitted
        ? std::make_shared<RegularizedCorrectionField4d>(recovery_options)
        : nullptr;
  }

  for (const auto &elastic : elastic_observations_)
  {
    EmergencySegmentAnchor *active_anchor = nullptr;
    for (auto &anchor : emergency_segment_anchors_)
    {
      if (anchor.start_keyframe_id > elastic.keyframe_id) break;
      active_anchor = &anchor;
    }
    RegularizedCorrectionField4d *field = nullptr;
    if (!active_anchor)
    {
      if (!quarantine_start_keyframe_id_ ||
          elastic.keyframe_id < *quarantine_start_keyframe_id_)
        field = regularized_field_.get();
    }
    else if (active_anchor->fitted && active_anchor->elastic_field)
      field = active_anchor->elastic_field.get();
    if (!field) continue;
    const Pose3d nominal = SegmentNominal(
        elastic.keyframe_id,
        local_slam_poses_.at(elastic.keyframe_id));
    field->AddObservation(
        elastic.keyframe_id, elastic.observation.timestamp,
        cumulative_distance_m_.at(elastic.keyframe_id), nominal,
        elastic.observation, elastic.use_position_observation);
  }
}

void GlobalPoseLayer::RecomputeGlobalPosesLocked()
{
  if (local_slam_poses_.size() != cumulative_distance_m_.size() ||
      local_slam_poses_.size() != keyframes_.size())
    throw std::logic_error("Global-pose layer snapshots differ.");
  global_poses_.resize(local_slam_poses_.size());
  for (std::size_t index = 0; index < local_slam_poses_.size(); ++index)
  {
    global_poses_[index] = ToGlobal(index, local_slam_poses_[index]);
    keyframes_[index]->set_T_global_body(global_poses_[index]);
  }
}

std::vector<Pose3d> GlobalPoseLayer::global_poses() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return global_poses_;
}

GlobalPoseLayer::GlobalMapSnapshot
GlobalPoseLayer::global_map_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return GlobalMapSnapshot{global_poses_, global_map_eligible_};
}

Pose3d GlobalPoseLayer::latest_global_pose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (global_poses_.empty())
    throw std::logic_error("Global-pose layer is empty.");
  return global_poses_.back();
}

GlobalPoseLayer::Statistics GlobalPoseLayer::statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return statistics_;
}

}  // namespace my_livo::backend
