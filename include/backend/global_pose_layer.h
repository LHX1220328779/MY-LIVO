#ifndef MY_LIVO_BACKEND_GLOBAL_POSE_LAYER_H
#define MY_LIVO_BACKEND_GLOBAL_POSE_LAYER_H

#include "backend/correction_feasibility_monitor.h"
#include "backend/elastic_segment_acceptance_monitor.h"
#include "backend/keyframe.h"
#include "backend/low_frequency_correction_field.h"
#include "backend/regularized_correction_field_4d.h"
#include "backend/rigid_relocalization_monitor.h"
#include "backend/rtk_observation_buffer.h"

#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace my_livo::backend
{

// Owns georeferenced poses outside the odometry+loop graph. A fixed A0 and a
// bounded low-frequency C(s) never feed back into the frontend/local graph.
class GlobalPoseLayer
{
public:
  struct Options
  {
    Pose3d T_global_slam;
    CorrectionFeasibilityMonitor::Options correction_monitor;
    LowFrequencyCorrectionField::Options correction_field;
    RegularizedCorrectionField4d::Options regularized_field;
    ElasticSegmentAcceptanceMonitor::Options elastic_acceptance;
    RigidRelocalizationMonitor::Options relocalization_monitor;
    RigidRelocalizationMonitor::Options recovery_monitor;
    std::string trajectory_csv_path;
    std::string observation_csv_path;
    std::string regularized_field_csv_path;
    std::string elastic_acceptance_csv_path;
  };

  struct Statistics
  {
    std::uint64_t keyframes = 0;
    std::uint64_t full_synchronizations = 0;
    std::uint64_t rtk_observations = 0;
    std::uint64_t correction_field_updates = 0;
    std::uint64_t emergency_segments = 0;
    std::uint64_t recovery_reanchors = 0;
    std::uint64_t regularized_field_updates = 0;
    std::uint64_t accepted_recovery_segments = 0;
  };

  struct RtkUpdate
  {
    CorrectionFeasibilityMonitor::Result feasibility;
    LowFrequencyCorrectionField::UpdateResult correction_field;
    bool quarantine_started = false;
  };

  struct GlobalMapSnapshot
  {
    std::vector<Pose3d> poses;
    std::vector<std::uint8_t> eligible;
  };

  struct RecoveryUpdate
  {
    RigidRelocalizationMonitor::Result relocalization;
    bool observation_used = false;
    bool segment_reanchored = false;
    // A rigid fit only enters a quarantined probation state.  The caller must
    // keep the recovery velocity support active until the later elastic
    // segment acceptance explicitly releases it.
    bool elastic_probation = false;
    double orientation_yaw_rad = 0.0;
  };

  struct ElasticUpdate
  {
    RegularizedCorrectionField4d::UpdateResult field;
    bool observation_used = false;
    bool field_changed = false;
    bool recovery_segment = false;
    std::uint64_t segment_id = 0;
    double residual_after_m = 0.0;
    double yaw_residual_after_deg = 0.0;
    ElasticSegmentAcceptanceMonitor::Result acceptance;
    bool segment_accepted = false;
    std::uint64_t trusted_start_keyframe_id = 0;
    RegularizedCorrectionField4d::Evaluation evaluation;
  };

  explicit GlobalPoseLayer(const Options &options);

  void AppendKeyframe(const Keyframe::Ptr &keyframe);
  void SynchronizeLocalPoses(
      const std::vector<Keyframe::Ptr> &keyframes,
      const std::vector<Pose3d> &local_slam_poses);
  RtkUpdate AddRtkObservation(
      std::uint64_t keyframe_id, const RtkObservation &observation);
  RigidRelocalizationMonitor::Result AddRelocalizationEvidence(
      std::uint64_t keyframe_id, bool velocity_baseline_available,
      bool guard_active, double post_velocity_error_mps);
  void AcknowledgeFrontendRestart(std::uint64_t keyframe_id);
  // Starts a new path-preserving global segment after a controlled frontend
  // recovery.  The trigger keyframe stays in the old quarantined segment;
  // subsequent poses are attached to RTK by translation plus carried yaw.
  void StartEmergencyGlobalSegment(
      std::uint64_t trigger_keyframe_id,
      const Eigen::Vector3d &rtk_anchor_position);
  RecoveryUpdate AddRecoveryObservation(
      std::uint64_t keyframe_id, const RtkObservation &observation,
      bool velocity_baseline_available, bool guard_active,
      double post_velocity_error_mps);
  // Adds a status-gated low-rate observation to the production C2 elastic
  // field.  This never changes T_slam or the frontend state.
  ElasticUpdate AddElasticObservation(
      std::uint64_t keyframe_id, const RtkObservation &observation,
      bool use_position_observation = true);

  std::vector<Pose3d> global_poses() const;
  GlobalMapSnapshot global_map_snapshot() const;
  Pose3d latest_global_pose() const;
  Statistics statistics() const;

private:
  Pose3d BaseGlobal(const Pose3d &T_slam_body) const;
  Pose3d SegmentNominal(std::size_t index,
                       const Pose3d &T_slam_body) const;
  Pose3d ToGlobal(std::size_t index, const Pose3d &T_slam_body) const;
  void InitializeTrajectoryCsv();
  void AppendTrajectoryRow(const Keyframe &keyframe,
                           const Pose3d &local_pose,
                           const Pose3d &global_pose);
  void RewriteTrajectoryLocked(
      const std::vector<Keyframe::Ptr> &keyframes,
      const std::vector<Pose3d> &local_slam_poses);
  void RecomputeCumulativeDistanceLocked();
  void RebuildGlobalCorrectionLocked();
  void RebuildRegularizedFieldsLocked();
  void RecomputeGlobalPosesLocked();
  bool StartQuarantineLocked(std::uint64_t keyframe_id);
  void ResetQuarantineLocked();

  struct SelectedObservation
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    RtkObservation observation;
    CorrectionRegime regime = CorrectionRegime::kWarmup;
    bool has_relocalization_evidence = false;
    bool frontend_restart_acknowledged = false;
    bool velocity_baseline_available = false;
    bool guard_active = false;
    double post_velocity_error_mps = 0.0;
  };

  struct EmergencySegmentAnchor
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t trigger_keyframe_id = 0;
    std::uint64_t start_keyframe_id = 0;
    std::uint64_t trusted_start_keyframe_id = 0;
    std::uint64_t transition_end_keyframe_id = 0;
    Eigen::Vector3d rtk_anchor_position = Eigen::Vector3d::Zero();
    bool fitted = false;
    // A recovery is a single gravity-constrained rigid transform. Receiver
    // attitude is never injected; both parts use robust position-path yaw.
    double position_yaw_rad = 0.0;
    double orientation_yaw_rad = 0.0;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    std::shared_ptr<RegularizedCorrectionField4d> elastic_field;
  };

  struct RecoveryObservation
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    RtkObservation observation;
    bool velocity_baseline_available = false;
    bool guard_active = false;
    double post_velocity_error_mps = 0.0;
  };

  struct ElasticObservation
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    RtkObservation observation;
    bool use_position_observation = true;
  };

  Options options_;
  mutable std::mutex mutex_;
  std::vector<Pose3d> global_poses_;
  std::vector<Pose3d> local_slam_poses_;
  std::vector<Keyframe::Ptr> keyframes_;
  std::vector<double> cumulative_distance_m_;
  std::vector<std::uint8_t> global_map_eligible_;
  std::optional<std::uint64_t> quarantine_start_keyframe_id_;
  std::vector<SelectedObservation,
              Eigen::aligned_allocator<SelectedObservation>>
      selected_observations_;
  std::vector<EmergencySegmentAnchor,
              Eigen::aligned_allocator<EmergencySegmentAnchor>>
      emergency_segment_anchors_;
  std::vector<RecoveryObservation,
              Eigen::aligned_allocator<RecoveryObservation>>
      recovery_observations_;
  std::vector<ElasticObservation,
              Eigen::aligned_allocator<ElasticObservation>>
      elastic_observations_;
  bool recovery_monitor_active_ = false;
  bool recovery_elastic_probation_ = false;
  bool recovery_segment_accepted_ = false;
  std::unique_ptr<CorrectionFeasibilityMonitor> correction_monitor_;
  std::unique_ptr<LowFrequencyCorrectionField> correction_field_;
  std::unique_ptr<RegularizedCorrectionField4d> regularized_field_;
  std::unique_ptr<ElasticSegmentAcceptanceMonitor> acceptance_monitor_;
  std::unique_ptr<RigidRelocalizationMonitor> relocalization_monitor_;
  std::unique_ptr<RigidRelocalizationMonitor> recovery_monitor_;
  Statistics statistics_;
  std::ofstream trajectory_stream_;
  std::ofstream observation_stream_;
  std::ofstream regularized_field_stream_;
  std::ofstream elastic_acceptance_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_GLOBAL_POSE_LAYER_H
