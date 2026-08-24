#ifndef MY_LIVO_BACKEND_RIGID_RELOCALIZATION_MONITOR_H
#define MY_LIVO_BACKEND_RIGID_RELOCALIZATION_MONITOR_H

#include "backend/correction_feasibility_monitor.h"

#include <Eigen/Core>

#include <cstdint>
#include <deque>
#include <fstream>
#include <string>

namespace my_livo::backend
{

enum class RigidRelocalizationDecision
{
  kMonitoring,
  kWaitingVelocity,
  kCollectingBaseline,
  kScaleMismatch,
  kFitRejected,
  kCandidate,
  kReady,
  kRestartRequired,
};

const char *RigidRelocalizationDecisionToString(
    RigidRelocalizationDecision decision);

enum class RelocalizationGeometryFailure
{
  kNone,
  kWeakGeometry,
  kSystematicScale,
  kSystematicScaleWeakGeometry,
  kAttitudeMisalignment,
  kScaleAndAttitude,
  kNonRigid,
};

const char *RelocalizationGeometryFailureToString(
    RelocalizationGeometryFailure failure);

// Shadow-mode gate for starting a new globally aligned map segment. It fits
// translation+yaw only: scale and gravity direction are deliberately fixed.
class RigidRelocalizationMonitor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options
  {
    bool enabled = true;
    double maximum_post_velocity_error_mps = 0.8;
    int minimum_observations = 4;
    double minimum_path_length_m = 30.0;
    double maximum_window_length_m = 80.0;
    double maximum_path_scale_error = 0.05;
    double maximum_planar_rms_m = 0.35;
    double maximum_vertical_rms_m = 0.50;
    int required_consecutive_candidates = 2;
    // Recovery may discard an early post-restart transient, but every tested
    // suffix must independently satisfy the full observation/path gates.
    bool select_best_valid_suffix = false;
    int restart_after_consecutive_structural_rejections = 3;
    double restart_minimum_rejection_span_m = 15.0;
    double diagnostic_minimum_condition_ratio = 0.01;
    double diagnostic_minimum_scale_error = 0.03;
    double diagnostic_minimum_nonyaw_rotation_deg = 1.0;
    double diagnostic_maximum_similarity_rms_m = 0.40;
    std::string csv_path;
  };

  struct Result
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    RigidRelocalizationDecision decision =
        RigidRelocalizationDecision::kMonitoring;
    bool ready = false;
    bool state_changed = false;
    bool frontend_restart_required = false;
    bool restart_state_changed = false;
    RigidRelocalizationDecision restart_reason =
        RigidRelocalizationDecision::kMonitoring;
    int consecutive_structural_rejections = 0;
    double structural_rejection_span_m = 0.0;
    bool guard_active = false;
    double post_velocity_error_mps = 0.0;
    std::size_t window_size = 0;
    std::uint64_t window_start_keyframe_id = 0;
    double local_path_length_m = 0.0;
    double rtk_path_length_m = 0.0;
    double path_scale_ratio = 1.0;
    double planar_rms_m = 0.0;
    double vertical_rms_m = 0.0;
    double cumulative_distance_m = 0.0;
    bool full_rotation_observable = false;
    double geometry_condition_ratio = 0.0;
    double se3_nonyaw_rotation_deg = 0.0;
    double se3_rms_m = 0.0;
    double similarity_scale = 1.0;
    double similarity_rms_m = 0.0;
    RelocalizationGeometryFailure geometry_failure =
        RelocalizationGeometryFailure::kNone;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double yaw_rad = 0.0;
  };

  explicit RigidRelocalizationMonitor(const Options &options);

  Result AddObservation(
      std::uint64_t keyframe_id, double timestamp, double distance_m,
      const Eigen::Vector3d &local_position,
      const Eigen::Vector3d &rtk_position, CorrectionRegime regime,
      bool velocity_baseline_available, bool guard_active,
      double post_velocity_error_mps);

  void AcknowledgeFrontendRestart(std::uint64_t keyframe_id);

  bool ready() const { return ready_; }

private:
  struct Sample
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    double distance_m = 0.0;
    Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d rtk_position = Eigen::Vector3d::Zero();
  };

  static void ValidateOptions(const Options &options);
  void ClearWindow();
  void Fit(std::size_t begin_index, Result *result) const;
  void WriteCsv(double timestamp, CorrectionRegime regime,
                const Result &result);

  Options options_;
  bool have_previous_input_ = false;
  std::uint64_t previous_keyframe_id_ = 0;
  double previous_timestamp_ = 0.0;
  std::deque<Sample, Eigen::aligned_allocator<Sample>> window_;
  int consecutive_candidates_ = 0;
  int consecutive_structural_rejections_ = 0;
  double first_structural_rejection_distance_m_ = 0.0;
  bool frontend_restart_required_ = false;
  RigidRelocalizationDecision restart_reason_ =
      RigidRelocalizationDecision::kMonitoring;
  bool ready_ = false;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_RIGID_RELOCALIZATION_MONITOR_H
