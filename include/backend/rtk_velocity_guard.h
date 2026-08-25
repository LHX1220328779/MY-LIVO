#ifndef MY_LIVO_BACKEND_RTK_VELOCITY_GUARD_H
#define MY_LIVO_BACKEND_RTK_VELOCITY_GUARD_H

#include "backend/correction_feasibility_monitor.h"

#include <Eigen/Core>

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>

namespace my_livo::backend
{

enum class RtkVelocityGuardDecision
{
  kWarmup,
  kMonitoring,
  kActivated,
  kCorrected,
  kHealthyVerticalAiding,
  kRecovered,
};

enum class RtkVelocitySource
{
  kUnavailable,
  kReceiverTwist,
  kPositionDifference,
};

const char *RtkVelocitySourceToString(RtkVelocitySource source);

const char *RtkVelocityGuardDecisionToString(
    RtkVelocityGuardDecision decision);

// Low-rate safety controller for an already-degenerate LIO propagation.
// Status-gated receiver twist is preferred; position differencing is only a
// fallback. Corrections are acceleration-bounded, never change pose directly,
// and remain inactive during healthy LIO.
class RtkVelocityGuard
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options
  {
    bool enabled = true;
    double low_pass_time_constant_sec = 2.0;
    double receiver_low_pass_time_constant_sec = 0.25;
    double maximum_receiver_position_difference_error_mps = 1.5;
    double activation_velocity_error_mps = 1.0;
    double activation_speed_ratio = 1.35;
    int activation_consecutive_observations = 2;
    double recovery_velocity_error_mps = 0.5;
    int recovery_consecutive_observations = 3;
    // Converts irregular observation intervals to a cadence-independent
    // first-order convergence gain: 1-exp(-dt/tau).
    double correction_time_constant_sec = 0.8;
    double maximum_planar_correction_mps = 2.0;
    double maximum_vertical_correction_mps = 0.75;
    double maximum_planar_correction_acceleration_mps2 = 0.75;
    double maximum_vertical_correction_acceleration_mps2 = 0.30;
    // Slow status-4 receiver-twist aid for a sustained vertical propagation
    // bias. It never uses RTK position and never changes pose directly.
    bool healthy_vertical_aiding_enabled = true;
    double healthy_vertical_activation_error_mps = 0.08;
    int healthy_vertical_activation_observations = 5;
    double healthy_vertical_time_constant_sec = 4.0;
    double healthy_vertical_maximum_acceleration_mps2 = 0.08;
    // Stronger limits are used only after the old voxel map has been
    // discarded and while the new segment is quarantined. Position is still
    // untouched; this prevents propagation velocity from immediately
    // diverging again before the rigid recovery gate has enough baseline.
    double recovery_tracking_time_constant_sec = 0.35;
    double recovery_tracking_planar_acceleration_mps2 = 1.50;
    double recovery_tracking_vertical_acceleration_mps2 = 0.75;
    // During quarantined recovery, a radial position outer loop biases the
    // velocity target and never writes pose.  A hysteretic capture latch
    // removes the proportional controller's steady-state error without
    // reacting to RTK jitter while ordinary local LIO is healthy.
    double recovery_position_soft_radius_m = 0.10;
    double recovery_position_full_radius_m = 0.50;
    double recovery_position_release_radius_m = 0.08;
    double recovery_position_capture_minimum_stiffness = 0.50;
    double recovery_position_time_constant_sec = 1.5;
    double recovery_maximum_planar_closure_velocity_mps = 0.80;
    double recovery_maximum_vertical_closure_velocity_mps = 0.80;
    // A bounded correction is intentionally slow.  If a status-4 receiver
    // velocity still disagrees strongly after several low-rate updates, the
    // frontend propagation has already left the elastic-recovery envelope and
    // needs one controlled local-segment restart.
    bool emergency_restart_enabled = true;
    double emergency_restart_velocity_error_mps = 2.0;
    int emergency_restart_consecutive_observations = 3;
    std::string csv_path;
  };

  struct Result
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    double timestamp = 0.0;
    RtkVelocityGuardDecision decision = RtkVelocityGuardDecision::kWarmup;
    RtkVelocitySource velocity_source = RtkVelocitySource::kUnavailable;
    bool baseline_available = false;
    bool active = false;
    bool state_changed = false;
    bool correction_applied = false;
    bool recovery_tracking = false;
    bool emergency_restart_required = false;
    bool emergency_restart_state_changed = false;
    int emergency_restart_evidence = 0;
    double interval_sec = 0.0;
    double velocity_error_mps = 0.0;
    double speed_ratio = 1.0;
    double applied_gain = 0.0;
    Eigen::Vector3d raw_rtk_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d filtered_rtk_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_position_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d recovery_closure_velocity = Eigen::Vector3d::Zero();
    double recovery_planar_stiffness = 0.0;
    double recovery_vertical_stiffness = 0.0;
    bool recovery_planar_capture_active = false;
    bool recovery_vertical_capture_active = false;
    Eigen::Vector3d tracking_target_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d lio_velocity_before = Eigen::Vector3d::Zero();
    Eigen::Vector3d lio_velocity_after = Eigen::Vector3d::Zero();
    Eigen::Vector3d applied_correction = Eigen::Vector3d::Zero();
  };

  explicit RtkVelocityGuard(const Options &options);

  Result AddObservation(std::uint64_t keyframe_id, double timestamp,
                        const Eigen::Vector3d &rtk_position,
                        const Eigen::Vector3d &global_position,
                        const Eigen::Vector3d &lio_velocity,
                        CorrectionRegime regime,
                        const std::optional<Eigen::Vector3d> &
                            receiver_velocity = std::nullopt);

  bool active() const { return active_; }
  void BeginRecoveryTracking(std::uint64_t keyframe_id);
  void AcknowledgeEmergencyRestart(std::uint64_t keyframe_id);
  void CompleteRecoveryTracking(std::uint64_t keyframe_id);

private:
  static void ValidateOptions(const Options &options);
  void WriteCsv(double timestamp, CorrectionRegime regime,
                const Result &result);

  Options options_;
  bool have_previous_ = false;
  std::uint64_t previous_keyframe_id_ = 0;
  double previous_timestamp_ = 0.0;
  Eigen::Vector3d previous_rtk_position_ = Eigen::Vector3d::Zero();
  bool have_filtered_velocity_ = false;
  Eigen::Vector3d filtered_rtk_velocity_ = Eigen::Vector3d::Zero();
  bool active_ = false;
  int activation_evidence_ = 0;
  int recovery_evidence_ = 0;
  int healthy_vertical_evidence_ = 0;
  int emergency_restart_evidence_ = 0;
  bool emergency_restart_required_ = false;
  bool emergency_restart_acknowledged_ = false;
  bool recovery_tracking_ = false;
  bool recovery_planar_capture_active_ = false;
  bool recovery_vertical_capture_active_ = false;
  std::uint64_t emergency_restart_keyframe_id_ = 0;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_RTK_VELOCITY_GUARD_H
