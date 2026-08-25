#include "backend/rtk_velocity_guard.h"

#include <iostream>
#include <stdexcept>

namespace
{
using my_livo::backend::CorrectionRegime;
using my_livo::backend::RtkVelocityGuard;
using my_livo::backend::RtkVelocityGuardDecision;
using my_livo::backend::RtkVelocitySource;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

RtkVelocityGuard::Result Add(
    RtkVelocityGuard &guard, std::uint64_t id, double position,
    double lio_speed, CorrectionRegime regime)
{
  return guard.AddObservation(
      id, 10.0 + static_cast<double>(id),
      Eigen::Vector3d(position, 0.0, 0.0),
      Eigen::Vector3d(position, 0.0, 0.0),
      Eigen::Vector3d(lio_speed, 0.0, 0.0), regime);
}
}  // namespace

int main()
{
  try
  {
    RtkVelocityGuard::Options options;
    options.low_pass_time_constant_sec = 2.0;
    options.activation_velocity_error_mps = 1.0;
    options.activation_speed_ratio = 1.35;
    options.activation_consecutive_observations = 2;
    options.recovery_velocity_error_mps = 0.5;
    options.recovery_consecutive_observations = 3;
    options.correction_time_constant_sec = 0.8;
    options.maximum_planar_correction_mps = 2.0;
    options.maximum_vertical_correction_mps = 0.75;
    options.maximum_planar_correction_acceleration_mps2 = 0.30;
    options.maximum_vertical_correction_acceleration_mps2 = 0.15;
    RtkVelocityGuard guard(options);

    const auto warmup = Add(
        guard, 0, 0.0, 1.0, CorrectionRegime::kWarmup);
    Require(!warmup.baseline_available && !warmup.active,
            "velocity guard did not begin in warmup");
    const auto healthy = Add(
        guard, 1, 1.0, 1.0, CorrectionRegime::kElastic);
    Require(healthy.baseline_available && !healthy.active &&
                !healthy.correction_applied,
            "healthy LIO activated the velocity guard");

    const auto evidence = Add(
        guard, 2, 2.0, 3.0, CorrectionRegime::kDegraded);
    Require(!evidence.active && !evidence.correction_applied,
            "single degraded sample activated the velocity guard");
    const auto activated = Add(
        guard, 3, 3.0, 5.0,
        CorrectionRegime::kRelocalizationRequired);
    Require(activated.active && activated.state_changed &&
                activated.correction_applied &&
                activated.decision == RtkVelocityGuardDecision::kActivated,
            "severe relocalization evidence did not activate the guard");
    Require(std::abs(activated.applied_correction.x() + 0.3) < 1.0e-12 &&
                std::abs(activated.lio_velocity_after.x() - 4.7) < 1.0e-12,
            "velocity correction was not gain/rate bounded");

    Require(Add(guard, 4, 4.0, 1.0,
                CorrectionRegime::kRelocalizationRequired).active,
            "guard recovered after only one healthy sample");
    Require(Add(guard, 5, 5.0, 1.0,
                CorrectionRegime::kRelocalizationRequired).active,
            "guard recovered after only two healthy samples");
    const auto recovered = Add(
        guard, 6, 6.0, 1.0,
        CorrectionRegime::kRelocalizationRequired);
    Require(!recovered.active && recovered.state_changed &&
                recovered.decision == RtkVelocityGuardDecision::kRecovered,
            "guard did not recover with hysteresis");

    RtkVelocityGuard receiver_guard(options);
    receiver_guard.AddObservation(
        0, 20.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        CorrectionRegime::kElastic, Eigen::Vector3d(4.0, 1.0, 0.0));
    const auto receiver = receiver_guard.AddObservation(
        1, 21.0, Eigen::Vector3d(4.0, 1.0, 0.0),
        Eigen::Vector3d(4.0, 1.0, 0.0),
        Eigen::Vector3d(4.0, 1.0, 0.0), CorrectionRegime::kElastic,
        Eigen::Vector3d(4.0, 1.0, 0.0));
    Require(receiver.velocity_source == RtkVelocitySource::kReceiverTwist &&
                (receiver.raw_rtk_velocity -
                 Eigen::Vector3d(4.0, 1.0, 0.0)).norm() < 1.0e-12 &&
                !receiver.active,
            "receiver twist did not override noisy position differencing");
    const auto inconsistent_receiver = receiver_guard.AddObservation(
        2, 22.0, Eigen::Vector3d(104.0, 1.0, 0.0),
        Eigen::Vector3d(104.0, 1.0, 0.0),
        Eigen::Vector3d(4.0, 1.0, 0.0), CorrectionRegime::kElastic,
        Eigen::Vector3d::Zero());
    Require(inconsistent_receiver.velocity_source ==
                RtkVelocitySource::kPositionDifference,
            "implausible receiver twist did not use the safe fallback");

    RtkVelocityGuard direction_guard(options);
    direction_guard.AddObservation(
        0, 24.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 1.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    const auto direction_evidence = direction_guard.AddObservation(
        1, 25.0, Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    const auto direction_activated = direction_guard.AddObservation(
        2, 26.0, Eigen::Vector3d(2.0, 0.0, 0.0),
        Eigen::Vector3d(2.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(
        !direction_evidence.active && direction_activated.active &&
            direction_activated.speed_ratio < 1.01 &&
            direction_activated.correction_applied,
        "equal-speed direction divergence did not activate the guard");

    options.healthy_vertical_aiding_enabled = true;
    RtkVelocityGuard healthy_vertical_guard(options);
    healthy_vertical_guard.AddObservation(
        0, 40.0, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(1.0, 0.0, 0.30), CorrectionRegime::kElastic,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    RtkVelocityGuard::Result vertical_aiding;
    for (std::uint64_t id = 1; id <= 5; ++id)
      vertical_aiding = healthy_vertical_guard.AddObservation(
          id, 40.0 + static_cast<double>(id),
          Eigen::Vector3d(static_cast<double>(id), 0.0, 0.0),
          Eigen::Vector3d(static_cast<double>(id), 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.30), CorrectionRegime::kElastic,
          Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(
        vertical_aiding.decision ==
            RtkVelocityGuardDecision::kHealthyVerticalAiding &&
            !vertical_aiding.active &&
            vertical_aiding.applied_correction.head<2>().norm() < 1.0e-12 &&
            vertical_aiding.applied_correction.z() < 0.0 &&
            std::abs(vertical_aiding.applied_correction.z()) <= 0.0800001,
        "healthy vertical aid was not sustained, isolated, and bounded");

    RtkVelocityGuard emergency_guard(options);
    emergency_guard.AddObservation(
        0, 30.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d(4.0, 0.0, 0.0),
        CorrectionRegime::kRelocalizationRequired,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    RtkVelocityGuard::Result emergency;
    for (std::uint64_t id = 1; id <= 3; ++id)
      emergency = emergency_guard.AddObservation(
          id, 30.0 + static_cast<double>(id),
          Eigen::Vector3d(static_cast<double>(id), 0.0, 0.0),
          Eigen::Vector3d(static_cast<double>(id), 0.0, 0.0),
          Eigen::Vector3d(4.0, 0.0, 0.0),
          CorrectionRegime::kRelocalizationRequired,
          Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(emergency.emergency_restart_required &&
                emergency.emergency_restart_state_changed &&
                emergency.emergency_restart_evidence == 3,
            "persistent receiver velocity error did not request recovery");
    emergency_guard.AcknowledgeEmergencyRestart(3);
    const auto tracking = emergency_guard.AddObservation(
        4, 34.0, Eigen::Vector3d(4.0, 0.0, 0.0),
        Eigen::Vector3d(4.0, 0.0, 0.0),
        Eigen::Vector3d(4.0, 0.0, 0.0),
        CorrectionRegime::kRelocalizationRequired,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(tracking.recovery_tracking &&
                std::abs(tracking.applied_correction.x() + 1.5) < 1.0e-12,
            "quarantined recovery velocity did not use its bounded tracker");
    emergency_guard.CompleteRecoveryTracking(4);
    const auto completed = emergency_guard.AddObservation(
        5, 35.0, Eigen::Vector3d(5.0, 0.0, 0.0),
        Eigen::Vector3d(5.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        CorrectionRegime::kRelocalizationRequired,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(!completed.recovery_tracking && !completed.active &&
                !completed.correction_applied,
            "4DOF completion did not release recovery velocity tracking");

    RtkVelocityGuard outer_loop_guard(options);
    outer_loop_guard.AddObservation(
        0, 50.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d(1.0, 0.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    outer_loop_guard.BeginRecoveryTracking(0);
    const auto closing = outer_loop_guard.AddObservation(
        1, 51.0, Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(4.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(
        std::abs(closing.global_position_error.x()-3.0) < 1.0e-12 &&
            std::abs(closing.recovery_closure_velocity.x()+0.375) <
                1.0e-12 &&
            std::abs(closing.tracking_target_velocity.x()-0.625) <
                1.0e-12 &&
            closing.applied_correction.x() < 0.0,
        "radial recovery position error did not bias velocity inward");
    const auto captured = outer_loop_guard.AddObservation(
        2, 52.0, Eigen::Vector3d(2.0, 0.0, 0.0),
        Eigen::Vector3d(2.16, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(
        captured.recovery_planar_capture_active &&
            std::abs(captured.recovery_planar_stiffness-0.5) < 1.0e-12 &&
            captured.recovery_closure_velocity.x() < 0.0,
        "recovery capture did not retain finite restoring stiffness");
    const auto retained = outer_loop_guard.AddObservation(
        3, 53.0, Eigen::Vector3d(3.0, 0.0, 0.0),
        Eigen::Vector3d(3.12, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    const auto released = outer_loop_guard.AddObservation(
        4, 54.0, Eigen::Vector3d(4.0, 0.0, 0.0),
        Eigen::Vector3d(4.09, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0), CorrectionRegime::kDegraded,
        Eigen::Vector3d(1.0, 0.0, 0.0));
    Require(
        retained.recovery_planar_capture_active &&
            !released.recovery_planar_capture_active &&
            std::abs(released.recovery_closure_velocity.x()) < 1.0e-12,
        "recovery capture hysteresis released at the wrong boundary");
  }
  catch (const std::exception &error)
  {
    std::cerr << "rtk_velocity_guard_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "rtk_velocity_guard_test passed\n";
  return 0;
}
