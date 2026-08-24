#include "backend/rigid_relocalization_monitor.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
using my_livo::backend::CorrectionRegime;
using my_livo::backend::RigidRelocalizationDecision;
using my_livo::backend::RigidRelocalizationMonitor;
using my_livo::backend::RelocalizationGeometryFailure;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

RigidRelocalizationMonitor::Result Add(
    RigidRelocalizationMonitor &monitor, std::uint64_t id,
    double distance, const Eigen::Vector3d &local,
    const Eigen::Vector3d &rtk, CorrectionRegime regime,
    double velocity_error)
{
  return monitor.AddObservation(
      id, 100.0 + static_cast<double>(id), distance, local, rtk,
      regime, true, true, velocity_error);
}
}  // namespace

int main()
{
  try
  {
    RigidRelocalizationMonitor::Options options;
    options.maximum_post_velocity_error_mps = 0.8;
    options.minimum_observations = 4;
    options.minimum_path_length_m = 30.0;
    options.maximum_window_length_m = 80.0;
    options.maximum_path_scale_error = 0.05;
    options.maximum_planar_rms_m = 0.35;
    options.maximum_vertical_rms_m = 0.50;
    options.required_consecutive_candidates = 2;
    options.restart_after_consecutive_structural_rejections = 3;
    options.restart_minimum_rejection_span_m = 15.0;
    RigidRelocalizationMonitor monitor(options);

    const auto healthy = Add(
        monitor, 0, 0.0, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), CorrectionRegime::kElastic, 0.1);
    Require(healthy.decision == RigidRelocalizationDecision::kMonitoring &&
                !healthy.ready,
            "healthy motion entered relocalization");
    const auto unstable = Add(
        monitor, 1, 10.0, Eigen::Vector3d(10.0, 0.0, 0.0),
        Eigen::Vector3d(8.0, 0.0, 0.0),
        CorrectionRegime::kRelocalizationRequired, 1.2);
    Require(unstable.decision ==
                RigidRelocalizationDecision::kWaitingVelocity &&
                unstable.window_size == 0,
            "unstable velocity was accepted into the fit window");

    RigidRelocalizationMonitor::Result scale_mismatch;
    for (std::uint64_t id = 2; id <= 6; ++id)
    {
      const double x = 10.0 * static_cast<double>(id - 2);
      scale_mismatch = Add(
          monitor, id, 10.0 * id, Eigen::Vector3d(x, 0.0, 0.0),
          Eigen::Vector3d(0.8 * x, 0.0, 0.0),
          CorrectionRegime::kRelocalizationRequired, 0.2);
    }
    Require(scale_mismatch.decision ==
                RigidRelocalizationDecision::kScaleMismatch &&
                std::abs(scale_mismatch.path_scale_ratio - 0.8) < 1.0e-12 &&
                !scale_mismatch.ready,
            "a scaled trajectory passed the rigid 4DOF gate");

    RigidRelocalizationMonitor restart_monitor(options);
    RigidRelocalizationMonitor::Result restart;
    for (std::uint64_t id = 0; id <= 6; ++id)
    {
      const double x = 10.0 * static_cast<double>(id);
      restart = Add(
          restart_monitor, id, x, Eigen::Vector3d(x, 0.0, 0.0),
          Eigen::Vector3d(0.9 * x, 0.0, 0.0),
          CorrectionRegime::kRelocalizationRequired, 0.2);
    }
    Require(restart.decision ==
                RigidRelocalizationDecision::kRestartRequired &&
                restart.frontend_restart_required &&
                restart.restart_state_changed &&
                restart.consecutive_structural_rejections == 3 &&
                restart.structural_rejection_span_m >= 15.0 &&
                restart.geometry_failure ==
                    RelocalizationGeometryFailure::
                        kSystematicScaleWeakGeometry &&
                std::abs(restart.similarity_scale - 0.9) < 1.0e-12 &&
                restart.similarity_rms_m < 1.0e-10,
            "persistent non-rigid evidence did not request a restart");
    const auto restart_latched = Add(
        restart_monitor, 7, 70.0, Eigen::Vector3d(70.0, 0.0, 0.0),
        Eigen::Vector3d(63.0, 0.0, 0.0),
        CorrectionRegime::kRelocalizationRequired, 1.2);
    Require(restart_latched.decision ==
                RigidRelocalizationDecision::kRestartRequired &&
                restart_latched.frontend_restart_required &&
                !restart_latched.restart_state_changed &&
                restart_latched.consecutive_structural_rejections == 3,
            "frontend restart request was not latched across stale evidence");
    restart_monitor.AcknowledgeFrontendRestart(7);
    const auto restarted_segment = Add(
        restart_monitor, 8, 80.0, Eigen::Vector3d(80.0, 0.0, 0.0),
        Eigen::Vector3d(72.0, 0.0, 0.0),
        CorrectionRegime::kRelocalizationRequired, 0.2);
    Require(restarted_segment.decision ==
                RigidRelocalizationDecision::kCollectingBaseline &&
                !restarted_segment.frontend_restart_required &&
                restarted_segment.consecutive_structural_rejections == 0,
            "acknowledged frontend segment did not restart its gate");

    const auto reset_by_velocity = Add(
        monitor, 7, 70.0, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        CorrectionRegime::kRelocalizationRequired, 1.0);
    Require(reset_by_velocity.consecutive_structural_rejections == 0,
            "unlatched structural evidence survived a velocity reset");
    const double yaw = 6.0 * 3.14159265358979323846 / 180.0;
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation(4.0, -3.0, 1.2);
    RigidRelocalizationMonitor::Result ready;
    for (std::uint64_t id = 8; id <= 12; ++id)
    {
      const double x = 10.0 * static_cast<double>(id - 8);
      const Eigen::Vector3d local(x, 0.02 * x * x, 0.1 * x);
      ready = Add(
          monitor, id, 80.0 + x, local, rotation * local + translation,
          CorrectionRegime::kRelocalizationRequired, 0.2);
    }
    Require(ready.decision == RigidRelocalizationDecision::kReady &&
                ready.ready && ready.state_changed,
            "consistent rigid motion did not become ready");
    Require((ready.translation - translation).norm() < 1.0e-10 &&
                std::abs(ready.yaw_rad - yaw) < 1.0e-12 &&
                ready.planar_rms_m < 1.0e-10 &&
                ready.vertical_rms_m < 1.0e-10,
            "gravity-constrained 4DOF fit is inaccurate");

    auto suffix_options = options;
    suffix_options.select_best_valid_suffix = true;
    suffix_options.required_consecutive_candidates = 1;
    suffix_options.restart_after_consecutive_structural_rejections = 1000;
    suffix_options.restart_minimum_rejection_span_m = 1000.0;
    RigidRelocalizationMonitor suffix_monitor(suffix_options);
    RigidRelocalizationMonitor::Result suffix_ready;
    for (std::uint64_t id = 0; id <= 12; ++id)
    {
      const double x = 5.0 * static_cast<double>(id);
      const Eigen::Vector3d local(x, 0.01 * x * x, 0.02 * x);
      Eigen::Vector3d rtk = local + Eigen::Vector3d(2.0, -1.0, 0.5);
      if (id < 4)
        rtk.y() += id % 2 == 0 ? 3.0 : -3.0;
      suffix_ready = Add(
          suffix_monitor, id, x, local, rtk,
          CorrectionRegime::kRelocalizationRequired, 0.1);
    }
    Require(
        suffix_ready.ready && suffix_ready.window_start_keyframe_id >= 4 &&
            suffix_ready.planar_rms_m < 1.0e-10,
        "best valid suffix did not discard a post-restart transient");
  }
  catch (const std::exception &error)
  {
    std::cerr << "rigid_relocalization_monitor_test failed: "
              << error.what() << '\n';
    return 1;
  }
  std::cout << "rigid_relocalization_monitor_test passed\n";
  return 0;
}
