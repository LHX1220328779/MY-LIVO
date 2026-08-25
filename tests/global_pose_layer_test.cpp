#include "backend/global_pose_layer.h"

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using my_livo::backend::GlobalPoseLayer;
using my_livo::backend::Keyframe;
using my_livo::backend::KeyframeCloud;
using my_livo::backend::KeyframePoint;
using my_livo::backend::KeyframeTrigger;
using my_livo::backend::Matrix6d;
using my_livo::backend::Pose3d;
using my_livo::backend::RtkObservation;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

Pose3d MakePose(double x, double y = 0.0)
{
  return Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, y, 0.0));
}

Keyframe::Ptr MakeKeyframe(std::uint64_t id, const Pose3d &pose)
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  cloud->push_back(KeyframePoint());
  return std::make_shared<Keyframe>(
      id, 10.0 + static_cast<double>(id), pose, cloud,
      Matrix6d::Identity(), id == 0 ?
          static_cast<std::uint8_t>(KeyframeTrigger::kFirst) :
          static_cast<std::uint8_t>(KeyframeTrigger::kTranslation));
}
}  // namespace

int main()
{
  try
  {
    GlobalPoseLayer::Options options;
    // This test audits legacy fixed-envelope continuity. Adaptive far-field
    // scaling has its own deterministic unit test.
    options.regularized_field.adaptive_gradient_enabled = false;
    options.T_global_slam = MakePose(5.0, -2.0);
    GlobalPoseLayer layer(options);
    std::vector<Keyframe::Ptr> keyframes{
        MakeKeyframe(0, MakePose(0.0)), MakeKeyframe(1, MakePose(1.0)),
        MakeKeyframe(2, MakePose(21.0))};
    for (const auto &keyframe : keyframes) layer.AppendKeyframe(keyframe);
    Require((keyframes[1]->T_global_body().translation -
             Eigen::Vector3d(6.0, -2.0, 0.0)).norm() < 1.0e-12,
            "fixed A0 was not applied outside the local graph");
    Require((keyframes[1]->T_slam_body().translation -
             Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < 1.0e-12,
            "global layer modified local SLAM");

    const std::vector<Pose3d> loop_updated{
        MakePose(0.0), MakePose(1.0, 0.25), MakePose(21.0)};
    keyframes[1]->set_T_slam_body(loop_updated[1]);
    layer.SynchronizeLocalPoses(keyframes, loop_updated);
    Require((keyframes[1]->T_global_body().translation -
             Eigen::Vector3d(6.0, -1.75, 0.0)).norm() < 1.0e-12,
            "loop-only local update did not propagate to global output");

    RtkObservation observation;
    observation.timestamp = 11.0;
    observation.position = Eigen::Vector3d(6.0, -1.75, 0.0);
    observation.position_covariance = Eigen::Matrix3d::Identity() * 0.01;
    observation.health = 1.0;
    observation.lower_ins_pos_mode = 4;
    observation.upper_ins_pos_mode = 4;
    layer.AddRtkObservation(1, observation);
    layer.AddElasticObservation(1, observation);
    Require((keyframes[1]->T_global_body().translation -
             Eigen::Vector3d(6.0, -1.75, 0.0)).norm() < 1.0e-12,
            "RTK observation directly changed a Phase-2 pose");
    Require(layer.statistics().rtk_observations == 1,
            "RTK observation was not retained by the global layer");

    RtkObservation drift_observation = observation;
    drift_observation.timestamp = 12.0;
    drift_observation.position = Eigen::Vector3d(26.4, -2.0, 0.0);
    const auto update = layer.AddRtkObservation(2, drift_observation);
    const auto elastic_update =
        layer.AddElasticObservation(2, drift_observation);
    Require(update.correction_field.field_changed,
            "bounded global correction field was not updated");
    Require(elastic_update.field_changed &&
                keyframes[2]->T_global_body().translation.x() > 26.0 &&
                keyframes[2]->T_global_body().translation.x() < 26.401,
            "causal rigid global correction was not applied");
    Require((keyframes[2]->T_slam_body().translation -
             Eigen::Vector3d(21.0, 0.0, 0.0)).norm() < 1.0e-12,
            "global correction modified local SLAM");

    GlobalPoseLayer::Options quarantine_options;
    GlobalPoseLayer quarantine_layer(quarantine_options);
    std::vector<Keyframe::Ptr> quarantine_keyframes{
        MakeKeyframe(0, MakePose(0.0)), MakeKeyframe(1, MakePose(20.0)),
        MakeKeyframe(2, MakePose(40.0))};
    for (const auto &keyframe : quarantine_keyframes)
      quarantine_layer.AppendKeyframe(keyframe);
    for (std::uint64_t id = 0; id < quarantine_keyframes.size(); ++id)
    {
      RtkObservation sample;
      sample.timestamp = 20.0 + static_cast<double>(id);
      sample.position = Eigen::Vector3d(30.0 * id, 0.0, 0.0);
      sample.position_covariance = Eigen::Matrix3d::Identity() * 0.01;
      const auto quarantine_update =
          quarantine_layer.AddRtkObservation(id, sample);
      if (id == 2)
        Require(quarantine_update.quarantine_started,
                "relocalization latch did not start map quarantine");
    }
    const auto tail = MakeKeyframe(3, MakePose(60.0));
    quarantine_layer.AppendKeyframe(tail);
    const auto snapshot = quarantine_layer.global_map_snapshot();
    Require(snapshot.eligible ==
                std::vector<std::uint8_t>({1U, 1U, 0U, 0U}),
            "global-map quarantine mask did not cover the failed tail");

    options.recovery_monitor.minimum_observations = 3;
    options.recovery_monitor.minimum_path_length_m = 10.0;
    options.recovery_monitor.required_consecutive_candidates = 1;
    options.recovery_monitor.maximum_path_scale_error = 0.10;
    options.recovery_monitor.maximum_planar_rms_m = 0.10;
    options.recovery_monitor.maximum_vertical_rms_m = 0.10;
    GlobalPoseLayer segment_layer(options);
    std::vector<Keyframe::Ptr> segment_keyframes{
        MakeKeyframe(0, MakePose(0.0)), MakeKeyframe(1, MakePose(10.0))};
    for (const auto &keyframe : segment_keyframes)
      segment_layer.AppendKeyframe(keyframe);
    segment_layer.StartEmergencyGlobalSegment(
        1, Eigen::Vector3d(100.0, 20.0, 3.0));
    const auto recovered = MakeKeyframe(2, MakePose(12.0));
    segment_layer.AppendKeyframe(recovered);
    Require((recovered->T_global_body().translation -
             Eigen::Vector3d(17.0, -2.0, 0.0)).norm() < 1.0e-12,
            "emergency segment did not start continuously");
    Require((recovered->T_slam_body().translation -
             Eigen::Vector3d(12.0, 0.0, 0.0)).norm() < 1.0e-12,
            "emergency global segment modified local SLAM");
    Require(segment_layer.global_map_snapshot().eligible.back() == 0U,
            "unvalidated emergency segment escaped quarantine");

    const auto recovered_3 = MakeKeyframe(3, MakePose(22.0));
    const auto recovered_4 = MakeKeyframe(4, MakePose(32.0));
    segment_layer.AppendKeyframe(recovered_3);
    segment_layer.AppendKeyframe(recovered_4);
    const auto provisional_before_fit = segment_layer.global_poses();
    for (std::uint64_t id = 2; id <= 4; ++id)
    {
      RtkObservation recovery_observation;
      recovery_observation.timestamp = 30.0 + static_cast<double>(id);
      recovery_observation.position = Eigen::Vector3d(
          100.0, 22.0 + 10.0 * static_cast<double>(id - 2), 3.0);
      recovery_observation.position_covariance =
          Eigen::Matrix3d::Identity() * 0.01;
      const auto recovery_update = segment_layer.AddRecoveryObservation(
          id, recovery_observation, true, true, 0.1);
      if (id == 4)
        Require(recovery_update.segment_reanchored,
                "stable recovery window did not reanchor the segment");
    }
    const auto recovered_poses = segment_layer.global_poses();
    for (std::uint64_t id = 2; id <= 4; ++id)
      Require(
          (recovered_poses[id].translation -
           provisional_before_fit[id].translation).norm() < 1.0e-12 &&
              recovered_poses[id].rotation.angularDistance(
                  provisional_before_fit[id].rotation) < 1.0e-12,
          "recovery gate changed the continuous quarantined history");

    RtkObservation first_elastic;
    first_elastic.timestamp = 34.0;
    first_elastic.position = Eigen::Vector3d(100.0, 42.0, 3.0);
    first_elastic.position_covariance =
        Eigen::Matrix3d::Identity() * 0.01;
    first_elastic.lower_ins_pos_mode = 4;
    first_elastic.upper_ins_pos_mode = 4;
    segment_layer.AddElasticObservation(4, first_elastic, true);
    Require(
        (segment_layer.global_poses()[4].translation -
         provisional_before_fit[4].translation).norm() < 1.0e-12,
        "first recovery elastic knot did not preserve continuity");

    const auto recovered_5 = MakeKeyframe(5, MakePose(42.0));
    segment_layer.AppendKeyframe(recovered_5);
    RtkObservation second_elastic;
    second_elastic.timestamp = 35.0;
    second_elastic.position = Eigen::Vector3d(100.0, 52.0, 3.0);
    second_elastic.position_covariance =
        Eigen::Matrix3d::Identity() * 0.01;
    second_elastic.lower_ins_pos_mode = 4;
    second_elastic.upper_ins_pos_mode = 4;
    segment_layer.AddRecoveryObservation(5, second_elastic, true, true, 0.1);
    segment_layer.AddElasticObservation(5, second_elastic, true);
    const auto after_elastic = segment_layer.global_poses();
    const Eigen::Vector3d carried_position =
        provisional_before_fit[4].translation + Eigen::Vector3d(10.0, 0.0, 0.0);
    const double released_correction =
        (after_elastic[5].translation - carried_position).norm();
    Require(released_correction > 1.0e-6 && released_correction < 0.6,
            "recovery elastic field was either frozen or discontinuous");
    Require(segment_layer.statistics().recovery_reanchors == 1,
            "recovery reanchor statistics differ");

    GlobalPoseLayer immediate_layer(options);
    immediate_layer.AppendKeyframe(MakeKeyframe(0, MakePose(0.0)));
    immediate_layer.AppendKeyframe(MakeKeyframe(1, MakePose(10.0)));
    immediate_layer.StartEmergencyGlobalSegment(
        1, Eigen::Vector3d(100.0, 20.0, 3.0));
    immediate_layer.AppendKeyframe(MakeKeyframe(2, MakePose(20.0)));
    immediate_layer.AppendKeyframe(MakeKeyframe(3, MakePose(30.0)));
    RtkObservation immediate_observation;
    immediate_observation.timestamp = 32.0;
    immediate_observation.position = Eigen::Vector3d(100.0, 22.0, 3.0);
    immediate_observation.position_covariance =
        Eigen::Matrix3d::Identity() * 0.01;
    immediate_observation.lower_ins_pos_mode = 4;
    immediate_observation.upper_ins_pos_mode = 4;
    immediate_layer.AddRecoveryObservation(
        2, immediate_observation, true, true, 0.1);
    immediate_layer.AddElasticObservation(2, immediate_observation, true);
    const Pose3d carried_before = immediate_layer.global_poses()[3];
    immediate_observation.timestamp = 33.0;
    immediate_observation.position = Eigen::Vector3d(100.0, 32.0, 3.0);
    const auto immediate_recovery = immediate_layer.AddRecoveryObservation(
        3, immediate_observation, true, true, 0.1);
    const auto immediate_elastic = immediate_layer.AddElasticObservation(
        3, immediate_observation, true);
    const double immediate_release =
        (immediate_layer.global_poses()[3].translation -
         carried_before.translation).norm();
    Require(!immediate_recovery.segment_reanchored &&
                immediate_elastic.field_changed &&
                immediate_release > 1.0e-6 && immediate_release < 0.6 &&
                immediate_layer.global_map_snapshot().eligible.back() == 0U,
            "quarantined elastic recovery waited for or escaped its gate");
  }
  catch (const std::exception &error)
  {
    std::cerr << "global_pose_layer_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "global_pose_layer_test passed\n";
  return 0;
}
