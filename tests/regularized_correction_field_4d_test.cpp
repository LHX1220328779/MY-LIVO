#include "backend/regularized_correction_field_4d.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

using my_livo::backend::Pose3d;
using my_livo::backend::RegularizedCorrectionField4d;
using my_livo::backend::RtkObservation;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

Pose3d Pose(double x, double y, double z, double yaw_deg)
{
  return Pose3d(
      Eigen::Quaterniond(Eigen::AngleAxisd(
          yaw_deg * kDegreesToRadians, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(x, y, z));
}

RtkObservation Observation(
    double timestamp, const Eigen::Vector3d &position, double yaw_deg,
    int mode = 4)
{
  RtkObservation observation;
  observation.timestamp = timestamp;
  observation.position = position;
  observation.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
      yaw_deg * kDegreesToRadians, Eigen::Vector3d::UnitZ()));
  observation.position_covariance =
      (Eigen::Vector3d(0.09, 0.09, 0.16)).asDiagonal();
  observation.lower_ins_pos_mode = mode;
  observation.upper_ins_pos_mode = mode;
  return observation;
}

double SlopeVariation(
    const std::vector<RegularizedCorrectionField4d::KnotState,
                      Eigen::aligned_allocator<
                          RegularizedCorrectionField4d::KnotState>> &knots)
{
  double variation = 0.0;
  for (std::size_t index = 1; index + 1 < knots.size(); ++index)
  {
    const double left_spacing =
        knots[index].cumulative_distance_m -
        knots[index - 1].cumulative_distance_m;
    const double right_spacing =
        knots[index + 1].cumulative_distance_m -
        knots[index].cumulative_distance_m;
    const double left_slope =
        (knots[index].fitted_correction.displacement.x() -
         knots[index - 1].fitted_correction.displacement.x()) /
        left_spacing;
    const double right_slope =
        (knots[index + 1].fitted_correction.displacement.x() -
         knots[index].fitted_correction.displacement.x()) /
        right_spacing;
    variation += (right_slope - left_slope) *
        (right_slope - left_slope);
  }
  return variation;
}

void TestStatusOrderingAndAnchorOptions()
{
  RegularizedCorrectionField4d anchored;
  Require(
      std::abs(anchored.ElasticStiffness(0.10)) < 1.0e-12 &&
          std::abs(anchored.ElasticStiffness(0.25) - 1.0) < 1.0e-12 &&
          anchored.ElasticStiffness(0.18) >
              anchored.ElasticStiffness(0.14),
      "distance-adaptive elastic stiffness is not monotonic");
  const Pose3d nominal = Pose(0.0, 0.0, 0.0, 0.0);
  const auto rejected = anchored.AddObservation(
      0, 1.0, 0.0, nominal,
      Observation(1.0, Eigen::Vector3d(1.0, 0.0, 0.0), 5.0, 3));
  Require(
      rejected.decision ==
          RegularizedCorrectionField4d::AddDecision::kStatusRejected &&
          anchored.knot_count() == 0,
      "non-status-4 RTK observation entered the correction field");

  const auto first = anchored.AddObservation(
      0, 1.0, 0.0, nominal,
      Observation(1.0, Eigen::Vector3d(1.0, -0.2, 0.1), 5.0));
  Require(
      first.fitted_correction.displacement.norm() < 1.0e-14 &&
          std::abs(first.fitted_correction.yaw_rad) < 1.0e-14,
      "identity gauge did not constrain the first knot exactly");

  RegularizedCorrectionField4d::Options free_options;
  free_options.anchor_first_knot_identity = false;
  RegularizedCorrectionField4d free_field(free_options);
  const auto free_first = free_field.AddObservation(
      0, 1.0, 0.0, nominal,
      Observation(1.0, Eigen::Vector3d(0.6, -0.2, 0.1), 5.0));
  Require(
      (free_first.fitted_correction.displacement -
       Eigen::Vector3d(0.6, -0.2, 0.1)).norm() < 1.0e-12,
      "recovery segment could not softly correct its first fitted knot");

  RegularizedCorrectionField4d::Options spaced_options;
  spaced_options.minimum_knot_time_interval_sec = 0.5;
  spaced_options.minimum_knot_spacing_m = 2.0;
  RegularizedCorrectionField4d spaced(spaced_options);
  spaced.AddObservation(
      0, 1.0, 0.0, nominal, Observation(1.0, nominal.translation, 0.0));
  const auto too_close = spaced.AddObservation(
      1, 1.4, 3.0, Pose(3.0, 0.0, 0.0, 0.0),
      Observation(1.4, Eigen::Vector3d(3.0, 0.0, 0.0), 0.0));
  Require(
      too_close.decision ==
          RegularizedCorrectionField4d::AddDecision::kSpacingRejected &&
          spaced.knot_count() == 1,
      "high-rate observation was not thinned by configured spacing");
}

void TestRegularizationNaturalSplineAndExtrapolation()
{
  RegularizedCorrectionField4d::Options options;
  options.orientation_yaw_window_m = 12.0;
  options.maximum_extrapolation_distance_m = 8.0;
  options.maximum_planar_extrapolation_slope_m_per_m = 0.10;
  options.maximum_vertical_extrapolation_slope_m_per_m = 0.05;
  options.maximum_yaw_extrapolation_slope_deg_per_m = 0.20;
  RegularizedCorrectionField4d field(options);

  for (std::uint64_t index = 0; index <= 15; ++index)
  {
    const double distance = 4.0 * static_cast<double>(index);
    const Pose3d nominal = Pose(distance, 0.0, 0.0, 0.0);
    const Eigen::Vector3d target(
        0.02 * distance, -0.01 * distance, 0.005 * distance);
    field.AddObservation(
        index, 10.0 + static_cast<double>(index), distance, nominal,
        Observation(
            10.0 + static_cast<double>(index),
            nominal.translation + target, 0.10 * distance));
  }

  const auto states = field.knots();
  Require(states.size() == 16, "not every 1 Hz correction knot was solved");
  Require(
      states.front().fitted_correction.displacement.norm() < 1.0e-14,
      "initial correction gauge moved after global re-solving");

  const auto start = field.EvaluateWithDerivatives(0.0);
  const auto finish = field.EvaluateWithDerivatives(60.0);
  Require(
      start.second_derivative.displacement.norm() < 1.0e-12 &&
          std::abs(start.second_derivative.yaw_rad) < 1.0e-12 &&
          finish.second_derivative.displacement.norm() < 1.0e-12 &&
          std::abs(finish.second_derivative.yaw_rad) < 1.0e-12,
      "natural spline boundary did not have zero second derivative");
  Require(
      finish.first_derivative.displacement.head<2>().norm() > 0.005,
      "newest knot was incorrectly forced to zero slope");

  for (std::size_t index = 1; index + 1 < states.size(); ++index)
  {
    const double distance = states[index].cumulative_distance_m;
    const auto left = field.EvaluateWithDerivatives(distance - 1.0e-6);
    const auto right = field.EvaluateWithDerivatives(distance + 1.0e-6);
    Require(
        (left.first_derivative.displacement -
         right.first_derivative.displacement).norm() < 1.0e-6 &&
            std::abs(left.first_derivative.yaw_rad -
                     right.first_derivative.yaw_rad) < 1.0e-6,
        "natural spline is not C1 at an internal knot");
    Require(
        (left.second_derivative.displacement -
         right.second_derivative.displacement).norm() < 1.0e-6 &&
            std::abs(left.second_derivative.yaw_rad -
                     right.second_derivative.yaw_rad) < 1.0e-6,
        "natural spline is not C2 at an internal knot");
  }

  const auto four_metres = field.EvaluateWithDerivatives(64.0);
  const auto horizon = field.EvaluateWithDerivatives(68.0);
  const auto held = field.EvaluateWithDerivatives(160.0);
  Require(
      (four_metres.correction.displacement.head<2>() -
       finish.correction.displacement.head<2>()).norm() <= 0.4000001 &&
          std::abs(four_metres.correction.displacement.z() -
                   finish.correction.displacement.z()) <= 0.2000001 &&
          std::abs(four_metres.correction.yaw_rad -
                   finish.correction.yaw_rad) <=
              0.800001 * kDegreesToRadians,
      "linear endpoint extrapolation exceeded a configured slope limit");
  Require(
      (held.correction.displacement -
       horizon.correction.displacement).norm() < 1.0e-12 &&
          std::abs(held.correction.yaw_rad -
                   horizon.correction.yaw_rad) < 1.0e-12 &&
          held.first_derivative.displacement.norm() < 1.0e-12,
      "correction field did not hold after its extrapolation horizon");
}

void TestAnchoredFieldAcceptsAttitudeOnlyStartupKnots()
{
  my_livo::backend::RegularizedCorrectionField4d::Options options;
  options.anchor_first_knot_identity = true;
  options.minimum_knot_time_interval_sec = 0.0;
  options.minimum_knot_spacing_m = 0.0;
  my_livo::backend::RegularizedCorrectionField4d field(options);

  my_livo::backend::Pose3d nominal;
  my_livo::backend::RtkObservation observation;
  observation.lower_ins_pos_mode = 4;
  observation.upper_ins_pos_mode = 4;
  observation.position_covariance = Eigen::Matrix3d::Identity() * 0.01;
  observation.timestamp = 1.0;
  observation.position = Eigen::Vector3d(2.0, -1.0, 0.5);
  field.AddObservation(0, 1.0, 0.0, nominal, observation, true);

  observation.timestamp = 2.0;
  observation.orientation = Eigen::AngleAxisd(
      0.02, Eigen::Vector3d::UnitZ());
  field.AddObservation(1, 2.0, 1.0, nominal, observation, false);
  const auto startup = field.Evaluate(1.0);
  Require(startup.displacement.norm() < 1.0e-12,
          "attitude-only startup changed anchored position");

  observation.timestamp = 3.0;
  observation.position = Eigen::Vector3d(2.2, -0.9, 0.5);
  field.AddObservation(2, 3.0, 2.0, nominal, observation, true);
  const auto observable = field.Evaluate(2.0);
  Require(observable.displacement.allFinite(),
          "position field stayed singular after slope became observable");
}

void TestSecondDifferenceRegularization()
{
  RegularizedCorrectionField4d::Options raw_options;
  raw_options.anchor_first_knot_identity = false;
  raw_options.position_second_difference_lambda = 0.0;
  RegularizedCorrectionField4d raw(raw_options);
  RegularizedCorrectionField4d::Options smooth_options = raw_options;
  smooth_options.position_second_difference_lambda = 10.0;
  RegularizedCorrectionField4d smooth(smooth_options);

  for (std::uint64_t index = 0; index < 12; ++index)
  {
    const double distance = 4.0 * static_cast<double>(index);
    const Pose3d nominal = Pose(distance, 0.0, 0.0, 0.0);
    const double noise = index % 2 == 0 ? 0.8 : -0.8;
    const Eigen::Vector3d position =
        nominal.translation + Eigen::Vector3d(0.01 * distance + noise, 0, 0);
    const RtkObservation observation = Observation(
        20.0 + static_cast<double>(index), position, 0.0);
    raw.AddObservation(
        index, observation.timestamp, distance, nominal, observation);
    smooth.AddObservation(
        index, observation.timestamp, distance, nominal, observation);
  }
  Require(
      SlopeVariation(smooth.knots()) < 0.5 * SlopeVariation(raw.knots()),
      "second-order slope regularization did not suppress knot noise");
}

void TestEndpointTrendRejectsLastSampleReversal()
{
  RegularizedCorrectionField4d::Options options;
  options.anchor_first_knot_identity = false;
  options.position_second_difference_lambda = 0.0;
  options.extrapolation_regression_knots = 3;
  options.maximum_planar_extrapolation_slope_m_per_m = 1.0;
  RegularizedCorrectionField4d field(options);
  const double targets[] = {0.0, 1.0, 0.5};
  for (std::uint64_t index = 0; index < 3; ++index)
  {
    const double distance = 10.0 * static_cast<double>(index);
    const Pose3d nominal = Pose(distance, 0.0, 0.0, 0.0);
    field.AddObservation(
        index, 50.0 + static_cast<double>(index), distance, nominal,
        Observation(
            50.0 + static_cast<double>(index),
            nominal.translation + Eigen::Vector3d(targets[index], 0.0, 0.0),
            0.0));
  }
  Require(
      field.Evaluate(25.0).displacement.x() > 0.5,
      "endpoint extrapolation followed one reversed sample instead of the "
      "short fitted trend");
}

void TestAttitudeOnlyRecoveryStartupHasTemporaryPositionGauge()
{
  RegularizedCorrectionField4d::Options options;
  options.anchor_first_knot_identity = false;
  RegularizedCorrectionField4d field(options);
  const Pose3d first_nominal = Pose(0.0, 0.0, 0.0, 0.0);
  const auto first = field.AddObservation(
      0, 70.0, 0.0, first_nominal,
      Observation(70.0, Eigen::Vector3d(4.0, 0.0, 0.0), 2.0), false);
  Require(
      first.fitted_correction.displacement.norm() < 1.0e-12 &&
          std::abs(first.fitted_correction.yaw_rad) > 1.0e-6,
      "attitude-only recovery startup did not isolate its temporary "
      "position gauge");
  const Pose3d second_nominal = Pose(4.0, 0.0, 0.0, 0.0);
  field.AddObservation(
      1, 71.0, 4.0, second_nominal,
      Observation(71.0, Eigen::Vector3d(4.5, 0.0, 0.0), 2.0), true);
  const Pose3d third_nominal = Pose(8.0, 0.0, 0.0, 0.0);
  const auto observable = field.AddObservation(
      2, 72.0, 8.0, third_nominal,
      Observation(72.0, Eigen::Vector3d(9.0, 0.0, 0.0), 2.0), true);
  Require(
      observable.fitted_correction.displacement.x() > 0.5,
      "recovery position gauge did not release after two observations");
}

void TestAttitudeYawIsNotPositionPathYaw()
{
  RegularizedCorrectionField4d::Options options;
  options.anchor_first_knot_identity = false;
  options.orientation_yaw_window_m = 16.0;
  RegularizedCorrectionField4d field(options);

  // Nominal positions run east and RTK positions run north, but both body
  // attitudes differ by only 10 degrees.  A path-yaw implementation would
  // incorrectly produce roughly 90 degrees here.
  for (std::uint64_t index = 0; index < 10; ++index)
  {
    const double distance = 4.0 * static_cast<double>(index);
    const double nominal_yaw = 175.0 + 2.0 * static_cast<double>(index);
    const Pose3d nominal = Pose(distance, 0.0, 0.0, nominal_yaw);
    field.AddObservation(
        index, 30.0 + static_cast<double>(index), distance, nominal,
        Observation(
            30.0 + static_cast<double>(index),
            Eigen::Vector3d(0.0, distance, 0.0), nominal_yaw + 10.0));
  }
  const double yaw_deg =
      field.Evaluate(36.0).yaw_rad / kDegreesToRadians;
  Require(
      std::abs(yaw_deg - 10.0) < 1.0e-8,
      "position-path yaw leaked into the body-attitude correction");
}

void TestCausalRigidElasticField()
{
  RegularizedCorrectionField4d::Options options;
  options.minimum_alignment_path_length_m = 10.0;
  options.alignment_window_length_m = 30.0;
  options.maximum_planar_gradient_m_per_m = 10.0;
  options.maximum_vertical_gradient_m_per_m = 10.0;
  options.maximum_yaw_gradient_deg_per_m = 10.0;
  RegularizedCorrectionField4d field(options);
  double previous_force = -1.0;
  for (int centimetres = 0; centimetres <= 100; ++centimetres)
  {
    const double distance = 0.01 * static_cast<double>(centimetres);
    const double force = field.ElasticStiffness(distance) * distance;
    Require(force + 1.0e-12 >= previous_force,
            "radial elastic force is not monotonic with distance");
    previous_force = force;
  }
  Require(
      std::abs(field.ElasticStiffness(0.09)) < 1.0e-12 &&
          field.ElasticStiffness(0.12) < 0.05 &&
          std::abs(field.ElasticStiffness(0.25) - 1.0) < 1.0e-12 &&
          field.ElasticStiffness(0.20) > field.ElasticStiffness(0.12),
      "elastic soft core or monotonic stiffness is incorrect");

  const double yaw = 5.0 * kDegreesToRadians;
  const Eigen::Matrix2d rotation =
      Eigen::Rotation2Dd(yaw).toRotationMatrix();
  const Eigen::Vector2d translation(2.0, -1.0);
  const auto global_position = [&](double distance) {
    const Eigen::Vector2d planar =
        rotation * Eigen::Vector2d(distance, 0.0) + translation;
    return Eigen::Vector3d(planar.x(), planar.y(), 0.2);
  };
  field.AddObservation(
      0, 1.0, 0.0, Pose(0.0, 0.0, 0.0, 0.0),
      Observation(1.0, global_position(0.0), 80.0));
  const auto attitude_only = field.AddObservation(
      1, 1.5, 2.0, Pose(2.0, 0.0, 0.0, 0.0),
      Observation(1.5, global_position(2.0), -120.0), false);
  Require(
      attitude_only.decision ==
          RegularizedCorrectionField4d::AddDecision::kNoPositionConstraint &&
          field.knot_count() == 1,
      "receiver attitude entered the production field");

  field.AddObservation(
      2, 2.0, 5.0, Pose(5.0, 0.0, 0.0, 0.0),
      Observation(2.0, global_position(5.0), -90.0));
  const auto frozen = field.Evaluate(2.5);
  for (std::uint64_t index = 3; index <= 5; ++index)
  {
    const double distance = 5.0 * static_cast<double>(index - 1);
    field.AddObservation(
        index, static_cast<double>(index), distance,
        Pose(distance, 0.0, 0.0, 0.0),
        Observation(static_cast<double>(index), global_position(distance),
                    140.0));
  }
  Require(
      (field.Evaluate(2.5).displacement - frozen.displacement).norm() <
          1.0e-12,
      "a new knot revised a completed history interval");
  Require(
      std::abs(field.Evaluate(20.0).yaw_rad - yaw) <
          0.5 * kDegreesToRadians,
      "long-baseline position geometry exceeded the yaw tolerance");
  const auto left = field.EvaluateWithDerivatives(10.0 - 1.0e-6);
  const auto right = field.EvaluateWithDerivatives(10.0 + 1.0e-6);
  Require(
      (left.first_derivative.displacement -
       right.first_derivative.displacement).norm() < 1.0e-5 &&
          (left.second_derivative.displacement -
           right.second_derivative.displacement).norm() < 1.0e-5,
      "causal quintic field is not C2 at a knot");

  RegularizedCorrectionField4d soft(options);
  soft.AddObservation(
      0, 1.0, 0.0, Pose(0.0, 0.0, 0.0, 0.0),
      Observation(1.0, Eigen::Vector3d(0.05, 0.0, 0.0), 0.0));
  soft.AddObservation(
      1, 2.0, 5.0, Pose(5.0, 0.0, 0.0, 0.0),
      Observation(2.0, Eigen::Vector3d(5.05, 0.0, 0.0), 90.0));
  Require(
      soft.Evaluate(5.0).displacement.norm() < 1.0e-12 &&
          std::abs(soft.Evaluate(5.0).yaw_rad) < 1.0e-12,
      "soft-core RTK noise modified the local trajectory");

  RegularizedCorrectionField4d::Options fallback_options = options;
  fallback_options.maximum_alignment_planar_rms_m = 0.01;
  RegularizedCorrectionField4d fallback(fallback_options);
  RegularizedCorrectionField4d::UpdateResult fallback_update;
  for (std::uint64_t index = 0; index < 4; ++index)
  {
    const double distance = 5.0 * static_cast<double>(index);
    const double cross_track = index % 2 == 0 ? 0.0 : 1.0;
    fallback_update = fallback.AddObservation(
        index, 10.0 + static_cast<double>(index), distance,
        Pose(distance, 0.0, 0.0, 0.0),
        Observation(10.0 + static_cast<double>(index),
                    Eigen::Vector3d(distance + 2.0, cross_track, 1.0), 0.0));
  }
  Require(
      fallback_update.decision ==
              RegularizedCorrectionField4d::AddDecision::
                  kTranslationFallback &&
          fallback_update.fitted_correction.displacement.norm() > 0.1,
      "non-rigid alignment disabled the far-distance restoring force");
  const auto fallback_knots = fallback.knots();
  Require(
      (fallback_knots.back().displacement_target -
       Eigen::Vector3d(2.0, 1.0, 1.0)).norm() < 1.0e-12,
      "degraded elastic target lagged behind the current radial error");
}

void TestAxisIndependentElasticFieldAndPeakAudit()
{
  RegularizedCorrectionField4d::Options options;
  options.anchor_first_knot_identity = false;
  options.maximum_planar_gradient_m_per_m = 10.0;
  options.maximum_vertical_gradient_m_per_m = 10.0;
  options.maximum_yaw_gradient_deg_per_m = 10.0;
  RegularizedCorrectionField4d field(options);

  const auto first = field.AddObservation(
      0, 1.0, 0.0, Pose(0.0, 0.0, 0.0, 0.0),
      Observation(1.0, Eigen::Vector3d(0.10, 0.0, 1.0), 0.0));
  Require(
      first.planar_elastic_stiffness < 1.0e-12 &&
          std::abs(first.vertical_elastic_stiffness - 1.0) < 1.0e-12 &&
          first.fitted_correction.displacement.head<2>().norm() < 1.0e-12 &&
          first.fitted_correction.displacement.z() > 0.9,
      "vertical drift incorrectly opened the planar elastic band");

  const auto second = field.AddObservation(
      1, 2.0, 5.0, Pose(5.0, 0.0, 0.0, 0.0),
      Observation(2.0, Eigen::Vector3d(5.10, 0.0, 2.0), 0.0));
  Require(
      second.planar_elastic_stiffness < 1.0e-12 &&
          second.interval_peak_planar_gradient_m_per_m < 1.0e-12 &&
          second.interval_peak_vertical_gradient_m_per_m > 0.1,
      "axis-specific interval peak gradients are incorrect");
  Require(
      field.YawElasticStiffness(0.24) < 1.0e-12 &&
          field.YawElasticStiffness(0.50) > 0.0 &&
          std::abs(field.YawElasticStiffness(1.0) - 1.0) < 1.0e-12,
      "yaw tolerance did not remain independent of position error");

  RegularizedCorrectionField4d nominal_gain(options);
  RegularizedCorrectionField4d weak_geometry_gain(options);
  const Pose3d gain_pose = Pose(0.0, 0.0, 0.0, 0.0);
  const RtkObservation gain_observation = Observation(
      3.0, Eigen::Vector3d(0.16, 0.0, 0.0), 0.0);
  const auto nominal_update = nominal_gain.AddObservation(
      0, 3.0, 0.0, gain_pose, gain_observation, true, 1.0);
  const auto weak_update = weak_geometry_gain.AddObservation(
      0, 3.0, 0.0, gain_pose, gain_observation, true, 3.0);
  Require(
      weak_update.planar_elastic_stiffness >
              nominal_update.planar_elastic_stiffness &&
          weak_update.fitted_correction.displacement.x() >
              nominal_update.fitted_correction.displacement.x() &&
          weak_update.fitted_correction.displacement.x() <= 0.16,
      "weak LIO geometry did not strengthen the bounded radial force");

  RegularizedCorrectionField4d::Options adaptive_options;
  RegularizedCorrectionField4d adaptive(adaptive_options);
  adaptive.AddObservation(
      0, 10.0, 0.0, Pose(0.0, 0.0, 0.0, 0.0),
      Observation(10.0, Eigen::Vector3d::Zero(), 0.0), true, 1.0);
  const auto far_update = adaptive.AddObservation(
      1, 11.0, 5.0, Pose(5.0, 0.0, 0.0, 0.0),
      Observation(11.0, Eigen::Vector3d(10.0, 0.0, 0.0), 0.0),
      true, 3.0);
  Require(
      std::abs(far_update.planar_gradient_gain-
               adaptive_options.adaptive_position_gradient_maximum_gain) <
              1.0e-12 &&
          far_update.interval_peak_planar_gradient_m_per_m >
              adaptive_options.maximum_planar_gradient_m_per_m &&
          far_update.interval_peak_planar_gradient_m_per_m <=
              adaptive_options.adaptive_position_gradient_maximum_gain *
                  adaptive_options.maximum_planar_gradient_m_per_m+
                  1.0e-12,
      "far elastic error did not open its causal gradient envelope");
}

}  // namespace

int main()
{
  try
  {
    TestCausalRigidElasticField();
    TestAxisIndependentElasticFieldAndPeakAudit();
  }
  catch (const std::exception &error)
  {
    std::cerr << "regularized_correction_field_4d_test failed: "
              << error.what() << '\n';
    return 1;
  }
  std::cout << "regularized_correction_field_4d_test passed\n";
  return 0;
}
