#include "backend/low_frequency_correction_field.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
using my_livo::backend::CorrectionFieldDecision;
using my_livo::backend::CorrectionRegime;
using my_livo::backend::LowFrequencyCorrectionField;
using my_livo::backend::Pose3d;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

LowFrequencyCorrectionField::UpdateResult Add(
    LowFrequencyCorrectionField &field, std::uint64_t id, double distance,
    const Eigen::Vector3d &rtk, CorrectionRegime regime)
{
  return field.AddObservation(
      id, 10.0 + static_cast<double>(id), distance,
      Eigen::Vector3d(distance, 0.0, 0.0), rtk,
      Eigen::Matrix3d::Identity() * 0.01, regime);
}
}  // namespace

int main()
{
  try
  {
    LowFrequencyCorrectionField::Options options;
    options.knot_spacing_m = 15.0;
    options.spatial_low_pass_length_m = 40.0;
    options.maximum_planar_gradient_m_per_m = 0.005;
    options.maximum_vertical_gradient_m_per_m = 0.003;
    options.maximum_yaw_gradient_deg_per_m = 0.01;
    options.maximum_planar_update_m = 0.05;
    options.maximum_vertical_update_m = 0.03;
    options.maximum_yaw_update_deg = 0.15;
    LowFrequencyCorrectionField field(options);
    Require(std::abs(field.ElasticStiffness(0.0) - 0.05) < 1.0e-12 &&
                std::abs(field.ElasticStiffness(0.19) - 0.525) < 1.0e-12 &&
                std::abs(field.ElasticStiffness(0.28) - 1.0) < 1.0e-12,
            "quintic radial stiffness endpoints/midpoint are incorrect");
    double previous_stiffness = field.ElasticStiffness(0.0);
    for (int sample = 1; sample <= 100; ++sample)
    {
      const double stiffness = field.ElasticStiffness(0.005 * sample);
      Require(stiffness + 1.0e-12 >= previous_stiffness,
              "radial elastic stiffness is not monotonic");
      previous_stiffness = stiffness;
    }

    const auto anchor = Add(
        field, 0, 0.0, Eigen::Vector3d::Zero(),
        CorrectionRegime::kWarmup);
    Require(anchor.decision == CorrectionFieldDecision::kAnchor,
            "first correction knot was not anchored");
    Require(anchor.correction.displacement.norm() < 1.0e-12,
            "C(s0) is not identity");
    Require(!anchor.field_changed,
            "identity anchor incorrectly reported an output change");

    const auto spacing = Add(
        field, 1, 7.5, Eigen::Vector3d(8.5, 0.5, 0.5),
        CorrectionRegime::kWarmup);
    Require(spacing.decision == CorrectionFieldDecision::kWarmup,
            "monitor warmup was allowed to modify the field");

    const auto update = Add(
        field, 2, 15.0, Eigen::Vector3d(16.0, 1.0, 1.0),
        CorrectionRegime::kElastic);
    Require(update.decision == CorrectionFieldDecision::kUpdated,
            "eligible correction knot was not added");
    Require(std::abs(update.elastic_distance_m - std::sqrt(3.0)) < 1.0e-12 &&
                std::abs(update.elastic_stiffness - 1.0) < 1.0e-12 &&
                std::abs(update.radial_force_proxy_m - std::sqrt(3.0)) <
                    1.0e-12,
            "radial elastic force did not use corrected global distance");
    Require(update.planar_step_m <= 0.05 + 1.0e-12 &&
                update.vertical_step_m <= 0.03 + 1.0e-12 &&
                update.yaw_step_deg <= 0.10 + 1.0e-12,
            "per-knot correction bounds were violated");

    const auto midpoint = field.Evaluate(7.5);
    Require((midpoint.displacement -
             0.5 * update.correction.displacement).norm() < 1.0e-12,
            "cubic midpoint interpolation is inconsistent");
    const Pose3d base(Eigen::Quaterniond::Identity(),
                      Eigen::Vector3d(7.5, 0.0, 0.0));
    const Pose3d corrected = field.Apply(7.5, base);
    Require((corrected.translation -
             (base.translation + midpoint.displacement)).norm() < 1.0e-12,
            "correction changed the base pose instead of the global output");

    double maximum_sampled_planar_gradient = 0.0;
    double maximum_sampled_vertical_gradient = 0.0;
    auto previous = field.Evaluate(0.0);
    for (int sample = 1; sample <= 1500; ++sample)
    {
      const double distance = 0.01 * sample;
      const auto current = field.Evaluate(distance);
      maximum_sampled_planar_gradient = std::max(
          maximum_sampled_planar_gradient,
          (current.displacement.head<2>() -
           previous.displacement.head<2>()).norm() / 0.01);
      maximum_sampled_vertical_gradient = std::max(
          maximum_sampled_vertical_gradient,
          std::abs(current.displacement.z() - previous.displacement.z()) /
              0.01);
      previous = current;
    }
    Require(maximum_sampled_planar_gradient <= 0.00501 &&
                maximum_sampled_vertical_gradient <= 0.00301,
            "cubic interpolation exceeded its spatial derivative bound");

    const auto frozen = Add(
        field, 3, 30.0, Eigen::Vector3d(35.0, 5.0, 5.0),
        CorrectionRegime::kRelocalizationRequired);
    Require(frozen.decision ==
                CorrectionFieldDecision::kFrozenRelocalization &&
                frozen.frozen && !frozen.field_changed,
            "relocalization did not freeze the elastic field");
    const auto held = field.Evaluate(100.0);
    Require((held.displacement - update.correction.displacement).norm() <
                1.0e-12,
            "frozen correction field changed after relocalization latch");
  }
  catch (const std::exception &error)
  {
    std::cerr << "low_frequency_correction_field_test failed: "
              << error.what() << '\n';
    return 1;
  }
  std::cout << "low_frequency_correction_field_test passed\n";
  return 0;
}
