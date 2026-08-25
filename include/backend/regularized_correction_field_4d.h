#ifndef MY_LIVO_BACKEND_REGULARIZED_CORRECTION_FIELD_4D_H
#define MY_LIVO_BACKEND_REGULARIZED_CORRECTION_FIELD_4D_H

#include "backend/keyframe.h"
#include "backend/rtk_observation_buffer.h"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace my_livo::backend
{

// A causal 4-DOF correction field along travelled distance. Status-4 RTK
// positions generate robust no-scale sliding-window rigid targets; receiver
// attitude is deliberately excluded. Completed intervals are immutable and a
// quintic transition gives C2 continuity without globally refitting history.
class RegularizedCorrectionField4d
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options
  {
    // The caller normally supplies already status-gated observations.  Keep
    // this guard enabled to make accidental use of non-mode-4 data explicit.
    bool require_status4 = true;
    int required_ins_pos_mode = 4;

    // Input is expected at about 1 Hz (normally 3--6 m between samples).
    // Positive values may be used to thin a higher-rate stream.
    double minimum_knot_time_interval_sec = 0.0;
    double minimum_knot_spacing_m = 0.0;

    double position_observation_weight = 1.0;
    double orientation_yaw_observation_weight = 1.0;
    double position_second_difference_lambda = 10.0;
    double orientation_yaw_second_difference_lambda = 10.0;
    double minimum_position_sigma_m = 0.10;

    // Before this baseline is accumulated, attitude-yaw observations remain
    // soft.  They are never replaced by position-path heading.
    double orientation_yaw_window_m = 30.0;
    double minimum_orientation_yaw_confidence = 0.05;

    double elastic_soft_radius_m = 0.10;
    double elastic_full_radius_m = 0.25;
    double elastic_minimum_stiffness = 0.0;
    double elastic_maximum_stiffness = 1.0;
    double vertical_elastic_soft_radius_m = 0.10;
    double vertical_elastic_full_radius_m = 0.30;
    double yaw_elastic_soft_radius_deg = 0.25;
    double yaw_elastic_full_radius_deg = 1.00;

    // RTK only estimates a low-frequency rigid xyz+yaw alignment. Before the
    // yaw baseline becomes observable, the previous yaw is retained and only
    // a robust translation is estimated.
    double alignment_window_length_m = 30.0;
    double minimum_alignment_path_length_m = 10.0;
    double alignment_huber_delta_m = 0.15;
    double maximum_alignment_planar_rms_m = 0.30;

    // Bound the endpoint correction requested by one new low-rate knot. The
    // quintic interpolation has a peak derivative of 1.875 times its endpoint
    // delta divided by spacing.
    double maximum_planar_gradient_m_per_m = 0.05;
    double maximum_vertical_gradient_m_per_m = 0.08;
    double maximum_yaw_gradient_deg_per_m = 0.10;
    // The near-field limits above remain unchanged. Once an error is beyond
    // the ordinary elastic band, the admissible causal slope increases
    // smoothly so the implemented pull, not only its diagnostic force proxy,
    // becomes stronger with distance. Completed intervals stay immutable.
    bool adaptive_gradient_enabled = true;
    double adaptive_position_gradient_full_distance_m = 1.50;
    double adaptive_yaw_gradient_full_distance_deg = 3.00;
    double adaptive_position_gradient_maximum_gain = 1.5;
    double adaptive_yaw_gradient_maximum_gain = 2.0;

    // Production segments use exact C(s0)=I so a recovery gate cannot
    // teleport its boundary. The false option is retained for isolated
    // analysis/tests, not for a fitted recovery pose command.
    bool anchor_first_knot_identity = true;

    // Past the newest knot, continue with the natural-spline endpoint slope,
    // replaced by a short robust linear trend to avoid cubic end overshoot,
    // subject to these spatial limits, then hold after the horizon.
    int extrapolation_regression_knots = 3;
    double maximum_extrapolation_distance_m = 8.0;
    double maximum_planar_extrapolation_slope_m_per_m = 0.10;
    double maximum_vertical_extrapolation_slope_m_per_m = 0.05;
    double maximum_yaw_extrapolation_slope_deg_per_m = 0.20;
  };

  enum class AddDecision
  {
    kAccepted,
    kStatusRejected,
    kSpacingRejected,
    kNoPositionConstraint,
    kTranslationFallback,
    kAlignmentRejected,
  };

  struct Correction
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
    double yaw_rad = 0.0;
  };

  struct Evaluation
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Correction correction;
    Correction first_derivative;
    Correction second_derivative;
    bool extrapolated = false;
  };

  struct UpdateResult
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AddDecision decision = AddDecision::kAccepted;
    std::size_t knot_count = 0;
    double elastic_distance_m = 0.0;
    double elastic_stiffness = 0.0;
    double orientation_yaw_confidence = 0.0;
    bool position_observation_used = true;
    double alignment_path_length_m = 0.0;
    double alignment_planar_rms_m = 0.0;
    double planar_elastic_distance_m = 0.0;
    double vertical_elastic_distance_m = 0.0;
    double yaw_elastic_distance_deg = 0.0;
    double planar_elastic_stiffness = 0.0;
    double vertical_elastic_stiffness = 0.0;
    double yaw_elastic_stiffness = 0.0;
    double interval_peak_planar_gradient_m_per_m = 0.0;
    double interval_peak_vertical_gradient_m_per_m = 0.0;
    double interval_peak_yaw_gradient_deg_per_m = 0.0;
    double planar_gradient_gain = 1.0;
    double vertical_gradient_gain = 1.0;
    double yaw_gradient_gain = 1.0;
    double yaw_residual_after_deg = 0.0;
    double constraint_gain = 1.0;
    Correction fitted_correction;
  };

  struct KnotState
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    double timestamp = 0.0;
    double cumulative_distance_m = 0.0;
    Eigen::Vector3d displacement_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d nominal_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d rtk_position = Eigen::Vector3d::Zero();
    // Stored unwrapped so the least-squares problem is continuous at +/-pi.
    double orientation_yaw_target_rad = 0.0;
    double orientation_yaw_confidence = 0.0;
    double elastic_stiffness = 0.0;
    Correction fitted_correction;
  };

  RegularizedCorrectionField4d();
  explicit RegularizedCorrectionField4d(const Options &options);

  UpdateResult AddObservation(
      std::uint64_t keyframe_id, double timestamp,
      double cumulative_distance_m, const Pose3d &nominal_pose,
      const RtkObservation &status4_observation,
      bool use_position_observation = true,
      double constraint_gain = 1.0);

  Correction Evaluate(double cumulative_distance_m) const;
  Evaluation EvaluateWithDerivatives(double cumulative_distance_m) const;
  Pose3d Apply(double cumulative_distance_m,
               const Pose3d &nominal_pose) const;

  double ElasticStiffness(double distance_m) const;
  double VerticalElasticStiffness(double distance_m) const;
  double YawElasticStiffness(double distance_deg) const;
  std::size_t knot_count() const { return knots_.size(); }
  std::vector<KnotState, Eigen::aligned_allocator<KnotState>> knots() const;

private:
  struct Knot
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    double timestamp = 0.0;
    double cumulative_distance_m = 0.0;
    Eigen::Vector3d displacement_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d nominal_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d rtk_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_data_weight = Eigen::Vector3d::Zero();
    double raw_orientation_yaw_target_rad = 0.0;
    double orientation_yaw_target_rad = 0.0;
    double orientation_yaw_confidence = 0.0;
    double elastic_stiffness = 0.0;
    Eigen::Matrix<double, 4, 1> fitted =
        Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix<double, 4, 1> spline_second_derivative =
        Eigen::Matrix<double, 4, 1>::Zero();
  };

  static void ValidateOptions(const Options &options);
  static double Yaw(const Eigen::Quaterniond &rotation);
  static double WrapRadians(double angle);
  static double UnwrapNear(double angle, double reference);
  double SmoothElasticStiffness(double distance, double soft_radius,
                                double full_radius) const;
  double AdaptiveGradientGain(
      double distance, double elastic_full_radius,
      double adaptive_full_distance, double maximum_gain,
      double elastic_stiffness, double constraint_gain) const;

  void Solve();
  Eigen::VectorXd SolveComponent(int component) const;
  void BuildNaturalSplineSecondDerivatives();
  double LongWindowOrientationYawTarget(
      double cumulative_distance_m, double raw_yaw_target,
      double *confidence) const;

  Options options_;
  std::vector<Knot, Eigen::aligned_allocator<Knot>> knots_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_REGULARIZED_CORRECTION_FIELD_4D_H
