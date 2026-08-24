#include "backend/regularized_correction_field_4d.h"

#include <Eigen/SparseCholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace my_livo::backend
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

double QuinticSmoothstep(double value)
{
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

Eigen::Vector2d LimitNorm(const Eigen::Vector2d &value, double limit)
{
  const double norm = value.norm();
  if (norm <= limit || norm <= std::numeric_limits<double>::epsilon())
    return value;
  return value * (limit / norm);
}
}  // namespace

RegularizedCorrectionField4d::RegularizedCorrectionField4d()
    : RegularizedCorrectionField4d(Options())
{
}

RegularizedCorrectionField4d::RegularizedCorrectionField4d(
    const Options &options)
    : options_(options)
{
  ValidateOptions(options_);
}

void RegularizedCorrectionField4d::ValidateOptions(const Options &options)
{
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  const auto nonnegative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (options.required_ins_pos_mode < 0 ||
      !nonnegative(options.minimum_knot_time_interval_sec) ||
      !nonnegative(options.minimum_knot_spacing_m) ||
      !positive(options.position_observation_weight) ||
      !positive(options.orientation_yaw_observation_weight) ||
      !nonnegative(options.position_second_difference_lambda) ||
      !nonnegative(options.orientation_yaw_second_difference_lambda) ||
      !positive(options.minimum_position_sigma_m) ||
      !positive(options.orientation_yaw_window_m) ||
      !positive(options.minimum_orientation_yaw_confidence) ||
      options.minimum_orientation_yaw_confidence > 1.0 ||
      !nonnegative(options.elastic_soft_radius_m) ||
      !positive(options.elastic_full_radius_m) ||
      options.elastic_full_radius_m <= options.elastic_soft_radius_m ||
      !nonnegative(options.elastic_minimum_stiffness) ||
      !positive(options.elastic_maximum_stiffness) ||
      options.elastic_maximum_stiffness <
          options.elastic_minimum_stiffness ||
      options.elastic_maximum_stiffness > 1.0 ||
      !positive(options.alignment_window_length_m) ||
      !positive(options.minimum_alignment_path_length_m) ||
      options.minimum_alignment_path_length_m >
          options.alignment_window_length_m ||
      !positive(options.alignment_huber_delta_m) ||
      !positive(options.maximum_alignment_planar_rms_m) ||
      !positive(options.maximum_planar_gradient_m_per_m) ||
      !positive(options.maximum_vertical_gradient_m_per_m) ||
      !positive(options.maximum_yaw_gradient_deg_per_m) ||
      options.extrapolation_regression_knots < 2 ||
      !nonnegative(options.maximum_extrapolation_distance_m) ||
      !positive(options.maximum_planar_extrapolation_slope_m_per_m) ||
      !positive(options.maximum_vertical_extrapolation_slope_m_per_m) ||
      !positive(options.maximum_yaw_extrapolation_slope_deg_per_m))
    throw std::invalid_argument(
        "Regularized 4-DOF correction-field options are invalid.");
}

double RegularizedCorrectionField4d::Yaw(
    const Eigen::Quaterniond &rotation)
{
  const Eigen::Matrix3d matrix = rotation.normalized().toRotationMatrix();
  return std::atan2(matrix(1, 0), matrix(0, 0));
}

double RegularizedCorrectionField4d::WrapRadians(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double RegularizedCorrectionField4d::UnwrapNear(
    double angle, double reference)
{
  return reference + WrapRadians(angle - reference);
}

double RegularizedCorrectionField4d::ElasticStiffness(
    double distance_m) const
{
  if (!std::isfinite(distance_m) || distance_m < 0.0)
    throw std::invalid_argument(
        "Elastic distance must be finite and non-negative.");
  const double normalized =
      (distance_m - options_.elastic_soft_radius_m) /
      (options_.elastic_full_radius_m - options_.elastic_soft_radius_m);
  return options_.elastic_minimum_stiffness +
      (options_.elastic_maximum_stiffness -
       options_.elastic_minimum_stiffness) *
          QuinticSmoothstep(normalized);
}

double RegularizedCorrectionField4d::LongWindowOrientationYawTarget(
    double cumulative_distance_m, double raw_yaw_target,
    double *confidence) const
{
  const double begin =
      cumulative_distance_m - options_.orientation_yaw_window_m;
  double sum = raw_yaw_target;
  std::size_t samples = 1;
  double oldest_distance = cumulative_distance_m;
  for (auto iterator = knots_.rbegin(); iterator != knots_.rend(); ++iterator)
  {
    if (iterator->cumulative_distance_m + 1.0e-12 < begin) break;
    sum += iterator->raw_orientation_yaw_target_rad;
    ++samples;
    oldest_distance = iterator->cumulative_distance_m;
  }
  const double baseline = cumulative_distance_m - oldest_distance;
  *confidence = std::max(
      options_.minimum_orientation_yaw_confidence,
      std::clamp(baseline / options_.orientation_yaw_window_m, 0.0, 1.0));
  return sum / static_cast<double>(samples);
}

RegularizedCorrectionField4d::UpdateResult
RegularizedCorrectionField4d::AddObservation(
    std::uint64_t keyframe_id, double timestamp,
    double cumulative_distance_m, const Pose3d &nominal_pose,
    const RtkObservation &status4_observation,
    bool use_position_observation)
{
  if (!std::isfinite(timestamp) || !std::isfinite(cumulative_distance_m) ||
      cumulative_distance_m < 0.0 || !nominal_pose.isFinite() ||
      !std::isfinite(status4_observation.timestamp) ||
      !status4_observation.position.allFinite() ||
      !status4_observation.orientation.coeffs().allFinite() ||
      status4_observation.orientation.norm() <= 1.0e-9 ||
      !status4_observation.position_covariance.allFinite())
    throw std::invalid_argument(
        "Regularized correction-field observation is invalid.");

  UpdateResult result;
  result.position_observation_used = use_position_observation;
  result.knot_count = knots_.size();
  if (options_.require_status4 &&
      (status4_observation.lower_ins_pos_mode !=
           options_.required_ins_pos_mode ||
       status4_observation.upper_ins_pos_mode !=
           options_.required_ins_pos_mode))
  {
    result.decision = AddDecision::kStatusRejected;
    return result;
  }

  // Receiver attitude is not an input to the production field.  Keyframe-rate
  // calls remain observable in the audit log but only the independent low-rate
  // status-4 position cadence may create a correction knot.
  if (!use_position_observation)
  {
    result.decision = AddDecision::kNoPositionConstraint;
    result.fitted_correction = Evaluate(cumulative_distance_m);
    return result;
  }

  if (!knots_.empty())
  {
    const Knot &last = knots_.back();
    if (keyframe_id <= last.keyframe_id || timestamp <= last.timestamp ||
        cumulative_distance_m <= last.cumulative_distance_m)
      throw std::logic_error(
          "Regularized correction-field inputs must be strictly ordered.");
    if (timestamp - last.timestamp + 1.0e-12 <
            options_.minimum_knot_time_interval_sec ||
        cumulative_distance_m - last.cumulative_distance_m + 1.0e-12 <
            options_.minimum_knot_spacing_m)
    {
      result.decision = AddDecision::kSpacingRejected;
      return result;
    }
  }

  Knot knot;
  knot.keyframe_id = keyframe_id;
  knot.timestamp = timestamp;
  knot.cumulative_distance_m = cumulative_distance_m;
  knot.nominal_position = nominal_pose.translation;
  knot.rtk_position = status4_observation.position;

  const Correction prediction = Evaluate(cumulative_distance_m);
  const double window_begin =
      cumulative_distance_m - options_.alignment_window_length_m;
  std::vector<const Knot *> samples;
  for (const Knot &sample : knots_)
    if (sample.cumulative_distance_m + 1.0e-12 >= window_begin)
      samples.push_back(&sample);
  samples.push_back(&knot);
  result.alignment_path_length_m = samples.size() > 1
      ? cumulative_distance_m - samples.front()->cumulative_distance_m
      : 0.0;

  double fitted_yaw = knots_.empty() ? 0.0 : knots_.back().fitted[3];
  Eigen::Vector3d fitted_translation = Eigen::Vector3d::Zero();
  std::vector<double> weights(samples.size(), 1.0);
  double planar_rms_m = 0.0;
  for (int iteration = 0; iteration < 4; ++iteration)
  {
    double weight_sum = 0.0;
    Eigen::Vector3d nominal_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d rtk_centroid = Eigen::Vector3d::Zero();
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
      weight_sum += weights[index];
      nominal_centroid +=
          weights[index] * samples[index]->nominal_position;
      rtk_centroid += weights[index] * samples[index]->rtk_position;
    }
    nominal_centroid /= weight_sum;
    rtk_centroid /= weight_sum;
    if (result.alignment_path_length_m + 1.0e-12 >=
            options_.minimum_alignment_path_length_m &&
        samples.size() >= 3U)
    {
      double dot = 0.0;
      double cross = 0.0;
      for (std::size_t index = 0; index < samples.size(); ++index)
      {
        const Eigen::Vector2d local =
            (samples[index]->nominal_position - nominal_centroid).head<2>();
        const Eigen::Vector2d global =
            (samples[index]->rtk_position - rtk_centroid).head<2>();
        dot += weights[index] * local.dot(global);
        cross += weights[index] *
            (local.x() * global.y() - local.y() * global.x());
      }
      if (std::hypot(dot, cross) > 1.0e-9)
        fitted_yaw = std::atan2(cross, dot);
    }
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(fitted_yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    fitted_translation = rtk_centroid - rotation * nominal_centroid;
    double squared_error = 0.0;
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
      const double residual =
          (rotation * samples[index]->nominal_position +
           fitted_translation - samples[index]->rtk_position)
              .head<2>().norm();
      squared_error += residual * residual;
      weights[index] = residual <= options_.alignment_huber_delta_m
          ? 1.0 : options_.alignment_huber_delta_m / residual;
    }
    planar_rms_m = std::sqrt(
        squared_error / static_cast<double>(samples.size()));
  }
  bool translation_fallback = planar_rms_m >
          options_.maximum_alignment_planar_rms_m &&
      result.alignment_path_length_m >=
          options_.minimum_alignment_path_length_m;
  if (translation_fallback)
  {
    // A non-rigid/weak planar window must not disable the radial restoring
    // force. Keep the last observable yaw and robustly estimate translation
    // only; this remains low-frequency and cannot import receiver attitude.
    fitted_yaw = knots_.empty() ? 0.0 : knots_.back().fitted[3];
    const Eigen::Matrix3d fallback_rotation =
        Eigen::AngleAxisd(fitted_yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    std::fill(weights.begin(), weights.end(), 1.0);
    for (int iteration = 0; iteration < 4; ++iteration)
    {
      double weight_sum = 0.0;
      fitted_translation.setZero();
      for (std::size_t index = 0; index < samples.size(); ++index)
      {
        weight_sum += weights[index];
        fitted_translation += weights[index] *
            (samples[index]->rtk_position -
             fallback_rotation * samples[index]->nominal_position);
      }
      fitted_translation /= weight_sum;
      double squared_error = 0.0;
      for (std::size_t index = 0; index < samples.size(); ++index)
      {
        const double residual =
            (fallback_rotation * samples[index]->nominal_position +
             fitted_translation - samples[index]->rtk_position)
                .head<2>().norm();
        squared_error += residual * residual;
        weights[index] = residual <= options_.alignment_huber_delta_m
            ? 1.0 : options_.alignment_huber_delta_m / residual;
      }
      planar_rms_m = std::sqrt(
          squared_error / static_cast<double>(samples.size()));
    }
  }
  result.alignment_planar_rms_m = planar_rms_m;

  const Eigen::Matrix3d fitted_rotation =
      Eigen::AngleAxisd(fitted_yaw, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  if (translation_fallback)
  {
    // A failed rigid fit is evidence that the required correction varies
    // across the window (for example scale/velocity drift). A centroid-only
    // translation lags the newest point by roughly half a window and made the
    // far elastic force ineffective. In this degraded branch the radial
    // target is therefore the current time-corresponding RTK point. The
    // distance deadband, low-rate cadence and C2 gradient envelope still
    // prevent its local noise from becoming keyframe-rate map motion.
    knot.displacement_target =
        status4_observation.position - nominal_pose.translation;
  }
  else
  {
    knot.displacement_target =
        fitted_rotation * nominal_pose.translation + fitted_translation -
        nominal_pose.translation;
  }
  knot.raw_orientation_yaw_target_rad = fitted_yaw;
  knot.orientation_yaw_target_rad = fitted_yaw;
  knot.orientation_yaw_confidence =
      std::clamp(result.alignment_path_length_m /
                     options_.minimum_alignment_path_length_m,
                 0.0, 1.0);
  result.orientation_yaw_confidence = knot.orientation_yaw_confidence;

  const Eigen::Vector3d current_rtk_error =
      nominal_pose.translation + prediction.displacement -
      status4_observation.position;
  result.elastic_distance_m = current_rtk_error.norm();
  result.elastic_stiffness = ElasticStiffness(result.elastic_distance_m);
  knot.elastic_stiffness = result.elastic_stiffness;
  knot.position_data_weight.setOnes();

  Eigen::Matrix<double, 4, 1> target;
  target.head<3>() = prediction.displacement + result.elastic_stiffness *
      (knot.displacement_target - prediction.displacement);
  target[3] = prediction.yaw_rad + result.elastic_stiffness *
      WrapRadians(fitted_yaw - prediction.yaw_rad);
  if (knots_.empty())
  {
    target.setZero();
  }
  else
  {
    const double spacing = cumulative_distance_m -
        knots_.back().cumulative_distance_m;
    // The same alpha(d) that defines the elastic force also opens the spatial
    // release envelope. Thus far-away states can actually respond more
    // strongly, while the inner tolerance cannot be moved by a large cap.
    const double release = result.elastic_stiffness;
    Eigen::Vector3d delta = target.head<3>() - knots_.back().fitted.head<3>();
    delta.head<2>() = LimitNorm(
        delta.head<2>(),
        release * options_.maximum_planar_gradient_m_per_m *
            spacing / 1.875);
    delta.z() = std::clamp(
        delta.z(),
        -release * options_.maximum_vertical_gradient_m_per_m *
            spacing / 1.875,
        release * options_.maximum_vertical_gradient_m_per_m *
            spacing / 1.875);
    target.head<3>() = knots_.back().fitted.head<3>() + delta;
    const double yaw_limit = release *
        options_.maximum_yaw_gradient_deg_per_m * kDegreesToRadians *
        spacing / 1.875;
    target[3] = knots_.back().fitted[3] + std::clamp(
        WrapRadians(target[3] - knots_.back().fitted[3]),
        -yaw_limit, yaw_limit);
  }
  knot.fitted = target;
  knot.spline_second_derivative.setZero();
  knots_.push_back(knot);
  result.knot_count = knots_.size();
  result.fitted_correction.displacement = target.head<3>();
  result.fitted_correction.yaw_rad = WrapRadians(target[3]);
  result.decision = translation_fallback
      ? AddDecision::kTranslationFallback : AddDecision::kAccepted;
  return result;
}

Eigen::VectorXd RegularizedCorrectionField4d::SolveComponent(
    int component) const
{
  const Eigen::Index count = static_cast<Eigen::Index>(knots_.size());
  using SparseMatrix = Eigen::SparseMatrix<double>;
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(static_cast<std::size_t>(count) * 10U);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(count);
  std::size_t position_constraints = 0;
  if (component < 3)
  {
    for (const Knot &knot : knots_)
      position_constraints += knot.position_data_weight[component] > 0.0;
  }
  // A recovery field is normally free at its first knot.  Before two sparse
  // position observations arrive, however, position-only affine modes are
  // unobservable while attitude-only knots are already valid.  A temporary
  // zero-displacement gauge makes that startup deterministic and is released
  // automatically once position translation and slope are observable.
  const bool constrain_first = options_.anchor_first_knot_identity ||
      (component < 3 && position_constraints < 2U);
  bool has_position_constraint_after_first = false;
  if (component < 3)
  {
    for (std::size_t index = 1; index < knots_.size(); ++index)
    {
      if (knots_[index].position_data_weight[component] > 0.0)
      {
        has_position_constraint_after_first = true;
        break;
      }
    }
  }
  // With only the first position fixed, a second-difference prior still has
  // an unobservable affine slope.  This occurs naturally because attitude is
  // sampled at keyframe rate while position is admitted at about 1 Hz.  Hold
  // the first interval flat only until a later position observation makes the
  // slope observable; then remove this startup gauge completely.
  const bool constrain_second = component < 3 && count > 1 &&
      constrain_first && !has_position_constraint_after_first;
  double gauge_value = 0.0;
  if (!options_.anchor_first_knot_identity && component < 3 &&
      position_constraints == 1U)
  {
    const auto position_knot = std::find_if(
        knots_.begin(), knots_.end(), [component](const Knot &knot) {
          return knot.position_data_weight[component] > 0.0;
        });
    gauge_value = position_knot->displacement_target[component];
  }

  for (Eigen::Index index = 0; index < count; ++index)
  {
    const Knot &knot = knots_[static_cast<std::size_t>(index)];
    const double weight = component < 3
        ? knot.position_data_weight[component]
        : options_.orientation_yaw_observation_weight *
              knot.elastic_stiffness * knot.orientation_yaw_confidence;
    const double target = component < 3
        ? knot.displacement_target[component]
        : knot.orientation_yaw_target_rad;
    if (!(constrain_first && index == 0) &&
        !(constrain_second && index == 1))
    {
      entries.emplace_back(index, index, weight);
      rhs[index] += weight * (target - gauge_value);
    }
  }

  const double lambda = component < 3
      ? options_.position_second_difference_lambda
      : options_.orientation_yaw_second_difference_lambda;
  for (Eigen::Index index = 1; index + 1 < count; ++index)
  {
    const double previous_spacing =
        knots_[static_cast<std::size_t>(index)].cumulative_distance_m -
        knots_[static_cast<std::size_t>(index - 1)].cumulative_distance_m;
    const double next_spacing =
        knots_[static_cast<std::size_t>(index + 1)].cumulative_distance_m -
        knots_[static_cast<std::size_t>(index)].cumulative_distance_m;
    Eigen::Vector3d coefficients;
    coefficients << -1.0 / previous_spacing,
        1.0 / previous_spacing + 1.0 / next_spacing,
        -1.0 / next_spacing;
    for (int left = 0; left < 3; ++left)
    {
      const Eigen::Index left_index = index - 1 + left;
      if ((constrain_first && left_index == 0) ||
          (constrain_second && left_index == 1))
        continue;
      for (int right = 0; right < 3; ++right)
      {
        const Eigen::Index right_index = index - 1 + right;
        if ((constrain_first && right_index == 0) ||
            (constrain_second && right_index == 1))
          continue;
        entries.emplace_back(
            left_index, right_index,
            lambda * coefficients[left] * coefficients[right]);
      }
    }
  }

  if (constrain_first && count > 0)
  {
    // Eliminate C(s0) exactly.  All entries involving column/row zero were
    // omitted above; its prescribed value is zero, so no RHS adjustment is
    // required.
    entries.emplace_back(0, 0, 1.0);
    rhs[0] = 0.0;
  }
  if (constrain_second)
  {
    entries.emplace_back(1, 1, 1.0);
    rhs[1] = 0.0;
  }

  SparseMatrix normal(count, count);
  normal.setFromTriplets(entries.begin(), entries.end());
  normal.makeCompressed();
  Eigen::SimplicialLDLT<SparseMatrix> decomposition;
  decomposition.compute(normal);
  if (decomposition.info() != Eigen::Success)
    throw std::runtime_error(
        "Regularized correction-field normal matrix is not solvable.");
  const Eigen::VectorXd solution = decomposition.solve(rhs);
  if (decomposition.info() != Eigen::Success || !solution.allFinite())
    throw std::runtime_error(
        "Regularized correction-field solve produced invalid values.");
  if (constrain_first)
    return (solution.array() + gauge_value).matrix();
  return solution;
}

void RegularizedCorrectionField4d::Solve()
{
  if (knots_.empty()) return;
  for (int component = 0; component < 4; ++component)
  {
    const Eigen::VectorXd solution = SolveComponent(component);
    for (Eigen::Index index = 0; index < solution.size(); ++index)
      knots_[static_cast<std::size_t>(index)].fitted[component] =
          solution[index];
  }
  BuildNaturalSplineSecondDerivatives();
}

void RegularizedCorrectionField4d::BuildNaturalSplineSecondDerivatives()
{
  for (Knot &knot : knots_) knot.spline_second_derivative.setZero();
  if (knots_.size() < 3) return;

  const Eigen::Index interior_count =
      static_cast<Eigen::Index>(knots_.size() - 2);
  using SparseMatrix = Eigen::SparseMatrix<double>;
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(static_cast<std::size_t>(interior_count) * 3U);
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(interior_count, 4);
  for (Eigen::Index row = 0; row < interior_count; ++row)
  {
    const std::size_t index = static_cast<std::size_t>(row + 1);
    const double previous_spacing =
        knots_[index].cumulative_distance_m -
        knots_[index - 1].cumulative_distance_m;
    const double next_spacing =
        knots_[index + 1].cumulative_distance_m -
        knots_[index].cumulative_distance_m;
    entries.emplace_back(
        row, row, 2.0 * (previous_spacing + next_spacing));
    if (row > 0) entries.emplace_back(row, row - 1, previous_spacing);
    if (row + 1 < interior_count)
      entries.emplace_back(row, row + 1, next_spacing);
    rhs.row(row) = 6.0 * (
        (knots_[index + 1].fitted - knots_[index].fitted) /
            next_spacing -
        (knots_[index].fitted - knots_[index - 1].fitted) /
            previous_spacing).transpose();
  }

  SparseMatrix system(interior_count, interior_count);
  system.setFromTriplets(entries.begin(), entries.end());
  system.makeCompressed();
  Eigen::SimplicialLDLT<SparseMatrix> decomposition;
  decomposition.compute(system);
  if (decomposition.info() != Eigen::Success)
    throw std::runtime_error(
        "Natural correction spline system is not solvable.");
  const Eigen::MatrixXd second = decomposition.solve(rhs);
  if (decomposition.info() != Eigen::Success || !second.allFinite())
    throw std::runtime_error(
        "Natural correction spline produced invalid derivatives.");
  for (Eigen::Index row = 0; row < interior_count; ++row)
    knots_[static_cast<std::size_t>(row + 1)].spline_second_derivative =
        second.row(row).transpose();
}

RegularizedCorrectionField4d::Evaluation
RegularizedCorrectionField4d::EvaluateWithDerivatives(
    double cumulative_distance_m) const
{
  if (!std::isfinite(cumulative_distance_m))
    throw std::invalid_argument(
        "Regularized correction-field query must be finite.");
  Evaluation result;
  if (knots_.empty()) return result;

  const auto set_output = [&result](
      const Eigen::Matrix<double, 4, 1> &value,
      const Eigen::Matrix<double, 4, 1> &first,
      const Eigen::Matrix<double, 4, 1> &second) {
    result.correction.displacement = value.head<3>();
    result.correction.yaw_rad = WrapRadians(value[3]);
    result.first_derivative.displacement = first.head<3>();
    result.first_derivative.yaw_rad = first[3];
    result.second_derivative.displacement = second.head<3>();
    result.second_derivative.yaw_rad = second[3];
  };

  if (knots_.size() == 1)
  {
    set_output(
        knots_.front().fitted, Eigen::Matrix<double, 4, 1>::Zero(),
        Eigen::Matrix<double, 4, 1>::Zero());
    result.extrapolated =
        cumulative_distance_m > knots_.front().cumulative_distance_m;
    return result;
  }

  if (cumulative_distance_m < knots_.front().cumulative_distance_m)
  {
    set_output(
        knots_.front().fitted, Eigen::Matrix<double, 4, 1>::Zero(),
        Eigen::Matrix<double, 4, 1>::Zero());
    return result;
  }

  const auto evaluate_interval = [](
      const Knot &left, const Knot &right, double query,
      Eigen::Matrix<double, 4, 1> *value,
      Eigen::Matrix<double, 4, 1> *first,
      Eigen::Matrix<double, 4, 1> *second) {
    const double spacing =
        right.cumulative_distance_m - left.cumulative_distance_m;
    const double u =
        (query - left.cumulative_distance_m) / spacing;
    const double blend = QuinticSmoothstep(u);
    const double first_blend =
        30.0 * u * u * (1.0 - u) * (1.0 - u) / spacing;
    const double second_blend =
        60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) /
        (spacing * spacing);
    const Eigen::Matrix<double, 4, 1> delta =
        right.fitted - left.fitted;
    *value = left.fitted + blend * delta;
    *first = first_blend * delta;
    *second = second_blend * delta;
  };

  if (cumulative_distance_m >= knots_.back().cumulative_distance_m)
  {
    set_output(
        knots_.back().fitted,
        Eigen::Matrix<double, 4, 1>::Zero(),
        Eigen::Matrix<double, 4, 1>::Zero());
    result.extrapolated =
        cumulative_distance_m > knots_.back().cumulative_distance_m;
    return result;
  }

  const auto upper = std::upper_bound(
      knots_.begin(), knots_.end(), cumulative_distance_m,
      [](double distance, const Knot &knot) {
        return distance < knot.cumulative_distance_m;
      });
  const Knot &right = *upper;
  const Knot &left = *(upper - 1);
  Eigen::Matrix<double, 4, 1> value;
  Eigen::Matrix<double, 4, 1> first;
  Eigen::Matrix<double, 4, 1> second;
  evaluate_interval(
      left, right, cumulative_distance_m, &value, &first, &second);
  set_output(value, first, second);
  return result;
}

RegularizedCorrectionField4d::Correction
RegularizedCorrectionField4d::Evaluate(double cumulative_distance_m) const
{
  return EvaluateWithDerivatives(cumulative_distance_m).correction;
}

Pose3d RegularizedCorrectionField4d::Apply(
    double cumulative_distance_m, const Pose3d &nominal_pose) const
{
  if (!nominal_pose.isFinite())
    throw std::invalid_argument(
        "Regularized correction-field nominal pose is invalid.");
  const Correction correction = Evaluate(cumulative_distance_m);
  const Eigen::Quaterniond yaw(
      Eigen::AngleAxisd(correction.yaw_rad, Eigen::Vector3d::UnitZ()));
  return Pose3d(
      yaw * nominal_pose.rotation,
      nominal_pose.translation + correction.displacement);
}

std::vector<RegularizedCorrectionField4d::KnotState,
            Eigen::aligned_allocator<
                RegularizedCorrectionField4d::KnotState>>
RegularizedCorrectionField4d::knots() const
{
  std::vector<KnotState, Eigen::aligned_allocator<KnotState>> result;
  result.reserve(knots_.size());
  for (const Knot &knot : knots_)
  {
    KnotState state;
    state.keyframe_id = knot.keyframe_id;
    state.timestamp = knot.timestamp;
    state.cumulative_distance_m = knot.cumulative_distance_m;
    state.displacement_target = knot.displacement_target;
    state.nominal_position = knot.nominal_position;
    state.rtk_position = knot.rtk_position;
    state.orientation_yaw_target_rad =
        knot.orientation_yaw_target_rad;
    state.orientation_yaw_confidence =
        knot.orientation_yaw_confidence;
    state.elastic_stiffness = knot.elastic_stiffness;
    state.fitted_correction.displacement = knot.fitted.head<3>();
    state.fitted_correction.yaw_rad = WrapRadians(knot.fitted[3]);
    result.push_back(state);
  }
  return result;
}

}  // namespace my_livo::backend
