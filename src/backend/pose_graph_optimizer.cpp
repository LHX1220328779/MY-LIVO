#include "backend/pose_graph_optimizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace my_livo::backend
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

gtsam::Key PoseKey(std::uint64_t id)
{
  return gtsam::Symbol('x', id);
}

gtsam::Pose3 ToGtsam(const Pose3d &pose)
{
  return gtsam::Pose3(
      gtsam::Rot3(pose.rotation.toRotationMatrix()),
      gtsam::Point3(pose.translation.x(), pose.translation.y(),
                    pose.translation.z()));
}

Pose3d FromGtsam(const gtsam::Pose3 &pose)
{
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  const gtsam::Point3 &translation = pose.translation();
  return Pose3d(
      Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                         quaternion.z()),
      Eigen::Vector3d(translation.x(), translation.y(), translation.z()));
}

gtsam::SharedNoiseModel PoseNoise(double translation_sigma_m,
                                  double rotation_sigma_deg)
{
  // GTSAM Pose3 tangent order is [rotation, translation], unlike this
  // backend's logged covariance order [translation, rotation].
  const double rotation_sigma_rad = rotation_sigma_deg * kPi / 180.0;
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma_rad, rotation_sigma_rad, rotation_sigma_rad,
      translation_sigma_m, translation_sigma_m, translation_sigma_m;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::SharedNoiseModel RobustPoseNoise(
    double translation_sigma_m, double rotation_sigma_deg,
    const std::string &kernel, double delta)
{
  const auto base = PoseNoise(translation_sigma_m, rotation_sigma_deg);
  gtsam::noiseModel::mEstimator::Base::shared_ptr robust;
  if (kernel == "cauchy")
    robust = gtsam::noiseModel::mEstimator::Cauchy::Create(delta);
  else if (kernel == "huber")
    robust = gtsam::noiseModel::mEstimator::Huber::Create(delta);
  else
    throw std::invalid_argument(
        "Pose-graph robust kernel must be 'cauchy' or 'huber'.");
  return gtsam::noiseModel::Robust::Create(robust, base);
}

gtsam::ISAM2Params MakeIsam2Parameters(
    const PoseGraphOptimizer::Options &options)
{
  gtsam::ISAM2Params parameters;
  parameters.optimizationParams =
      gtsam::ISAM2GaussNewtonParams(options.wildfire_threshold);
  parameters.relinearizeThreshold = options.relinearize_threshold;
  parameters.relinearizeSkip = options.relinearize_skip;
  parameters.enableRelinearization = true;
  parameters.evaluateNonlinearError = true;
  parameters.factorization = gtsam::ISAM2Params::CHOLESKY;
  parameters.cacheLinearizedFactors = true;
  parameters.enablePartialRelinearizationCheck = false;
  // GTSAM 4.2.2 leaves several scalar fields in ISAM2Result uninitialized
  // when an empty update does not recalculate the Bayes tree. Detailed
  // variable status is value-initialized and therefore provides reliable
  // per-update diagnostics for both normal and empty refinement updates.
  parameters.enableDetailedResults = true;
  return parameters;
}

std::uint64_t CountDetailedStatus(
    const gtsam::ISAM2Result &result,
    bool gtsam::ISAM2Result::DetailedResults::VariableStatus::*member)
{
  if (!result.detail)
    throw std::logic_error(
        "GTSAM iSAM2 detailed results were unexpectedly disabled.");
  std::uint64_t count = 0;
  for (const auto &entry : result.detail->variableStatus)
    if (entry.second.*member) ++count;
  return count;
}

template <typename ResultType>
void AccumulateWork(const gtsam::ISAM2Result &update, ResultType *result)
{
  result->variables_relinearized += CountDetailedStatus(
      update,
      &gtsam::ISAM2Result::DetailedResults::VariableStatus::isRelinearized);
  result->variables_reeliminated += CountDetailedStatus(
      update,
      &gtsam::ISAM2Result::DetailedResults::VariableStatus::isReeliminated);
}

double PositionDifference(const Pose3d &left, const Pose3d &right)
{
  return (left.translation - right.translation).norm();
}

double RotationDifferenceDegrees(const Pose3d &left, const Pose3d &right)
{
  return left.rotation.angularDistance(right.rotation) * 180.0 / kPi;
}

// GTSAM >= 4.2 reports errorBefore/errorAfter as std::optional while older
// releases use boost::optional; accept either.
template <typename Optional>
double OptionalError(const Optional &error)
{
  return error ? *error : 0.0;
}

double MillisecondsSince(const std::chrono::steady_clock::time_point &start)
{
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

struct RelativeResidual
{
  double translation_m = 0.0;
  double rotation_deg = 0.0;
};

RelativeResidual RelativePoseResidual(const Pose3d &left,
                                      const Pose3d &right,
                                      const Pose3d &measurement)
{
  const Pose3d prediction = left.inverse() * right;
  const Pose3d error = measurement.inverse() * prediction;
  return RelativeResidual{
      error.translation.norm(),
      measurement.rotation.angularDistance(prediction.rotation) *
          180.0 / kPi};
}
}  // namespace

class PoseGraphOptimizer::Impl
{
public:
  explicit Impl(const Options &input_options)
      : options(input_options), isam2(MakeIsam2Parameters(input_options))
  {
    ValidateOptions();
    odometry_noise = PoseNoise(
        options.odometry_translation_sigma_m,
        options.odometry_rotation_sigma_deg);
    prior_noise = PoseNoise(options.prior_translation_sigma_m,
                            options.prior_rotation_sigma_deg);
    loop_noise = RobustPoseNoise(
        options.loop_translation_sigma_m, options.loop_rotation_sigma_deg,
        options.loop_robust_kernel, options.loop_robust_delta);

    if (!options.csv_path.empty())
    {
      const std::filesystem::path csv_path(options.csv_path);
      if (csv_path.has_parent_path())
        std::filesystem::create_directories(csv_path.parent_path());
      csv_stream.open(csv_path, std::ios::out | std::ios::trunc);
      if (!csv_stream.is_open())
        throw std::runtime_error("Cannot open pose-graph CSV: " +
                                 csv_path.string());
      csv_stream
          << "solver,id,timestamp,nodes,odometry_factors,optimization_ran,"
             "solution_usable,iterations,initial_cost,final_cost,"
             "optimization_time_ms,variables_relinearized,"
             "variables_reeliminated,raw_tx,raw_ty,raw_tz,raw_qx,raw_qy,"
             "raw_qz,raw_qw,opt_tx,opt_ty,opt_tz,opt_qx,opt_qy,opt_qz,"
             "opt_qw,position_delta_m,angle_delta_deg\n";
    }
    if (!options.loop_csv_path.empty())
    {
      const std::filesystem::path csv_path(options.loop_csv_path);
      if (csv_path.has_parent_path())
        std::filesystem::create_directories(csv_path.parent_path());
      loop_csv_stream.open(csv_path, std::ios::out | std::ios::trunc);
      if (!loop_csv_stream.is_open())
        throw std::runtime_error("Cannot open loop-factor CSV: " +
                                 csv_path.string());
      loop_csv_stream
          << "historical_id,current_id,robust_kernel,robust_delta,"
             "translation_sigma_m,rotation_sigma_deg,initial_cost,"
             "final_cost,residual_translation_before_m,"
             "residual_rotation_before_deg,residual_translation_after_m,"
             "residual_rotation_after_deg,maximum_pose_correction_m,"
             "maximum_pose_correction_deg,optimization_time_ms,"
             "variables_relinearized,variables_reeliminated,measurement_tx,"
             "measurement_ty,measurement_tz,measurement_qx,measurement_qy,"
             "measurement_qz,measurement_qw\n";
    }
    if (!options.rtk_csv_path.empty())
    {
      const std::filesystem::path csv_path(options.rtk_csv_path);
      if (csv_path.has_parent_path())
        std::filesystem::create_directories(csv_path.parent_path());
      rtk_csv_stream.open(csv_path, std::ios::out | std::ios::trunc);
      if (!rtk_csv_stream.is_open())
        throw std::runtime_error("Cannot open RTK-factor CSV: " +
                                 csv_path.string());
      rtk_csv_stream
          << "keyframe_id,robust_kernel,robust_delta,sigma_x,sigma_y,"
             "sigma_z,measurement_x,measurement_y,measurement_z,"
             "innovation_before_x,innovation_before_y,"
             "innovation_before_z,innovation_after_x,innovation_after_y,"
             "innovation_after_z,initial_cost,final_cost,"
             "maximum_pose_correction_m,maximum_pose_correction_deg,"
             "optimization_time_ms,variables_relinearized,"
             "variables_reeliminated\n";
    }
    InitializeOptimizedTrajectoryLocked();
  }

  ~Impl()
  {
    std::lock_guard<std::mutex> lock(mutex);
    WriteOptimizedTrajectoryLocked();
  }

  UpdateResult AddKeyframe(const Keyframe::Ptr &keyframe)
  {
    if (!keyframe)
      throw std::invalid_argument("Cannot add a null keyframe to pose graph.");
    std::lock_guard<std::mutex> lock(mutex);
    if (keyframe->id() != keyframes.size())
      throw std::logic_error(
          "Pose-graph keyframe IDs must be contiguous and start at zero.");
    if (!keyframes.empty() &&
        keyframe->timestamp() <= keyframes.back()->timestamp())
      throw std::logic_error(
          "Pose-graph keyframe timestamps must be strictly increasing.");
    if (!keyframe->T_odom_body().isFinite() ||
        !keyframe->T_slam_body().isFinite())
      throw std::invalid_argument("Pose-graph keyframe pose is not finite.");

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    const gtsam::Key current_key = PoseKey(keyframe->id());

    if (keyframe->id() == 0U)
    {
      // A tight prior removes the six-dimensional gauge freedom. The prior
      // measurement is immutable raw odometry, while T_slam_body is only the
      // initial value; this distinction is exercised by the unit test.
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          current_key, ToGtsam(keyframe->T_odom_body()), prior_noise);
      new_values.insert(current_key, ToGtsam(keyframe->T_slam_body()));
    }
    else
    {
      const Keyframe &previous = *keyframes.back();
      const Pose3d measurement =
          previous.T_odom_body().inverse() * keyframe->T_odom_body();
      const gtsam::Pose3 gtsam_measurement = ToGtsam(measurement);
      const gtsam::Key previous_key = PoseKey(previous.id());
      new_factors.emplace_shared<
          gtsam::BetweenFactor<gtsam::Pose3>>(
              previous_key, current_key, gtsam_measurement,
              odometry_noise);

      // Propagate the latest optimized pose through the raw relative motion.
      // This remains a good initial value after future loop/RTK corrections.
      const gtsam::Pose3 previous_estimate =
          isam2.calculateEstimate<gtsam::Pose3>(previous_key);
      new_values.insert(current_key,
                        previous_estimate.compose(gtsam_measurement));
    }

    UpdateResult result;
    result.optimization_ran = keyframe->id() > 0U;
    const auto start = std::chrono::steady_clock::now();
    try
    {
      gtsam::ISAM2Result update_result =
          isam2.update(new_factors, new_values);
      result.initial_cost = OptionalError(update_result.errorBefore);
      result.final_cost = OptionalError(update_result.errorAfter);
      AccumulateWork(update_result, &result);

      for (int step = 0; step < options.additional_update_steps; ++step)
      {
        update_result = isam2.update();
        result.final_cost = OptionalError(update_result.errorAfter);
        AccumulateWork(update_result, &result);
      }
    }
    catch (const std::exception &error)
    {
      ++statistics.failed_optimizations;
      throw std::runtime_error(
          "GTSAM iSAM2 update failed for keyframe " +
          std::to_string(keyframe->id()) + ": " + error.what());
    }
    const auto end = std::chrono::steady_clock::now();

    result.iterations = result.optimization_ran
                            ? 1 + options.additional_update_steps
                            : 0;
    result.optimization_time_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    keyframes.push_back(keyframe);
    ++statistics.nodes;
    if (keyframe->id() > 0U)
    {
      ++statistics.odometry_factors;
      ++statistics.optimization_runs;
    }
    statistics.variables_relinearized += result.variables_relinearized;
    statistics.variables_reeliminated += result.variables_reeliminated;
    statistics.last_optimization_time_ms = result.optimization_time_ms;
    statistics.maximum_optimization_time_ms = std::max(
        statistics.maximum_optimization_time_ms,
        result.optimization_time_ms);

    // An odometry-only append introduces no new constraint on historical
    // poses. Extracting every pose here would make normal graph growth O(N^2)
    // over a long mission; loop updates still refresh all poses.
    keyframe->set_T_slam_body(FromGtsam(
        isam2.calculateEstimate<gtsam::Pose3>(current_key)));
    result.latest_pose = keyframe->T_slam_body();
    result.solution_usable = result.latest_pose.isFinite();
    if (!result.solution_usable)
    {
      ++statistics.failed_optimizations;
      throw std::runtime_error(
          "GTSAM iSAM2 returned a non-finite estimate for keyframe " +
          std::to_string(keyframe->id()) + '.');
    }
    WriteCsvLocked(*keyframe, result);
    AppendOptimizedTrajectoryLocked(*keyframe);
    return result;
  }

  LoopUpdateResult AddLoopFactor(
      std::uint64_t historical_id, std::uint64_t current_id,
      const Pose3d &T_historical_current)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (historical_id >= current_id || current_id >= keyframes.size())
      throw std::invalid_argument(
          "Loop factor IDs must reference existing ordered graph nodes.");
    if (!T_historical_current.isFinite())
      throw std::invalid_argument("Loop factor measurement is not finite.");
    const auto pair = std::make_pair(historical_id, current_id);
    if (loop_factor_pairs.count(pair) != 0U)
      throw std::logic_error("Duplicate loop factor for keyframe pair " +
                             std::to_string(historical_id) + "<-" +
                             std::to_string(current_id) + '.');

    LoopUpdateResult result;
    result.historical_id = historical_id;
    result.current_id = current_id;
    const Pose3d historical_before =
        keyframes[historical_id]->T_slam_body();
    const Pose3d current_before = keyframes[current_id]->T_slam_body();
    const RelativeResidual residual_before = RelativePoseResidual(
        historical_before, current_before, T_historical_current);
    result.residual_translation_before_m = residual_before.translation_m;
    result.residual_rotation_before_deg = residual_before.rotation_deg;
    std::vector<Pose3d> poses_before;
    poses_before.reserve(keyframes.size());
    for (const auto &keyframe : keyframes)
      poses_before.push_back(keyframe->T_slam_body());

    gtsam::NonlinearFactorGraph new_factors;
    new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        PoseKey(historical_id), PoseKey(current_id),
        ToGtsam(T_historical_current), loop_noise);
    const auto start = std::chrono::steady_clock::now();
    try
    {
      gtsam::ISAM2Result update_result =
          isam2.update(new_factors, gtsam::Values());
      result.initial_cost = OptionalError(update_result.errorBefore);
      result.final_cost = OptionalError(update_result.errorAfter);
      AccumulateWork(update_result, &result);
      for (int step = 0; step < options.loop_additional_update_steps; ++step)
      {
        update_result = isam2.update();
        result.final_cost = OptionalError(update_result.errorAfter);
        AccumulateWork(update_result, &result);
      }
    }
    catch (const std::exception &error)
    {
      ++statistics.failed_optimizations;
      throw std::runtime_error(
          "GTSAM iSAM2 loop update failed for pair " +
          std::to_string(historical_id) + "<-" +
          std::to_string(current_id) + ": " + error.what());
    }
    result.optimization_time_ms = MillisecondsSince(start);
    result.updates = 1 + options.loop_additional_update_steps;

    loop_factor_pairs.insert(pair);
    ++statistics.loop_factors;
    ++statistics.loop_optimization_runs;
    statistics.variables_relinearized += result.variables_relinearized;
    statistics.variables_reeliminated += result.variables_reeliminated;
    statistics.last_optimization_time_ms = result.optimization_time_ms;
    statistics.maximum_optimization_time_ms = std::max(
        statistics.maximum_optimization_time_ms,
        result.optimization_time_ms);

    UpdateKeyframePosesLocked();
    result.solution_usable = ValidateEstimateLocked();
    if (!result.solution_usable)
    {
      ++statistics.failed_optimizations;
      throw std::runtime_error(
          "GTSAM iSAM2 returned a non-finite estimate after loop pair " +
          std::to_string(historical_id) + "<-" +
          std::to_string(current_id) + '.');
    }
    const Pose3d historical_after =
        keyframes[historical_id]->T_slam_body();
    const Pose3d current_after = keyframes[current_id]->T_slam_body();
    const RelativeResidual residual_after = RelativePoseResidual(
        historical_after, current_after, T_historical_current);
    result.residual_translation_after_m = residual_after.translation_m;
    result.residual_rotation_after_deg = residual_after.rotation_deg;
    for (std::size_t index = 0; index < keyframes.size(); ++index)
    {
      const Pose3d pose_after = keyframes[index]->T_slam_body();
      result.maximum_pose_correction_m = std::max(
          result.maximum_pose_correction_m,
          PositionDifference(poses_before[index], pose_after));
      result.maximum_pose_correction_deg = std::max(
          result.maximum_pose_correction_deg,
          RotationDifferenceDegrees(poses_before[index], pose_after));
    }
    result.added = true;
    WriteLoopCsvLocked(T_historical_current, result);
    WriteOptimizedTrajectoryLocked();
    return result;
  }

  std::vector<Pose3d> OptimizedPoses() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Pose3d> poses;
    poses.reserve(keyframes.size());
    for (const auto &keyframe : keyframes)
      poses.push_back(keyframe->T_slam_body());
    return poses;
  }

  Statistics GetStatistics() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return statistics;
  }

private:
  void ValidateOptions() const
  {
    const auto require_positive = [](double value, const char *name) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(name) +
                                    " must be finite and positive.");
    };
    require_positive(options.odometry_translation_sigma_m,
                     "Odometry translation sigma");
    require_positive(options.odometry_rotation_sigma_deg,
                     "Odometry rotation sigma");
    require_positive(options.prior_translation_sigma_m,
                     "Prior translation sigma");
    require_positive(options.prior_rotation_sigma_deg,
                     "Prior rotation sigma");
    require_positive(options.relinearize_threshold,
                     "iSAM2 relinearization threshold");
    require_positive(options.wildfire_threshold,
                     "iSAM2 wildfire threshold");
    require_positive(options.loop_translation_sigma_m,
                     "Loop translation sigma");
    require_positive(options.loop_rotation_sigma_deg,
                     "Loop rotation sigma");
    require_positive(options.loop_robust_delta,
                     "Loop robust delta");
    if (options.odometry_rotation_sigma_deg > 180.0 ||
        options.prior_rotation_sigma_deg > 180.0 ||
        options.loop_rotation_sigma_deg > 180.0)
      throw std::invalid_argument(
          "Pose-graph rotation sigmas must not exceed 180 degrees.");
    if (options.relinearize_skip <= 0)
      throw std::invalid_argument(
          "iSAM2 relinearization skip must be positive.");
    if (options.additional_update_steps < 0)
      throw std::invalid_argument(
          "iSAM2 additional update steps must be non-negative.");
    if (options.loop_additional_update_steps < 0)
      throw std::invalid_argument(
          "iSAM2 loop additional update steps must be non-negative.");
    if (options.loop_robust_kernel != "cauchy" &&
        options.loop_robust_kernel != "huber")
      throw std::invalid_argument(
          "Loop robust kernel must be 'cauchy' or 'huber'.");
  }

  void UpdateKeyframePosesLocked()
  {
    const gtsam::Values estimate = isam2.calculateEstimate();
    for (const auto &keyframe : keyframes)
      keyframe->set_T_slam_body(
          FromGtsam(estimate.at<gtsam::Pose3>(PoseKey(keyframe->id()))));
  }

  bool ValidateEstimateLocked() const
  {
    for (const auto &keyframe : keyframes)
      if (!keyframe->T_slam_body().isFinite()) return false;
    return true;
  }

  void WriteCsvLocked(const Keyframe &keyframe,
                      const UpdateResult &result)
  {
    if (!csv_stream.is_open()) return;
    const Pose3d &raw = keyframe.T_odom_body();
    const Pose3d optimized = keyframe.T_slam_body();
    csv_stream << std::setprecision(17) << "gtsam_isam2," << keyframe.id()
               << ',' << keyframe.timestamp() << ',' << statistics.nodes
               << ',' << statistics.odometry_factors << ','
               << static_cast<int>(result.optimization_ran) << ','
               << static_cast<int>(result.solution_usable) << ','
               << result.iterations << ',' << result.initial_cost << ','
               << result.final_cost << ',' << result.optimization_time_ms
               << ',' << result.variables_relinearized << ','
               << result.variables_reeliminated << ','
               << raw.translation.x() << ',' << raw.translation.y() << ','
               << raw.translation.z() << ',' << raw.rotation.x() << ','
               << raw.rotation.y() << ',' << raw.rotation.z() << ','
               << raw.rotation.w() << ',' << optimized.translation.x() << ','
               << optimized.translation.y() << ','
               << optimized.translation.z() << ',' << optimized.rotation.x()
               << ',' << optimized.rotation.y() << ','
               << optimized.rotation.z() << ',' << optimized.rotation.w()
               << ',' << PositionDifference(raw, optimized) << ','
               << RotationDifferenceDegrees(raw, optimized) << '\n';
    csv_stream.flush();
  }

  void WriteLoopCsvLocked(const Pose3d &measurement,
                          const LoopUpdateResult &result)
  {
    if (!loop_csv_stream.is_open()) return;
    loop_csv_stream
        << std::setprecision(17) << result.historical_id << ','
        << result.current_id << ',' << options.loop_robust_kernel << ','
        << options.loop_robust_delta << ','
        << options.loop_translation_sigma_m << ','
        << options.loop_rotation_sigma_deg << ',' << result.initial_cost
        << ',' << result.final_cost << ','
        << result.residual_translation_before_m << ','
        << result.residual_rotation_before_deg << ','
        << result.residual_translation_after_m << ','
        << result.residual_rotation_after_deg << ','
        << result.maximum_pose_correction_m << ','
        << result.maximum_pose_correction_deg << ','
        << result.optimization_time_ms << ','
        << result.variables_relinearized << ','
        << result.variables_reeliminated << ','
        << measurement.translation.x() << ','
        << measurement.translation.y() << ','
        << measurement.translation.z() << ',' << measurement.rotation.x()
        << ',' << measurement.rotation.y() << ',' << measurement.rotation.z()
        << ',' << measurement.rotation.w() << '\n';
    loop_csv_stream.flush();
  }

  static void WriteOptimizedTrajectoryHeader(std::ostream &stream)
  {
    stream << "id,timestamp,raw_tx,raw_ty,raw_tz,raw_qx,raw_qy,raw_qz,"
              "raw_qw,opt_tx,opt_ty,opt_tz,opt_qx,opt_qy,opt_qz,opt_qw,"
              "position_delta_m,angle_delta_deg\n";
  }

  static void WriteOptimizedTrajectoryRow(
      std::ostream &stream, const Keyframe &keyframe)
  {
    const Pose3d &raw = keyframe.T_odom_body();
    const Pose3d optimized = keyframe.T_slam_body();
    stream << std::setprecision(17) << keyframe.id() << ','
           << keyframe.timestamp() << ',' << raw.translation.x() << ','
           << raw.translation.y() << ',' << raw.translation.z() << ','
           << raw.rotation.x() << ',' << raw.rotation.y() << ','
           << raw.rotation.z() << ',' << raw.rotation.w() << ','
           << optimized.translation.x() << ','
           << optimized.translation.y() << ','
           << optimized.translation.z() << ',' << optimized.rotation.x()
           << ',' << optimized.rotation.y() << ','
           << optimized.rotation.z() << ',' << optimized.rotation.w()
           << ',' << PositionDifference(raw, optimized) << ','
           << RotationDifferenceDegrees(raw, optimized) << '\n';
  }

  void InitializeOptimizedTrajectoryLocked()
  {
    if (options.optimized_trajectory_csv_path.empty()) return;
    const std::filesystem::path path(
        options.optimized_trajectory_csv_path);
    if (path.has_parent_path())
      std::filesystem::create_directories(path.parent_path());
    optimized_trajectory_csv_stream.open(
        path, std::ios::out | std::ios::trunc);
    if (!optimized_trajectory_csv_stream.is_open())
      throw std::runtime_error("Cannot open optimized-trajectory CSV: " +
                               path.string());
    WriteOptimizedTrajectoryHeader(optimized_trajectory_csv_stream);
    optimized_trajectory_csv_stream.flush();
  }

  void AppendOptimizedTrajectoryLocked(const Keyframe &keyframe)
  {
    if (!optimized_trajectory_csv_stream.is_open()) return;
    WriteOptimizedTrajectoryRow(optimized_trajectory_csv_stream, keyframe);
    optimized_trajectory_csv_stream.flush();
  }

  void ReopenOptimizedTrajectoryForAppendLocked(
      const std::filesystem::path &path)
  {
    optimized_trajectory_csv_stream.clear();
    optimized_trajectory_csv_stream.open(path, std::ios::out | std::ios::app);
  }

  void WriteOptimizedTrajectoryLocked()
  {
    if (options.optimized_trajectory_csv_path.empty()) return;
    const std::filesystem::path path(
        options.optimized_trajectory_csv_path);
    std::filesystem::path temporary_path = path;
    temporary_path += ".tmp";
    std::ofstream temporary_stream(
        temporary_path, std::ios::out | std::ios::trunc);
    if (!temporary_stream.is_open()) return;
    WriteOptimizedTrajectoryHeader(temporary_stream);
    for (const auto &keyframe : keyframes)
    {
      WriteOptimizedTrajectoryRow(temporary_stream, *keyframe);
    }
    temporary_stream.flush();
    if (!temporary_stream.good())
    {
      temporary_stream.close();
      std::error_code error;
      std::filesystem::remove(temporary_path, error);
      return;
    }
    temporary_stream.close();

    if (optimized_trajectory_csv_stream.is_open())
    {
      optimized_trajectory_csv_stream.flush();
      optimized_trajectory_csv_stream.close();
    }
    // std::rename atomically replaces an existing file on the POSIX target
    // platform.  Validators therefore see either the previous complete
    // snapshot or the new complete snapshot, never a truncated rewrite.
    if (std::rename(temporary_path.c_str(), path.c_str()) != 0)
    {
      std::error_code error;
      std::filesystem::remove(temporary_path, error);
    }
    ReopenOptimizedTrajectoryForAppendLocked(path);
  }

  Options options;
  mutable std::mutex mutex;
  gtsam::ISAM2 isam2;
  gtsam::SharedNoiseModel odometry_noise;
  gtsam::SharedNoiseModel prior_noise;
  gtsam::SharedNoiseModel loop_noise;
  std::vector<Keyframe::Ptr> keyframes;
  std::set<std::pair<std::uint64_t, std::uint64_t>> loop_factor_pairs;
  Statistics statistics;
  std::ofstream csv_stream;
  std::ofstream loop_csv_stream;
  std::ofstream rtk_csv_stream;
  std::ofstream optimized_trajectory_csv_stream;
};

PoseGraphOptimizer::PoseGraphOptimizer(const Options &options)
    : impl_(std::make_unique<Impl>(options))
{
}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;

PoseGraphOptimizer::UpdateResult PoseGraphOptimizer::AddKeyframe(
    const Keyframe::Ptr &keyframe)
{
  return impl_->AddKeyframe(keyframe);
}

PoseGraphOptimizer::LoopUpdateResult PoseGraphOptimizer::AddLoopFactor(
    std::uint64_t historical_id, std::uint64_t current_id,
    const Pose3d &T_historical_current)
{
  return impl_->AddLoopFactor(
      historical_id, current_id, T_historical_current);
}

std::vector<Pose3d> PoseGraphOptimizer::optimized_poses() const
{
  return impl_->OptimizedPoses();
}

PoseGraphOptimizer::Statistics PoseGraphOptimizer::statistics() const
{
  return impl_->GetStatistics();
}

}  // namespace my_livo::backend
