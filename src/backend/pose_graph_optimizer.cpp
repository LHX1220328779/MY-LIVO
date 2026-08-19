#include "backend/pose_graph_optimizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
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

void AccumulateWork(const gtsam::ISAM2Result &update,
                    PoseGraphOptimizer::UpdateResult *result)
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

double OptionalError(const boost::optional<double> &error)
{
  return error ? *error : 0.0;
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
        !keyframe->T_map_body().isFinite())
      throw std::invalid_argument("Pose-graph keyframe pose is not finite.");

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    const gtsam::Key current_key = PoseKey(keyframe->id());

    if (keyframe->id() == 0U)
    {
      // A tight prior removes the six-dimensional gauge freedom. The prior
      // measurement is immutable raw odometry, while T_map_body is only the
      // initial value; this distinction is exercised by the unit test.
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          current_key, ToGtsam(keyframe->T_odom_body()), prior_noise);
      new_values.insert(current_key, ToGtsam(keyframe->T_map_body()));
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

    UpdateKeyframePosesLocked();
    result.latest_pose = keyframe->T_map_body();
    result.solution_usable = ValidateEstimateLocked();
    if (!result.solution_usable)
    {
      ++statistics.failed_optimizations;
      throw std::runtime_error(
          "GTSAM iSAM2 returned a non-finite estimate for keyframe " +
          std::to_string(keyframe->id()) + '.');
    }
    WriteCsvLocked(*keyframe, result);
    return result;
  }

  std::vector<Pose3d> OptimizedPoses() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Pose3d> poses;
    poses.reserve(keyframes.size());
    for (const auto &keyframe : keyframes)
      poses.push_back(keyframe->T_map_body());
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
    if (options.odometry_rotation_sigma_deg > 180.0 ||
        options.prior_rotation_sigma_deg > 180.0)
      throw std::invalid_argument(
          "Pose-graph rotation sigmas must not exceed 180 degrees.");
    if (options.relinearize_skip <= 0)
      throw std::invalid_argument(
          "iSAM2 relinearization skip must be positive.");
    if (options.additional_update_steps < 0)
      throw std::invalid_argument(
          "iSAM2 additional update steps must be non-negative.");
  }

  void UpdateKeyframePosesLocked()
  {
    const gtsam::Values estimate = isam2.calculateEstimate();
    for (const auto &keyframe : keyframes)
      keyframe->set_T_map_body(
          FromGtsam(estimate.at<gtsam::Pose3>(PoseKey(keyframe->id()))));
  }

  bool ValidateEstimateLocked() const
  {
    for (const auto &keyframe : keyframes)
      if (!keyframe->T_map_body().isFinite()) return false;
    return true;
  }

  void WriteCsvLocked(const Keyframe &keyframe,
                      const UpdateResult &result)
  {
    if (!csv_stream.is_open()) return;
    const Pose3d &raw = keyframe.T_odom_body();
    const Pose3d optimized = keyframe.T_map_body();
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

  Options options;
  mutable std::mutex mutex;
  gtsam::ISAM2 isam2;
  gtsam::SharedNoiseModel odometry_noise;
  gtsam::SharedNoiseModel prior_noise;
  std::vector<Keyframe::Ptr> keyframes;
  Statistics statistics;
  std::ofstream csv_stream;
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

std::vector<Pose3d> PoseGraphOptimizer::optimized_poses() const
{
  return impl_->OptimizedPoses();
}

PoseGraphOptimizer::Statistics PoseGraphOptimizer::statistics() const
{
  return impl_->GetStatistics();
}

}  // namespace my_livo::backend
