#include "backend/loop_registration.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace my_livo::backend
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

struct FrozenFrame
{
  std::uint64_t id = 0;
  Pose3d T_map_body;
  KeyframeCloud::ConstPtr cloud_body;
};

struct RegistrationJob
{
  LoopCandidate candidate;
  FrozenFrame current;
  FrozenFrame historical_anchor;
  std::vector<FrozenFrame> target_frames;
};

double MillisecondsSince(const std::chrono::steady_clock::time_point &start)
{
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

Eigen::Matrix4f PoseMatrix(const Pose3d &pose)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) =
      pose.rotation.toRotationMatrix().cast<float>();
  matrix.block<3, 1>(0, 3) = pose.translation.cast<float>();
  return matrix;
}

Pose3d PoseFromMatrix(const Eigen::Matrix4f &matrix)
{
  Eigen::Quaterniond rotation(matrix.block<3, 3>(0, 0).cast<double>());
  rotation.normalize();
  return Pose3d(rotation, matrix.block<3, 1>(0, 3).cast<double>());
}

KeyframeCloud::Ptr VoxelDownsample(const KeyframeCloud::ConstPtr &input,
                                   double leaf_size_m)
{
  KeyframeCloud::Ptr output(new KeyframeCloud());
  pcl::VoxelGrid<KeyframePoint> filter;
  const float leaf = static_cast<float>(leaf_size_m);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}

double FiniteOr(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}

void OpenCsv(const std::string &path, std::ofstream *stream,
             const std::string &header, const char *description)
{
  if (path.empty()) return;
  const std::filesystem::path csv_path(path);
  if (csv_path.has_parent_path())
    std::filesystem::create_directories(csv_path.parent_path());
  stream->open(csv_path, std::ios::out | std::ios::trunc);
  if (!stream->is_open())
    throw std::runtime_error(std::string("Cannot open ") + description +
                             " CSV: " + csv_path.string());
  *stream << header << '\n';
}
}  // namespace

const char *LoopRegistrationStatusToString(LoopRegistrationStatus status)
{
  switch (status)
  {
    case LoopRegistrationStatus::kCompleted:
      return "completed";
    case LoopRegistrationStatus::kInvalidInput:
      return "invalid_input";
    case LoopRegistrationStatus::kInsufficientSourcePoints:
      return "insufficient_source_points";
    case LoopRegistrationStatus::kInsufficientTargetPoints:
      return "insufficient_target_points";
    case LoopRegistrationStatus::kNdtException:
      return "ndt_exception";
    case LoopRegistrationStatus::kNonfiniteResult:
      return "nonfinite_result";
  }
  return "unknown";
}

class LoopRegistration::Impl
{
public:
  explicit Impl(const Options &input_options) : options(input_options)
  {
    ValidateOptions();
    OpenCsv(
        options.registration_csv_path, &registration_csv,
        "current_id,candidate_id,status,converged,converged_levels,"
        "configured_levels,levels_executed,raw_source_points,"
        "raw_target_points,final_source_points,final_target_points,"
        "transformation_probability,fitness_score_m2,overlap,"
        "overlap_rmse_m,initial_tx,initial_ty,initial_tz,initial_qx,"
        "initial_qy,initial_qz,initial_qw,final_tx,final_ty,final_tz,"
        "final_qx,final_qy,final_qz,final_qw,correction_translation_m,"
        "correction_angle_deg,registration_time_ms",
        "loop-registration");
    OpenCsv(
        options.level_csv_path, &level_csv,
        "current_id,candidate_id,level,resolution_m,voxel_leaf_size_m,"
        "source_points,target_points,converged,iterations,"
        "transformation_probability,fitness_score_m2,elapsed_ms",
        "loop-registration level");
    worker = std::thread([this]() { WorkerLoop(); });
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stop_requested = true;
    }
    work_available.notify_all();
    if (worker.joinable()) worker.join();
  }

  LoopRegistrationResult Register(
      const LoopCandidate &candidate,
      const std::vector<Keyframe::Ptr> &keyframes) const
  {
    RegistrationJob job;
    std::string diagnostic;
    if (!FreezeJob(candidate, keyframes, &job, &diagnostic))
    {
      LoopRegistrationResult result;
      result.candidate = candidate;
      result.T_candidate_current_initial =
          candidate.T_candidate_current_initial;
      result.T_candidate_current = candidate.T_candidate_current_initial;
      result.status = LoopRegistrationStatus::kInvalidInput;
      result.diagnostic = diagnostic;
      return result;
    }
    return Compute(job);
  }

  bool Enqueue(const LoopCandidate &candidate,
               const std::vector<Keyframe::Ptr> &keyframes)
  {
    RegistrationJob job;
    std::string diagnostic;
    if (!FreezeJob(candidate, keyframes, &job, &diagnostic))
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++statistics.queue_drops;
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex);
    if (stop_requested || queue.size() >= options.maximum_queue_size)
    {
      ++statistics.queue_drops;
      return false;
    }
    queue.push_back(std::move(job));
    ++statistics.enqueued;
    statistics.maximum_queue_depth =
        std::max(statistics.maximum_queue_depth, queue.size());
    work_available.notify_one();
    return true;
  }

  void SetResultCallback(ResultCallback input_callback)
  {
    std::lock_guard<std::mutex> lock(mutex);
    callback = std::move(input_callback);
  }

  void WaitUntilIdle()
  {
    std::unique_lock<std::mutex> lock(mutex);
    idle.wait(lock, [this]() { return queue.empty() && !worker_busy; });
  }

  Statistics GetStatistics() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return statistics;
  }

private:
  void ValidateOptions() const
  {
    if (options.target_submap_half_width_keyframes < 0)
      throw std::invalid_argument(
          "Loop-registration target half width must be non-negative.");
    if (options.target_submap_stride_keyframes <= 0)
      throw std::invalid_argument(
          "Loop-registration target stride must be positive.");
    if (options.resolutions_m.empty())
      throw std::invalid_argument(
          "Loop-registration resolutions must not be empty.");
    double previous = std::numeric_limits<double>::infinity();
    for (double resolution : options.resolutions_m)
    {
      if (!std::isfinite(resolution) || resolution <= 0.0 ||
          resolution >= previous)
        throw std::invalid_argument(
            "Loop-registration resolutions must be finite, positive, and "
            "strictly coarse-to-fine.");
      previous = resolution;
    }
    const auto positive = [](double value, const char *name) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(name) +
                                    " must be finite and positive.");
    };
    positive(options.voxel_leaf_size_ratio,
             "Loop-registration voxel leaf ratio");
    positive(options.minimum_voxel_leaf_size_m,
             "Loop-registration minimum voxel leaf size");
    positive(options.transformation_epsilon,
             "Loop-registration transformation epsilon");
    positive(options.step_size, "Loop-registration step size");
    positive(options.overlap_max_correspondence_distance_m,
             "Loop-registration overlap distance");
    if (options.maximum_iterations <= 0)
      throw std::invalid_argument(
          "Loop-registration maximum iterations must be positive.");
    if (options.minimum_source_points == 0U ||
        options.minimum_target_points == 0U ||
        options.maximum_queue_size == 0U)
      throw std::invalid_argument(
          "Loop-registration point and queue limits must be positive.");
    if (!options.registration_csv_path.empty() &&
        options.registration_csv_path == options.level_csv_path)
      throw std::invalid_argument(
          "Loop-registration summary and level CSV paths must differ.");
  }

  bool FreezeFrame(const Keyframe::Ptr &keyframe, FrozenFrame *frame,
                   std::string *diagnostic) const
  {
    if (!keyframe)
    {
      *diagnostic = "null keyframe";
      return false;
    }
    const Pose3d pose = keyframe->T_map_body();
    // KeyframeManager already performs a full finite-point validation before
    // storing an immutable cloud. Repeating that O(points) scan here would put
    // NDT preparation work back onto the FAST-LIVO2 thread.
    if (!pose.isFinite() || !keyframe->cloud_body() ||
        keyframe->cloud_body()->empty())
    {
      *diagnostic = "non-finite pose or invalid cloud";
      return false;
    }
    frame->id = keyframe->id();
    frame->T_map_body = pose;
    frame->cloud_body = keyframe->cloud_body();
    return true;
  }

  bool FreezeJob(const LoopCandidate &candidate,
                 const std::vector<Keyframe::Ptr> &keyframes,
                 RegistrationJob *job, std::string *diagnostic) const
  {
    if (candidate.candidate_id >= candidate.current_id ||
        candidate.current_id >= keyframes.size() ||
        candidate.candidate_id >= keyframes.size() ||
        !candidate.T_candidate_current_initial.isFinite())
    {
      *diagnostic = "candidate IDs or initial transform are invalid";
      return false;
    }
    for (std::size_t index = 0; index < keyframes.size(); ++index)
    {
      if (!keyframes[index] || keyframes[index]->id() != index)
      {
        *diagnostic = "keyframe snapshot IDs are not contiguous";
        return false;
      }
    }

    job->candidate = candidate;
    if (!FreezeFrame(keyframes[candidate.current_id], &job->current,
                     diagnostic) ||
        !FreezeFrame(keyframes[candidate.candidate_id],
                     &job->historical_anchor, diagnostic))
      return false;

    std::set<std::uint64_t> target_ids;
    const std::int64_t anchor =
        static_cast<std::int64_t>(candidate.candidate_id);
    const std::int64_t half = options.target_submap_half_width_keyframes;
    const std::int64_t stride = options.target_submap_stride_keyframes;
    for (std::int64_t offset = -half; offset <= half; offset += stride)
    {
      const std::int64_t id = anchor + offset;
      if (id >= 0 && id < static_cast<std::int64_t>(keyframes.size()))
        target_ids.insert(static_cast<std::uint64_t>(id));
    }
    target_ids.insert(candidate.candidate_id);
    job->target_frames.reserve(target_ids.size());
    for (std::uint64_t id : target_ids)
    {
      FrozenFrame frame;
      if (!FreezeFrame(keyframes[id], &frame, diagnostic)) return false;
      job->target_frames.push_back(std::move(frame));
    }
    return true;
  }

  LoopRegistrationResult Compute(const RegistrationJob &job) const
  {
    const auto registration_start = std::chrono::steady_clock::now();
    LoopRegistrationResult result;
    result.candidate = job.candidate;
    result.T_candidate_current_initial =
        job.candidate.T_candidate_current_initial;
    result.T_candidate_current = result.T_candidate_current_initial;
    result.T_map_candidate_snapshot = job.historical_anchor.T_map_body;
    result.T_map_current_snapshot = job.current.T_map_body;
    result.raw_source_points = job.current.cloud_body->size();

    try
    {
      KeyframeCloud::Ptr target(new KeyframeCloud());
      const Pose3d T_candidate_map =
          job.historical_anchor.T_map_body.inverse();
      for (const FrozenFrame &frame : job.target_frames)
      {
        const Pose3d T_candidate_frame =
            T_candidate_map * frame.T_map_body;
        KeyframeCloud transformed;
        pcl::transformPointCloud(*frame.cloud_body, transformed,
                                 PoseMatrix(T_candidate_frame));
        *target += transformed;
      }
      result.raw_target_points = target->size();

      if (result.raw_source_points < options.minimum_source_points)
      {
        result.status =
            LoopRegistrationStatus::kInsufficientSourcePoints;
        result.diagnostic = "raw source cloud is too small";
        result.registration_time_ms = MillisecondsSince(registration_start);
        return result;
      }
      if (result.raw_target_points < options.minimum_target_points)
      {
        result.status =
            LoopRegistrationStatus::kInsufficientTargetPoints;
        result.diagnostic = "raw target submap is too small";
        result.registration_time_ms = MillisecondsSince(registration_start);
        return result;
      }

      Eigen::Matrix4f estimate =
          PoseMatrix(result.T_candidate_current_initial);
      KeyframeCloud::Ptr final_source;
      KeyframeCloud::Ptr final_target;
      for (double resolution : options.resolutions_m)
      {
        const auto level_start = std::chrono::steady_clock::now();
        LoopRegistrationLevelResult level;
        level.resolution_m = resolution;
        level.voxel_leaf_size_m = std::max(
            options.minimum_voxel_leaf_size_m,
            resolution * options.voxel_leaf_size_ratio);
        KeyframeCloud::Ptr source_level = VoxelDownsample(
            job.current.cloud_body, level.voxel_leaf_size_m);
        KeyframeCloud::Ptr target_level =
            VoxelDownsample(target, level.voxel_leaf_size_m);
        level.source_points = source_level->size();
        level.target_points = target_level->size();

        if (source_level->size() < options.minimum_source_points)
        {
          level.elapsed_ms = MillisecondsSince(level_start);
          result.levels.push_back(level);
          result.status =
              LoopRegistrationStatus::kInsufficientSourcePoints;
          result.diagnostic = "downsampled source cloud is too small";
          result.registration_time_ms =
              MillisecondsSince(registration_start);
          return result;
        }
        if (target_level->size() < options.minimum_target_points)
        {
          level.elapsed_ms = MillisecondsSince(level_start);
          result.levels.push_back(level);
          result.status =
              LoopRegistrationStatus::kInsufficientTargetPoints;
          result.diagnostic = "downsampled target submap is too small";
          result.registration_time_ms =
              MillisecondsSince(registration_start);
          return result;
        }

        pcl::NormalDistributionsTransform<KeyframePoint, KeyframePoint> ndt;
        ndt.setTransformationEpsilon(options.transformation_epsilon);
        ndt.setStepSize(options.step_size);
        ndt.setMaximumIterations(options.maximum_iterations);
        ndt.setResolution(resolution);
        ndt.setInputSource(source_level);
        ndt.setInputTarget(target_level);
        KeyframeCloud aligned;
        ndt.align(aligned, estimate);
        estimate = ndt.getFinalTransformation();
        level.converged = ndt.hasConverged();
        level.iterations = ndt.getFinalNumIteration();
        level.transformation_probability = FiniteOr(
            ndt.getTransformationProbability(), 0.0);
        level.fitness_score_m2 = FiniteOr(
            ndt.getFitnessScore(
                options.overlap_max_correspondence_distance_m),
            std::numeric_limits<double>::max());
        level.elapsed_ms = MillisecondsSince(level_start);
        result.levels.push_back(level);
        if (level.converged) ++result.converged_levels;
        if (!estimate.allFinite())
        {
          result.status = LoopRegistrationStatus::kNonfiniteResult;
          result.diagnostic = "NDT produced a non-finite transform";
          result.registration_time_ms =
              MillisecondsSince(registration_start);
          return result;
        }
        final_source = std::move(source_level);
        final_target = std::move(target_level);
      }

      result.T_candidate_current = PoseFromMatrix(estimate);
      if (!result.T_candidate_current.isFinite() || !final_source ||
          !final_target)
      {
        result.status = LoopRegistrationStatus::kNonfiniteResult;
        result.diagnostic = "final NDT state is invalid";
        result.registration_time_ms = MillisecondsSince(registration_start);
        return result;
      }
      result.final_source_points = final_source->size();
      result.final_target_points = final_target->size();
      result.converged = result.levels.back().converged;
      result.transformation_probability =
          result.levels.back().transformation_probability;
      result.fitness_score_m2 = result.levels.back().fitness_score_m2;

      KeyframeCloud::Ptr aligned_source(new KeyframeCloud());
      pcl::transformPointCloud(*final_source, *aligned_source, estimate);
      pcl::KdTreeFLANN<KeyframePoint> target_tree;
      target_tree.setInputCloud(final_target);
      const double maximum_squared_distance =
          options.overlap_max_correspondence_distance_m *
          options.overlap_max_correspondence_distance_m;
      std::size_t overlap_points = 0;
      double squared_error_sum = 0.0;
      std::vector<int> nearest_index(1);
      std::vector<float> nearest_squared_distance(1);
      for (const KeyframePoint &point : aligned_source->points)
      {
        if (target_tree.nearestKSearch(
                point, 1, nearest_index, nearest_squared_distance) > 0 &&
            nearest_squared_distance[0] <= maximum_squared_distance)
        {
          ++overlap_points;
          squared_error_sum += nearest_squared_distance[0];
        }
      }
      result.overlap = aligned_source->empty()
                           ? 0.0
                           : static_cast<double>(overlap_points) /
                                 static_cast<double>(aligned_source->size());
      result.overlap_rmse_m =
          overlap_points == 0U
              ? 0.0
              : std::sqrt(squared_error_sum /
                          static_cast<double>(overlap_points));

      const Pose3d correction =
          result.T_candidate_current_initial.inverse() *
          result.T_candidate_current;
      result.correction_translation_m = correction.translation.norm();
      result.correction_angle_deg =
          result.T_candidate_current_initial.rotation.angularDistance(
              result.T_candidate_current.rotation) *
          180.0 / kPi;
      result.status = LoopRegistrationStatus::kCompleted;
    }
    catch (const std::exception &error)
    {
      result.status = LoopRegistrationStatus::kNdtException;
      result.diagnostic = error.what();
    }
    result.registration_time_ms = MillisecondsSince(registration_start);
    return result;
  }

  void WorkerLoop()
  {
    while (true)
    {
      RegistrationJob job;
      {
        std::unique_lock<std::mutex> lock(mutex);
        work_available.wait(
            lock, [this]() { return stop_requested || !queue.empty(); });
        if (stop_requested && queue.empty()) break;
        job = std::move(queue.front());
        queue.pop_front();
        worker_busy = true;
      }

      LoopRegistrationResult result = Compute(job);
      WriteCsv(result);

      ResultCallback result_callback;
      {
        std::lock_guard<std::mutex> lock(mutex);
        ++statistics.completed;
        statistics.total_registration_time_ms +=
            result.registration_time_ms;
        if (result.converged) ++statistics.converged;
        if (result.status != LoopRegistrationStatus::kCompleted)
          ++statistics.failed;
        result_callback = callback;
      }
      if (result_callback)
      {
        try
        {
          result_callback(result);
        }
        catch (...)
        {
          // A visualization/logging callback must never terminate the NDT
          // worker or prevent the remaining queued candidates from running.
        }
      }

      {
        std::lock_guard<std::mutex> lock(mutex);
        worker_busy = false;
        if (queue.empty()) idle.notify_all();
      }
    }
    std::lock_guard<std::mutex> lock(mutex);
    worker_busy = false;
    idle.notify_all();
  }

  void WriteCsv(const LoopRegistrationResult &result)
  {
    if (registration_csv.is_open())
    {
      const Pose3d &initial = result.T_candidate_current_initial;
      const Pose3d &final = result.T_candidate_current;
      registration_csv
          << std::setprecision(17) << result.candidate.current_id << ','
          << result.candidate.candidate_id << ','
          << LoopRegistrationStatusToString(result.status) << ','
          << static_cast<int>(result.converged) << ','
          << result.converged_levels << ',' << options.resolutions_m.size()
          << ',' << result.levels.size() << ',' << result.raw_source_points
          << ',' << result.raw_target_points << ','
          << result.final_source_points << ',' << result.final_target_points
          << ',' << result.transformation_probability << ','
          << result.fitness_score_m2 << ',' << result.overlap << ','
          << result.overlap_rmse_m << ',' << initial.translation.x() << ','
          << initial.translation.y() << ',' << initial.translation.z() << ','
          << initial.rotation.x() << ',' << initial.rotation.y() << ','
          << initial.rotation.z() << ',' << initial.rotation.w() << ','
          << final.translation.x() << ',' << final.translation.y() << ','
          << final.translation.z() << ',' << final.rotation.x() << ','
          << final.rotation.y() << ',' << final.rotation.z() << ','
          << final.rotation.w() << ',' << result.correction_translation_m
          << ',' << result.correction_angle_deg << ','
          << result.registration_time_ms << '\n';
      registration_csv.flush();
    }
    if (level_csv.is_open())
    {
      for (std::size_t index = 0; index < result.levels.size(); ++index)
      {
        const LoopRegistrationLevelResult &level = result.levels[index];
        level_csv << std::setprecision(17) << result.candidate.current_id
                  << ',' << result.candidate.candidate_id << ',' << index
                  << ',' << level.resolution_m << ','
                  << level.voxel_leaf_size_m << ',' << level.source_points
                  << ',' << level.target_points << ','
                  << static_cast<int>(level.converged) << ','
                  << level.iterations << ','
                  << level.transformation_probability << ','
                  << level.fitness_score_m2 << ',' << level.elapsed_ms
                  << '\n';
      }
      level_csv.flush();
    }
  }

  Options options;
  mutable std::mutex mutex;
  std::condition_variable work_available;
  std::condition_variable idle;
  std::deque<RegistrationJob> queue;
  bool worker_busy = false;
  bool stop_requested = false;
  std::thread worker;
  ResultCallback callback;
  Statistics statistics;
  std::ofstream registration_csv;
  std::ofstream level_csv;
};

LoopRegistration::LoopRegistration(const Options &options)
    : impl_(std::make_unique<Impl>(options))
{
}

LoopRegistration::~LoopRegistration() = default;

LoopRegistrationResult LoopRegistration::Register(
    const LoopCandidate &candidate,
    const std::vector<Keyframe::Ptr> &keyframes) const
{
  return impl_->Register(candidate, keyframes);
}

bool LoopRegistration::Enqueue(
    const LoopCandidate &candidate,
    const std::vector<Keyframe::Ptr> &keyframes)
{
  return impl_->Enqueue(candidate, keyframes);
}

void LoopRegistration::SetResultCallback(ResultCallback callback)
{
  impl_->SetResultCallback(std::move(callback));
}

void LoopRegistration::WaitUntilIdle()
{
  impl_->WaitUntilIdle();
}

LoopRegistration::Statistics LoopRegistration::statistics() const
{
  return impl_->GetStatistics();
}

}  // namespace my_livo::backend
