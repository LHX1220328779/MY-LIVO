#include "backend/loop_candidate_detector.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

namespace my_livo::backend
{
namespace
{
struct RankedCandidate
{
  Keyframe::Ptr keyframe;
  double planar_distance_m = 0.0;
  double height_difference_m = 0.0;
  double translation_distance_m = 0.0;
};

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

LoopCandidateDetector::LoopCandidateDetector(const Options &options)
    : options_(options)
{
  ValidateOptions(options_);
  OpenCsv(
      options_.detection_csv_path, &detection_csv_stream_,
      "current_id,current_timestamp,checked,history_keyframes,"
      "eligible_history,nearby_history,candidates",
      "loop-detection summary");
  OpenCsv(
      options_.candidate_csv_path, &candidate_csv_stream_,
      "current_id,candidate_id,current_timestamp,candidate_timestamp,"
      "id_separation,time_separation_sec,planar_distance_m,"
      "height_difference_m,translation_distance_m,initial_tx,initial_ty,"
      "initial_tz,initial_qx,initial_qy,initial_qz,initial_qw",
      "loop-candidate");
}

void LoopCandidateDetector::ValidateOptions(const Options &options)
{
  if (options.check_interval_keyframes == 0U)
    throw std::invalid_argument(
        "Loop-candidate check interval must be positive.");
  if (options.minimum_id_separation_keyframes == 0U)
    throw std::invalid_argument(
        "Loop-candidate minimum ID separation must be positive.");
  if (!std::isfinite(options.minimum_time_separation_sec) ||
      options.minimum_time_separation_sec < 0.0)
    throw std::invalid_argument(
        "Loop-candidate minimum time separation must be finite and "
        "non-negative.");
  if (!std::isfinite(options.maximum_planar_distance_m) ||
      options.maximum_planar_distance_m <= 0.0)
    throw std::invalid_argument(
        "Loop-candidate planar distance must be finite and positive.");
  if (!std::isfinite(options.maximum_height_difference_m) ||
      options.maximum_height_difference_m < 0.0)
    throw std::invalid_argument(
        "Loop-candidate height difference must be finite and non-negative.");
  if (options.minimum_candidate_id_separation_keyframes == 0U)
    throw std::invalid_argument(
        "Loop-candidate suppression separation must be positive.");
  if (options.maximum_candidates_per_keyframe == 0U)
    throw std::invalid_argument(
        "Loop-candidate maximum candidate count must be positive.");
  if (!options.detection_csv_path.empty() &&
      options.detection_csv_path == options.candidate_csv_path)
    throw std::invalid_argument(
        "Loop detection and candidate CSV paths must be different.");
}

LoopCandidateDetector::DetectionResult LoopCandidateDetector::AddKeyframe(
    const Keyframe::Ptr &keyframe)
{
  if (!keyframe)
    throw std::invalid_argument(
        "Cannot add a null keyframe to loop-candidate detector.");
  std::lock_guard<std::mutex> lock(mutex_);
  if (keyframe->id() != keyframes_.size())
    throw std::logic_error(
        "Loop-candidate keyframe IDs must be contiguous and start at zero.");
  if (!keyframes_.empty() &&
      keyframe->timestamp() <= keyframes_.back()->timestamp())
    throw std::logic_error(
        "Loop-candidate keyframe timestamps must be strictly increasing.");
  if (!keyframe->T_map_body().isFinite())
    throw std::invalid_argument(
        "Loop-candidate keyframe optimized pose is not finite.");

  DetectionResult result;
  result.current_id = keyframe->id();
  result.history_keyframes = keyframes_.size();
  const bool old_enough =
      keyframe->id() >= options_.minimum_id_separation_keyframes;
  const bool interval_elapsed =
      !has_checked_keyframe_ ||
      keyframe->id() - last_checked_id_ >=
          options_.check_interval_keyframes;

  if (old_enough && interval_elapsed)
  {
    result.checked = true;
    has_checked_keyframe_ = true;
    last_checked_id_ = keyframe->id();
    ++statistics_.detection_checks;

    const Pose3d current_pose = keyframe->T_map_body();
    std::vector<RankedCandidate> nearby;
    for (const auto &historical : keyframes_)
    {
      const std::uint64_t id_separation =
          keyframe->id() - historical->id();
      const double time_separation =
          keyframe->timestamp() - historical->timestamp();
      if (id_separation < options_.minimum_id_separation_keyframes ||
          time_separation < options_.minimum_time_separation_sec)
        continue;

      ++result.eligible_history;
      const Pose3d historical_pose = historical->T_map_body();
      const Eigen::Vector3d delta =
          current_pose.translation - historical_pose.translation;
      const double planar_distance = delta.head<2>().norm();
      const double height_difference = std::abs(delta.z());
      if (planar_distance > options_.maximum_planar_distance_m ||
          height_difference > options_.maximum_height_difference_m)
        continue;

      ++result.nearby_history;
      nearby.push_back(RankedCandidate{
          historical, planar_distance, height_difference, delta.norm()});
    }

    std::stable_sort(
        nearby.begin(), nearby.end(),
        [](const RankedCandidate &left, const RankedCandidate &right) {
          if (left.planar_distance_m != right.planar_distance_m)
            return left.planar_distance_m < right.planar_distance_m;
          return left.keyframe->id() < right.keyframe->id();
        });

    for (const RankedCandidate &ranked : nearby)
    {
      bool separated = true;
      for (const LoopCandidate &selected : result.candidates)
      {
        const std::uint64_t larger =
            std::max(ranked.keyframe->id(), selected.candidate_id);
        const std::uint64_t smaller =
            std::min(ranked.keyframe->id(), selected.candidate_id);
        if (larger - smaller <
            options_.minimum_candidate_id_separation_keyframes)
        {
          separated = false;
          break;
        }
      }
      if (!separated) continue;

      const Pose3d candidate_pose = ranked.keyframe->T_map_body();
      LoopCandidate candidate;
      candidate.current_id = keyframe->id();
      candidate.candidate_id = ranked.keyframe->id();
      candidate.current_timestamp = keyframe->timestamp();
      candidate.candidate_timestamp = ranked.keyframe->timestamp();
      candidate.id_separation = keyframe->id() - ranked.keyframe->id();
      candidate.time_separation_sec =
          keyframe->timestamp() - ranked.keyframe->timestamp();
      candidate.planar_distance_m = ranked.planar_distance_m;
      candidate.height_difference_m = ranked.height_difference_m;
      candidate.translation_distance_m = ranked.translation_distance_m;
      candidate.T_candidate_current_initial =
          candidate_pose.inverse() * current_pose;
      result.candidates.push_back(candidate);
      if (result.candidates.size() >=
          options_.maximum_candidates_per_keyframe)
        break;
    }
  }

  ++statistics_.keyframes;
  statistics_.eligible_history += result.eligible_history;
  statistics_.nearby_history += result.nearby_history;
  statistics_.candidates += result.candidates.size();
  WriteDetectionCsv(*keyframe, result);
  for (const LoopCandidate &candidate : result.candidates)
    WriteCandidateCsv(candidate);
  keyframes_.push_back(keyframe);
  return result;
}

LoopCandidateDetector::Statistics LoopCandidateDetector::statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return statistics_;
}

void LoopCandidateDetector::WriteDetectionCsv(
    const Keyframe &keyframe, const DetectionResult &result)
{
  if (!detection_csv_stream_.is_open()) return;
  detection_csv_stream_ << std::setprecision(17) << keyframe.id() << ','
                        << keyframe.timestamp() << ','
                        << static_cast<int>(result.checked) << ','
                        << result.history_keyframes << ','
                        << result.eligible_history << ','
                        << result.nearby_history << ','
                        << result.candidates.size() << '\n';
  detection_csv_stream_.flush();
}

void LoopCandidateDetector::WriteCandidateCsv(
    const LoopCandidate &candidate)
{
  if (!candidate_csv_stream_.is_open()) return;
  const Pose3d &initial = candidate.T_candidate_current_initial;
  candidate_csv_stream_
      << std::setprecision(17) << candidate.current_id << ','
      << candidate.candidate_id << ',' << candidate.current_timestamp << ','
      << candidate.candidate_timestamp << ',' << candidate.id_separation
      << ',' << candidate.time_separation_sec << ','
      << candidate.planar_distance_m << ',' << candidate.height_difference_m
      << ',' << candidate.translation_distance_m << ','
      << initial.translation.x() << ',' << initial.translation.y() << ','
      << initial.translation.z() << ',' << initial.rotation.x() << ','
      << initial.rotation.y() << ',' << initial.rotation.z() << ','
      << initial.rotation.w() << '\n';
  candidate_csv_stream_.flush();
}

}  // namespace my_livo::backend
