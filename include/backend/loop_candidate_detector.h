#ifndef MY_LIVO_BACKEND_LOOP_CANDIDATE_DETECTOR_H
#define MY_LIVO_BACKEND_LOOP_CANDIDATE_DETECTOR_H

#include "backend/keyframe.h"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace my_livo::backend
{

// A geometrically plausible historical keyframe for later registration.
// T_candidate_current_initial maps current-body points into candidate-body.
// It is only an initial estimate; this stage never inserts a graph factor.
struct LoopCandidate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t current_id = 0;
  std::uint64_t candidate_id = 0;
  double current_timestamp = 0.0;
  double candidate_timestamp = 0.0;
  std::uint64_t id_separation = 0;
  double time_separation_sec = 0.0;
  double planar_distance_m = 0.0;
  double height_difference_m = 0.0;
  double translation_distance_m = 0.0;
  Pose3d T_candidate_current_initial;
};

// Lightweight first-stage loop detector adapted from lightning-lm's
// DetectLoopCandidates(). It uses optimized graph positions for spatial
// gating, adds time/height gates, ranks by distance, and performs keyframe-ID
// non-maximum suppression before returning a bounded candidate set.
class LoopCandidateDetector
{
public:
  struct Options
  {
    std::uint64_t check_interval_keyframes = 20;
    std::uint64_t minimum_id_separation_keyframes = 50;
    double minimum_time_separation_sec = 30.0;
    double maximum_planar_distance_m = 20.0;
    double maximum_height_difference_m = 5.0;
    std::uint64_t minimum_candidate_id_separation_keyframes = 20;
    std::size_t maximum_candidates_per_keyframe = 3;
    std::string detection_csv_path;
    std::string candidate_csv_path;
  };

  struct DetectionResult
  {
    std::uint64_t current_id = 0;
    bool checked = false;
    std::size_t history_keyframes = 0;
    std::size_t eligible_history = 0;
    std::size_t nearby_history = 0;
    std::vector<LoopCandidate> candidates;
  };

  struct Statistics
  {
    std::uint64_t keyframes = 0;
    std::uint64_t detection_checks = 0;
    std::uint64_t eligible_history = 0;
    std::uint64_t nearby_history = 0;
    std::uint64_t candidates = 0;
  };

  explicit LoopCandidateDetector(const Options &options);

  DetectionResult AddKeyframe(const Keyframe::Ptr &keyframe);
  Statistics statistics() const;

private:
  static void ValidateOptions(const Options &options);
  void WriteDetectionCsv(const Keyframe &keyframe,
                         const DetectionResult &result);
  void WriteCandidateCsv(const LoopCandidate &candidate);

  Options options_;
  mutable std::mutex mutex_;
  std::vector<Keyframe::Ptr> keyframes_;
  Statistics statistics_;
  bool has_checked_keyframe_ = false;
  std::uint64_t last_checked_id_ = 0;
  std::ofstream detection_csv_stream_;
  std::ofstream candidate_csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_LOOP_CANDIDATE_DETECTOR_H
