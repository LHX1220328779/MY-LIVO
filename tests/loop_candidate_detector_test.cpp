#include "backend/loop_candidate_detector.h"

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using my_livo::backend::Keyframe;
using my_livo::backend::KeyframeCloud;
using my_livo::backend::KeyframePoint;
using my_livo::backend::KeyframeTrigger;
using my_livo::backend::LoopCandidateDetector;
using my_livo::backend::Matrix6d;
using my_livo::backend::Pose3d;

void Require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

Keyframe::Ptr MakeKeyframe(std::uint64_t id, double x, double z = 0.0)
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  KeyframePoint point;
  point.x = 1.0F;
  point.intensity = 1.0F;
  cloud->push_back(point);
  return std::make_shared<Keyframe>(
      id, static_cast<double>(id),
      Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, 0.0, z)),
      cloud, Matrix6d::Identity(),
      static_cast<std::uint8_t>(
          id == 0U ? KeyframeTrigger::kFirst
                   : KeyframeTrigger::kTranslation));
}

std::size_t CountLines(const std::filesystem::path &path)
{
  std::ifstream stream(path);
  Require(stream.good(), "expected CSV was not created");
  std::size_t lines = 0;
  std::string line;
  while (std::getline(stream, line)) ++lines;
  return lines;
}

void TestDetectionCadenceRankingAndSuppression()
{
  const std::filesystem::path temporary =
      std::filesystem::temp_directory_path();
  const auto detection_path = temporary / "my_livo_loop_detection_test.csv";
  const auto candidate_path = temporary / "my_livo_loop_candidates_test.csv";

  LoopCandidateDetector::Options options;
  options.check_interval_keyframes = 2;
  options.minimum_id_separation_keyframes = 4;
  options.minimum_time_separation_sec = 3.0;
  options.maximum_planar_distance_m = 2.0;
  options.maximum_height_difference_m = 1.0;
  options.minimum_candidate_id_separation_keyframes = 2;
  options.maximum_candidates_per_keyframe = 2;
  options.detection_csv_path = detection_path.string();
  options.candidate_csv_path = candidate_path.string();

  {
    LoopCandidateDetector detector(options);
    const std::vector<double> positions{0.0, 50.0, 1.0, 50.0,
                                        0.2, 50.0, 0.9};
    std::vector<LoopCandidateDetector::DetectionResult> results;
    for (std::uint64_t id = 0; id < positions.size(); ++id)
      results.push_back(detector.AddKeyframe(MakeKeyframe(id, positions[id])));

    Require(!results[3].checked,
            "detector checked before minimum history was available");
    Require(results[4].checked && results[4].candidates.size() == 1,
            "first eligible loop check did not return the expected candidate");
    Require(results[4].candidates.front().candidate_id == 0,
            "first loop candidate ID is incorrect");
    Require(std::abs(
                results[4].candidates.front()
                    .T_candidate_current_initial.translation.x() -
                0.2) < 1.0e-12,
            "candidate-to-current initial transform direction is incorrect");
    Require(!results[5].checked,
            "loop detector ignored the configured check interval");
    Require(results[6].checked && results[6].candidates.size() == 2,
            "ranked/suppressed loop candidate count is incorrect");
    Require(results[6].candidates[0].candidate_id == 2 &&
                results[6].candidates[1].candidate_id == 0,
            "loop candidates were not sorted by planar distance");
    Require(results[6].candidates[0].planar_distance_m <=
                results[6].candidates[1].planar_distance_m,
            "loop candidate distance order is incorrect");

    const auto statistics = detector.statistics();
    Require(statistics.keyframes == positions.size(),
            "loop detector keyframe count is incorrect");
    Require(statistics.detection_checks == 2,
            "loop detector check count is incorrect");
    Require(statistics.candidates == 3,
            "loop detector candidate count is incorrect");
  }

  Require(CountLines(detection_path) == 8,
          "loop detection CSV must contain one row per keyframe");
  Require(CountLines(candidate_path) == 4,
          "loop candidate CSV row count is incorrect");
  std::filesystem::remove(detection_path);
  std::filesystem::remove(candidate_path);
}

void TestHeightAndIdInvariants()
{
  LoopCandidateDetector::Options options;
  options.check_interval_keyframes = 1;
  options.minimum_id_separation_keyframes = 2;
  options.minimum_time_separation_sec = 0.0;
  options.maximum_planar_distance_m = 5.0;
  options.maximum_height_difference_m = 0.5;
  options.minimum_candidate_id_separation_keyframes = 1;
  options.maximum_candidates_per_keyframe = 2;
  LoopCandidateDetector detector(options);
  (void)detector.AddKeyframe(MakeKeyframe(0, 0.0, 0.0));
  (void)detector.AddKeyframe(MakeKeyframe(1, 20.0, 0.0));
  const auto result = detector.AddKeyframe(MakeKeyframe(2, 0.1, 2.0));
  Require(result.checked && result.nearby_history == 0 &&
              result.candidates.empty(),
          "height gate accepted an implausible loop candidate");

  bool rejected = false;
  try
  {
    (void)detector.AddKeyframe(MakeKeyframe(4, 0.0));
  }
  catch (const std::logic_error &)
  {
    rejected = true;
  }
  Require(rejected, "non-contiguous loop-detector ID was not rejected");
}
}  // namespace

int main()
{
  try
  {
    TestDetectionCadenceRankingAndSuppression();
    TestHeightAndIdInvariants();
  }
  catch (const std::exception &error)
  {
    std::cerr << "loop_candidate_detector_test failed: " << error.what()
              << '\n';
    return 1;
  }
  std::cout << "loop_candidate_detector_test passed\n";
  return 0;
}
