#include "backend/loop_verifier.h"

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
using my_livo::backend::LoopCandidate;
using my_livo::backend::LoopRegistrationResult;
using my_livo::backend::LoopRegistrationStatus;
using my_livo::backend::LoopRejectReason;
using my_livo::backend::LoopVerifier;
using my_livo::backend::Pose3d;

void Require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

Pose3d Translation(double x)
{
  return Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, 0.0, 0.0));
}

LoopRegistrationResult MakeRegistration(std::uint64_t historical_id,
                                        std::uint64_t current_id,
                                        double historical_x,
                                        double current_x)
{
  LoopRegistrationResult result;
  result.candidate.candidate_id = historical_id;
  result.candidate.current_id = current_id;
  result.status = LoopRegistrationStatus::kCompleted;
  result.converged = true;
  result.converged_levels = 4;
  result.transformation_probability = 0.55;
  result.fitness_score_m2 = 0.06;
  result.overlap = 0.98;
  result.overlap_rmse_m = 0.24;
  result.correction_translation_m = 0.05;
  result.correction_angle_deg = 0.10;
  result.T_map_candidate_snapshot = Translation(historical_x);
  result.T_map_current_snapshot = Translation(current_x);
  result.T_candidate_current = Translation(current_x - historical_x);
  result.T_candidate_current_initial = result.T_candidate_current;
  return result;
}

std::size_t CountLines(const std::filesystem::path &path)
{
  std::ifstream stream(path);
  Require(stream.good(), "loop-verification CSV was not created");
  std::size_t lines = 0;
  std::string line;
  while (std::getline(stream, line)) ++lines;
  return lines;
}

void TestIndividualGatesAndNeighborConsistency()
{
  const auto csv_path = std::filesystem::temp_directory_path() /
                        "my_livo_loop_verifier_test.csv";
  LoopVerifier::Options options;
  options.csv_path = csv_path.string();
  {
    LoopVerifier verifier(options);
    const auto first = MakeRegistration(10, 100, 10.0, 100.0);
    const auto second = MakeRegistration(20, 120, 20.0, 120.0);
    Require(verifier.Add(first).empty(),
            "an unsupported registration was accepted immediately");
    const auto supported = verifier.Add(second);
    Require(supported.size() == 2 && supported[0].accepted &&
                supported[1].accepted,
            "two cycle-consistent neighboring loops were not accepted");
    for (const auto &decision : supported)
    {
      Require(decision.consistent_neighbors >= 1,
              "accepted loop has no consistency evidence");
      Require(decision.best_neighbor_translation_error_m < 1.0e-12,
              "exact neighboring loops have a translation inconsistency");
      Require(decision.best_neighbor_rotation_error_deg < 1.0e-9,
              "exact neighboring loops have a rotation inconsistency");
    }

    auto poor = MakeRegistration(30, 140, 30.0, 140.0);
    poor.transformation_probability = 0.20;
    const auto rejected = verifier.Add(poor);
    Require(rejected.size() == 1 && !rejected.front().accepted &&
                rejected.front().reject_reason ==
                    LoopRejectReason::kProbabilityTooLow,
            "low-probability loop was not rejected with the correct reason");

    const auto isolated = MakeRegistration(200, 400, 200.0, 400.0);
    const auto expired = verifier.Add(isolated);
    Require(expired.empty(), "isolated loop should remain pending");
    const auto flushed = verifier.Flush();
    Require(flushed.size() == 1 && !flushed.front().accepted &&
                flushed.front().reject_reason ==
                    LoopRejectReason::kNoConsistentNeighbor,
            "isolated loop was not rejected at flush");

    const auto statistics = verifier.statistics();
    Require(statistics.registrations == 4 && statistics.accepted == 2 &&
                statistics.rejected == 2 && statistics.pending == 0,
            "loop-verification statistics are inconsistent");
  }
  Require(CountLines(csv_path) == 5,
          "loop-verification CSV must contain one row per decision");
  std::filesystem::remove(csv_path);
}

void TestConfigurableMultipleNeighborEvidence()
{
  LoopVerifier::Options options;
  options.minimum_consistent_neighbors = 2;
  LoopVerifier verifier(options);
  Require(verifier.Add(MakeRegistration(10, 100, 10.0, 100.0)).empty(),
          "first loop unexpectedly resolved");
  Require(verifier.Add(MakeRegistration(20, 120, 20.0, 120.0)).empty(),
          "one neighbor unexpectedly satisfied a two-neighbor policy");
  const auto decisions =
      verifier.Add(MakeRegistration(30, 140, 30.0, 140.0));
  Require(decisions.size() == 3,
          "three mutually consistent loops did not resolve together");
  for (const auto &decision : decisions)
    Require(decision.accepted && decision.consistent_neighbors >= 2,
            "multi-neighbor policy accepted insufficient evidence");
}
}  // namespace

int main()
{
  try
  {
    TestIndividualGatesAndNeighborConsistency();
    TestConfigurableMultipleNeighborEvidence();
  }
  catch (const std::exception &error)
  {
    std::cerr << "loop_verifier_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "loop_verifier_test passed\n";
  return 0;
}
