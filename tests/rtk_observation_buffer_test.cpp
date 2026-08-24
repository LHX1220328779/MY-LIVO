#include "backend/rtk_observation_buffer.h"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using namespace my_livo::backend;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

diagnostic_msgs::msg::DiagnosticArray Status(const std::string &value)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  diagnostic_msgs::msg::DiagnosticStatus status;
  diagnostic_msgs::msg::KeyValue unrelated;
  unrelated.key = "gps_num_sats";
  unrelated.value = "40";
  status.values.push_back(unrelated);
  diagnostic_msgs::msg::KeyValue mode;
  mode.key = "ins_pos_mode";
  mode.value = value;
  status.values.push_back(mode);
  message.status.push_back(status);
  return message;
}

RtkSolution Solution(double time, double x)
{
  RtkSolution solution;
  solution.timestamp = time;
  solution.position = Eigen::Vector3d(x, 2.0 * x, -x);
  solution.orientation = Eigen::Quaterniond::Identity();
  return solution;
}

void TestStatusParser()
{
  Require(ParseInsPosMode(Status("4")) == 4, "mode 4 did not parse");
  Require(ParseInsPosMode(Status(" 0 ")) == 0, "mode 0 did not parse");
  Require(!ParseInsPosMode(Status("4x")), "bad mode parsed");
  diagnostic_msgs::msg::DiagnosticArray missing;
  Require(!ParseInsPosMode(missing), "missing mode parsed");
  auto conflicting = Status("4");
  conflicting.status.push_back(Status("0").status.front());
  Require(!ParseInsPosMode(conflicting), "conflicting modes parsed");
}

RtkObservationBuffer MakeBuffer()
{
  RtkObservationBuffer::Options options;
  options.maximum_status_age_sec = 0.1;
  options.maximum_interpolation_gap_sec = 1.1;
  options.maximum_endpoint_distance_sec = 1.0;
  options.retention_sec = 3.0;
  return RtkObservationBuffer(options);
}

void TestExactAndMidpoint()
{
  auto buffer = MakeBuffer();
  buffer.AddStatus(0.0, 4);
  buffer.AddStatus(1.0, 4);
  buffer.AddSolution(Solution(0.0, 0.0));
  buffer.AddSolution(Solution(1.0, 10.0));
  const auto exact = buffer.GetObservationAt(0.0);
  Require(exact.observation && exact.observation->position.norm() < 1.0e-12,
          "exact timestamp failed");
  const auto midpoint = buffer.GetObservationAt(0.5);
  Require(midpoint.observation &&
          (midpoint.observation->position - Eigen::Vector3d(5, 10, -5)).norm()
              < 1.0e-12,
          "midpoint interpolation failed");
  Require(std::abs(std::sqrt(
              midpoint.observation->position_covariance(0, 0)) - 0.30) < 1e-12,
          "configured covariance was not applied");
  Require(midpoint.observation->health == 1.0 &&
              midpoint.observation->lower_ins_pos_mode == 4 &&
              midpoint.observation->upper_ins_pos_mode == 4,
          "accepted observation health was not preserved");
}

void TestRawLeverArmAndReportedCovarianceInterpolation()
{
  auto buffer = MakeBuffer();
  buffer.AddStatus(0.0, 4);
  buffer.AddStatus(1.0, 4);
  RtkSolution lower = Solution(0.0, 0.0);
  lower.has_raw_position = true;
  lower.raw_position = Eigen::Vector3d(-1.0, -2.0, -3.0);
  lower.lever_arm_correction = lower.position - lower.raw_position;
  lower.position_covariance.diagonal() << 0.04, 0.09, 0.16;
  lower.has_velocity = true;
  lower.velocity = Eigen::Vector3d(2.0, 3.0, 4.0);
  lower.velocity_covariance.diagonal() << 0.01, 0.04, 0.09;
  RtkSolution upper = Solution(1.0, 2.0);
  upper.has_raw_position = true;
  upper.raw_position = Eigen::Vector3d(1.0, 2.0, -5.0);
  upper.lever_arm_correction = upper.position - upper.raw_position;
  upper.position_covariance.diagonal() << 0.16, 0.25, 0.36;
  upper.has_velocity = true;
  upper.velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  upper.velocity_covariance.diagonal() << 0.09, 0.16, 0.25;
  buffer.AddSolution(lower);
  buffer.AddSolution(upper);

  const auto midpoint = buffer.GetObservationAt(0.5);
  Require(midpoint.observation.has_value(),
          "raw lever-arm midpoint observation failed");
  Require((midpoint.observation->raw_position -
           Eigen::Vector3d(0.0, 0.0, -4.0)).norm() < 1.0e-12,
          "raw RTK position was not interpolated");
  Require((midpoint.observation->lever_arm_correction -
           Eigen::Vector3d(1.0, 2.0, 3.0)).norm() < 1.0e-12,
          "lever-arm correction was not interpolated");
  Require(std::abs(
              midpoint.observation->reported_position_covariance(0, 0) -
              0.10) < 1.0e-12,
          "reported covariance was not preserved separately");
  Require(midpoint.observation->has_velocity &&
              (midpoint.observation->velocity -
               Eigen::Vector3d(3.0, 4.0, 5.0)).norm() < 1.0e-12 &&
              std::abs(midpoint.observation->
                           reported_velocity_covariance(0, 0) - 0.05) <
                  1.0e-12,
          "receiver velocity/covariance was not interpolated");
}

void TestInvalidStatusGapAndBracket()
{
  {
    auto buffer = MakeBuffer();
    buffer.AddStatus(0.0, 4);
    buffer.AddStatus(1.0, 0);
    buffer.AddSolution(Solution(0.0, 0.0));
    buffer.AddSolution(Solution(1.0, 1.0));
    Require(buffer.GetObservationAt(0.5).reason ==
                RtkObservationRejectReason::kStatusNotEligible,
            "invalid status interval was crossed");
  }
  {
    RtkObservationBuffer::Options options;
    options.maximum_status_age_sec = 0.1;
    options.maximum_interpolation_gap_sec = 0.2;
    options.maximum_endpoint_distance_sec = 1.0;
    auto buffer = RtkObservationBuffer(options);
    buffer.AddStatus(0.0, 4);
    buffer.AddStatus(1.0, 4);
    buffer.AddSolution(Solution(0.0, 0.0));
    buffer.AddSolution(Solution(1.0, 1.0));
    Require(buffer.GetObservationAt(0.5).reason ==
                RtkObservationRejectReason::kInterpolationGap,
            "large interpolation gap was accepted");
  }
  {
    auto buffer = MakeBuffer();
    buffer.AddStatus(0.0, 4);
    buffer.AddSolution(Solution(0.0, 0.0));
    Require(buffer.GetObservationAt(0.5).reason ==
                RtkObservationRejectReason::kNoBracket,
            "query without bracket was accepted");
  }
}

void TestStatusDropoutAndRecovery()
{
  auto buffer = MakeBuffer();
  buffer.AddStatus(0.0, 4);
  buffer.AddStatus(1.0, 0);
  buffer.AddStatus(2.0, 4);
  buffer.AddStatus(3.0, 4);
  buffer.AddSolution(Solution(0.0, 0.0));
  buffer.AddSolution(Solution(1.0, 1.0));
  buffer.AddSolution(Solution(2.0, 2.0));
  buffer.AddSolution(Solution(3.0, 3.0));
  Require(!buffer.GetObservationAt(0.5).observation,
          "factor crossed into RTK status loss");
  Require(!buffer.GetObservationAt(1.5).observation,
          "factor crossed out of RTK status loss");
  Require(buffer.GetObservationAt(2.5).observation.has_value(),
          "RTK did not recover after two valid bracket endpoints");
}

void TestSelectorFrequencyInvariant()
{
  RtkFactorSelector::Options options;
  options.minimum_time_interval_sec = 2.0;
  options.maximum_time_interval_sec = 10.0;
  options.minimum_translation_m = 5.0;
  RtkFactorSelector selector(options);
  for (std::uint64_t id = 0; id < 30; ++id)
  {
    RtkObservation observation;
    observation.timestamp = static_cast<double>(id);
    observation.position.x() = static_cast<double>(id);
    if (selector.ShouldSelect(id, observation))
      selector.MarkSelected(id, observation);
  }
  Require(selector.selected_count() == 6,
          "selector count depends on message-rate assumptions");
}

void TestQueryCsvIsDurableWhileBufferIsAlive()
{
  const auto path = std::filesystem::temp_directory_path() /
                    "my_livo_rtk_query_flush_test.csv";
  std::filesystem::remove(path);
  RtkObservationBuffer::Options options;
  options.maximum_status_age_sec = 0.1;
  options.maximum_interpolation_gap_sec = 1.1;
  options.maximum_endpoint_distance_sec = 1.0;
  options.retention_sec = 3.0;
  options.query_csv_path = path.string();
  RtkObservationBuffer buffer(options);
  buffer.AddStatus(0.0, 4);
  buffer.AddStatus(1.0, 4);
  buffer.AddSolution(Solution(0.0, 0.0));
  buffer.AddSolution(Solution(1.0, 1.0));
  Require(buffer.GetObservationAt(0.5).observation.has_value(),
          "accepted query for CSV durability test failed");
  Require(!buffer.GetObservationAt(2.0).observation.has_value(),
          "rejected query for CSV durability test failed");

  // The producer deliberately remains alive while a second stream reads the
  // file.  This catches regressions where keyframe rows only reach disk from
  // the ofstream destructor.
  std::ifstream csv(path);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(csv, line)) lines.push_back(line);
  Require(lines.size() == 3,
          "RTK query CSV is not durable at keyframe cadence");
  const auto columns = [](const std::string &row) {
    return 1 + static_cast<int>(
        std::count(row.begin(), row.end(), ','));
  };
  Require(columns(lines[0]) == columns(lines[1]) &&
              columns(lines[0]) == columns(lines[2]),
          "accepted/rejected RTK query CSV schemas differ");
  std::filesystem::remove(path);
}

void TestSolutionVelocityCsvSchema()
{
  const auto path = std::filesystem::temp_directory_path() /
                    "my_livo_rtk_solution_velocity_test.csv";
  std::filesystem::remove(path);
  {
    RtkObservationBuffer::Options options;
    options.solution_csv_path = path.string();
    RtkObservationBuffer buffer(options);
    buffer.AddSolution(Solution(0.0, 0.0));
    RtkSolution velocity = Solution(1.0, 1.0);
    velocity.has_velocity = true;
    velocity.velocity = Eigen::Vector3d(1.0, 2.0, 3.0);
    buffer.AddSolution(velocity);
  }
  std::ifstream csv(path);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(csv, line)) lines.push_back(line);
  const auto columns = [](const std::string &row) {
    return 1 + static_cast<int>(
        std::count(row.begin(), row.end(), ','));
  };
  Require(lines.size() == 3 &&
              columns(lines[0]) == columns(lines[1]) &&
              columns(lines[0]) == columns(lines[2]),
          "RTK solution velocity CSV schemas differ");
  std::filesystem::remove(path);
}
}  // namespace

int main()
{
  try
  {
    TestStatusParser();
    TestExactAndMidpoint();
    TestRawLeverArmAndReportedCovarianceInterpolation();
    TestInvalidStatusGapAndBracket();
    TestStatusDropoutAndRecovery();
    TestSelectorFrequencyInvariant();
    TestQueryCsvIsDurableWhileBufferIsAlive();
    TestSolutionVelocityCsvSchema();
    std::cout << "rtk_observation_buffer_test passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "rtk_observation_buffer_test failed: " << error.what() << '\n';
    return 1;
  }
}
