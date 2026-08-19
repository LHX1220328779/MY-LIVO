#include "backend/rtk_observation_buffer.h"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

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
}  // namespace

int main()
{
  try
  {
    TestStatusParser();
    TestExactAndMidpoint();
    TestInvalidStatusGapAndBracket();
    TestStatusDropoutAndRecovery();
    TestSelectorFrequencyInvariant();
    std::cout << "rtk_observation_buffer_test passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "rtk_observation_buffer_test failed: " << error.what() << '\n';
    return 1;
  }
}
