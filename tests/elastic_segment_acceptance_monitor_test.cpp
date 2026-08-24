#include "backend/elastic_segment_acceptance_monitor.h"

#include <iostream>
#include <stdexcept>

namespace
{
void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
}

int main()
{
  try
  {
    using Monitor = my_livo::backend::ElasticSegmentAcceptanceMonitor;
    Monitor::Options options;
    options.minimum_position_observations = 4;
    options.minimum_path_length_m = 15.0;
    Monitor monitor(options);
    auto result = monitor.AddObservation(
        10, 100.0, 0.1, 0.2, 0.1, true, true, 0.1, 0.1);
    Require(!result.accepted, "probation accepted without a baseline");
    result = monitor.AddObservation(
        11, 105.0, 0.1, 0.2, 0.1, true, true, 0.1, 0.1);
    result = monitor.AddObservation(
        12, 110.0, 0.1, 0.2, 0.1, true, true, 0.1, 0.1);
    result = monitor.AddObservation(
        13, 115.0, 0.1, 0.2, 0.1, true, true, 0.1, 0.1);
    Require(result.accepted && result.state_changed,
            "valid elastic probation was not accepted");
    result = monitor.AddObservation(
        14, 120.0, 1.0, 2.0, 1.0, false, false, 1.0, 1.0);
    Require(result.accepted && !result.state_changed,
            "accepted elastic segment did not latch");
    monitor.Reset();
    result = monitor.AddObservation(
        20, 200.0, 0.31, 0.2, 0.1, true, true, 0.1, 0.1);
    Require(result.decision == Monitor::Decision::kRejected,
            "out-of-bound residual did not reset probation");
  }
  catch (const std::exception &error)
  {
    std::cerr << "elastic_segment_acceptance_monitor_test failed: "
              << error.what() << '\n';
    return 1;
  }
  std::cout << "elastic_segment_acceptance_monitor_test passed\n";
  return 0;
}
