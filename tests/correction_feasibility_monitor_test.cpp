#include "backend/correction_feasibility_monitor.h"

#include <Eigen/Core>

#include <iostream>
#include <stdexcept>

namespace
{
using my_livo::backend::CorrectionFeasibilityMonitor;
using my_livo::backend::CorrectionRegime;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

CorrectionFeasibilityMonitor::Result Add(
    CorrectionFeasibilityMonitor &monitor, std::uint64_t id,
    double distance, double required_correction)
{
  const Eigen::Vector3d local(distance, 0.0, 0.0);
  const Eigen::Vector3d rtk(distance - required_correction, 0.0, 0.0);
  return monitor.AddObservation(id, 100.0 + id, distance, local, rtk);
}
}  // namespace

int main()
{
  try
  {
    CorrectionFeasibilityMonitor::Options options;
    options.minimum_baseline_m = 20.0;
    options.elastic_gradient_limit_m_per_m = 0.02;
    options.relocalization_gradient_m_per_m = 0.15;
    options.relocalization_consecutive_observations = 2;
    CorrectionFeasibilityMonitor monitor(options);

    Require(Add(monitor, 0, 0.0, 0.0).regime ==
                CorrectionRegime::kWarmup,
            "first observation must be warmup");
    const auto elastic = Add(monitor, 1, 20.0, 0.2);
    Require(elastic.regime == CorrectionRegime::kElastic &&
                elastic.elastic_correction_allowed,
            "small correction gradient was not elastic");
    const auto degraded = Add(monitor, 2, 40.0, 2.2);
    Require(degraded.regime == CorrectionRegime::kDegraded &&
                degraded.elastic_correction_allowed,
            "bounded excessive gradient was not degraded");
    Require(Add(monitor, 3, 60.0, 6.2).regime ==
                CorrectionRegime::kDegraded,
            "single catastrophic sample should not latch");
    const auto relocation = Add(monitor, 4, 80.0, 10.2);
    Require(relocation.regime == CorrectionRegime::kRelocalizationRequired &&
                relocation.state_changed &&
                !relocation.elastic_correction_allowed,
            "sustained catastrophic gradient did not request relocalization");
    Require(Add(monitor, 5, 100.0, 10.3).regime ==
                CorrectionRegime::kRelocalizationRequired,
            "relocalization request must remain latched");

    const auto statistics = monitor.statistics();
    Require(statistics.observations == 6 &&
                statistics.relocalization_required == 2 &&
                statistics.maximum_gradient_m_per_m > 0.19,
            "correction monitor statistics are wrong");
  }
  catch (const std::exception &error)
  {
    std::cerr << "correction_feasibility_monitor_test failed: "
              << error.what() << '\n';
    return 1;
  }
  std::cout << "correction_feasibility_monitor_test passed\n";
  return 0;
}
