#ifndef MY_LIVO_BACKEND_CORRECTION_FEASIBILITY_MONITOR_H
#define MY_LIVO_BACKEND_CORRECTION_FEASIBILITY_MONITOR_H

#include <Eigen/Core>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace my_livo::backend
{

enum class CorrectionRegime
{
  kWarmup,
  kElastic,
  kDegraded,
  kRelocalizationRequired,
};

const char *CorrectionRegimeToString(CorrectionRegime regime);

// Distinguishes a correction that can be distributed smoothly along distance
// from a trajectory failure that no locally rigid elastic field can absorb.
// This is deliberately independent of RTK health: every input has already
// passed status/time gating and a large residual never makes RTK less trusted.
class CorrectionFeasibilityMonitor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options
  {
    double minimum_baseline_m = 20.0;
    double elastic_gradient_limit_m_per_m = 0.02;
    double relocalization_gradient_m_per_m = 0.15;
    int relocalization_consecutive_observations = 2;
    std::string csv_path;
  };

  struct Result
  {
    std::uint64_t keyframe_id = 0;
    CorrectionRegime regime = CorrectionRegime::kWarmup;
    bool state_changed = false;
    bool baseline_available = false;
    bool elastic_correction_allowed = false;
    double residual_m = 0.0;
    double baseline_m = 0.0;
    double correction_gradient_m_per_m = 0.0;
    double planar_gradient_m_per_m = 0.0;
    double vertical_gradient_m_per_m = 0.0;
    double chord_scale_ratio = 1.0;
    double heading_disagreement_deg = 0.0;
  };

  struct Statistics
  {
    std::uint64_t observations = 0;
    std::uint64_t warmup = 0;
    std::uint64_t elastic = 0;
    std::uint64_t degraded = 0;
    std::uint64_t relocalization_required = 0;
    double maximum_gradient_m_per_m = 0.0;
  };

  explicit CorrectionFeasibilityMonitor(const Options &options);

  Result AddObservation(std::uint64_t keyframe_id, double timestamp,
                        double cumulative_distance_m,
                        const Eigen::Vector3d &aligned_local_position,
                        const Eigen::Vector3d &rtk_position);

  Statistics statistics() const { return statistics_; }
  CorrectionRegime regime() const { return regime_; }

private:
  struct Sample
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    double timestamp = 0.0;
    double cumulative_distance_m = 0.0;
    Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d rtk_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  };

  static void ValidateOptions(const Options &options);
  void WriteCsv(const Result &result, const Sample &sample,
                std::uint64_t baseline_keyframe_id);

  Options options_;
  std::vector<Sample, Eigen::aligned_allocator<Sample>> samples_;
  CorrectionRegime regime_ = CorrectionRegime::kWarmup;
  int consecutive_relocalization_evidence_ = 0;
  Statistics statistics_;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_CORRECTION_FEASIBILITY_MONITOR_H
