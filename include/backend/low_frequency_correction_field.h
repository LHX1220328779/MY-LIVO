#ifndef MY_LIVO_BACKEND_LOW_FREQUENCY_CORRECTION_FIELD_H
#define MY_LIVO_BACKEND_LOW_FREQUENCY_CORRECTION_FIELD_H

#include "backend/correction_feasibility_monitor.h"
#include "backend/keyframe.h"

#include <Eigen/Core>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace my_livo::backend
{

enum class CorrectionFieldDecision
{
  kDisabled,
  kAnchor,
  kWarmup,
  kSpacing,
  kUpdated,
  kFrozenRelocalization,
};

const char *CorrectionFieldDecisionToString(CorrectionFieldDecision decision);

// A causal 4-DOF correction field parameterized by travelled distance.  The
// state is a body-position displacement plus yaw, which is equivalent to a
// path-centred SE(3) correction and avoids rotation-about-global-origin
// conditioning.  Cubic smoothstep interpolation is C1 continuous and its
// derivative is explicitly bounded.
class LowFrequencyCorrectionField
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options
  {
    bool enabled = true;
    double knot_spacing_m = 15.0;
    double spatial_low_pass_length_m = 40.0;
    double elastic_soft_radius_m = 0.10;
    double elastic_full_radius_m = 0.28;
    double elastic_minimum_stiffness = 0.05;
    double elastic_maximum_stiffness = 1.0;
    double maximum_planar_gradient_m_per_m = 0.005;
    double maximum_vertical_gradient_m_per_m = 0.003;
    double maximum_yaw_gradient_deg_per_m = 0.01;
    double maximum_planar_update_m = 0.05;
    double maximum_vertical_update_m = 0.03;
    double maximum_yaw_update_deg = 0.15;
    std::string csv_path;
  };

  struct Correction
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
    double yaw_rad = 0.0;
  };

  struct UpdateResult
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    CorrectionFieldDecision decision = CorrectionFieldDecision::kDisabled;
    bool field_changed = false;
    bool frozen = false;
    std::size_t knot_count = 0;
    double cumulative_distance_m = 0.0;
    double elastic_distance_m = 0.0;
    double elastic_stiffness = 0.0;
    double radial_force_proxy_m = 0.0;
    double residual_before_m = 0.0;
    double residual_after_m = 0.0;
    double planar_step_m = 0.0;
    double vertical_step_m = 0.0;
    double yaw_step_deg = 0.0;
    Correction correction;
  };

  struct Statistics
  {
    std::uint64_t observations = 0;
    std::uint64_t updates = 0;
    std::uint64_t frozen_observations = 0;
    std::size_t knots = 0;
  };

  explicit LowFrequencyCorrectionField(const Options &options);

  UpdateResult AddObservation(
      std::uint64_t keyframe_id, double timestamp,
      double cumulative_distance_m, const Eigen::Vector3d &base_position,
      const Eigen::Vector3d &rtk_position,
      const Eigen::Matrix3d &position_covariance,
      CorrectionRegime regime);

  Correction Evaluate(double cumulative_distance_m) const;
  Pose3d Apply(double cumulative_distance_m,
               const Pose3d &base_pose) const;
  double ElasticStiffness(double distance_m) const;
  Statistics statistics() const { return statistics_; }
  bool frozen() const { return frozen_; }

private:
  struct Knot
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t keyframe_id = 0;
    double timestamp = 0.0;
    double distance_m = 0.0;
    Eigen::Vector3d source_base_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_rtk_position = Eigen::Vector3d::Zero();
    Correction correction;
  };

  static void ValidateOptions(const Options &options);
  void AccumulateTarget(const Eigen::Vector3d &target,
                        const Eigen::Matrix3d &covariance);
  void ClearPending();
  void WriteCsv(double timestamp, CorrectionRegime regime,
                const UpdateResult &result);

  Options options_;
  std::vector<Knot, Eigen::aligned_allocator<Knot>> knots_;
  Eigen::Vector3d pending_weighted_target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d pending_weight_ = Eigen::Vector3d::Zero();
  bool frozen_ = false;
  Statistics statistics_;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_LOW_FREQUENCY_CORRECTION_FIELD_H
