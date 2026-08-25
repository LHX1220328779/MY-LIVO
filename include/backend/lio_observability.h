#ifndef MY_LIVO_BACKEND_LIO_OBSERVABILITY_H
#define MY_LIVO_BACKEND_LIO_OBSERVABILITY_H

#include <Eigen/Core>

#include <cstddef>

namespace my_livo::backend
{

// Per-scan geometry diagnostics derived from the weighted point-to-plane
// information matrix. Counts alone cannot reveal directional degeneracy.
struct LioObservability
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  std::size_t downsampled_features = 0;
  std::size_t effective_features = 0;
  double effective_feature_ratio = 0.0;
  double mean_absolute_residual_m = 0.0;
  Eigen::Vector3d translation_information_eigenvalues =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_information_eigenvalues =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d weakest_translation_direction =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d weakest_rotation_direction =
      Eigen::Vector3d::Zero();
  double translation_condition_ratio = 0.0;
  double rotation_condition_ratio = 0.0;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_LIO_OBSERVABILITY_H
