#ifndef FAST_LIVO_MINIMAL_SOPHUS_SE3_H
#define FAST_LIVO_MINIMAL_SOPHUS_SE3_H

#include <Eigen/Core>

namespace Sophus
{

// FAST-LIVO2 only needs the rigid-transform subset of the legacy Sophus API.
class SE3
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3() : rotation_(Eigen::Matrix3d::Identity()), translation_(Eigen::Vector3d::Zero()) {}
  SE3(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
  : rotation_(rotation), translation_(translation) {}

  SE3 inverse() const
  {
    const Eigen::Matrix3d inverse_rotation = rotation_.transpose();
    return SE3(inverse_rotation, -inverse_rotation * translation_);
  }

  SE3 operator*(const SE3 &other) const
  {
    return SE3(rotation_ * other.rotation_, rotation_ * other.translation_ + translation_);
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d &point) const
  {
    return rotation_ * point + translation_;
  }

  const Eigen::Matrix3d &rotation_matrix() const { return rotation_; }
  const Eigen::Vector3d &translation() const { return translation_; }

private:
  Eigen::Matrix3d rotation_;
  Eigen::Vector3d translation_;
};

}  // namespace Sophus

#endif
