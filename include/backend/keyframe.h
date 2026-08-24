#ifndef MY_LIVO_BACKEND_KEYFRAME_H
#define MY_LIVO_BACKEND_KEYFRAME_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <mutex>

namespace my_livo::backend
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using KeyframePoint = pcl::PointXYZI;
using KeyframeCloud = pcl::PointCloud<KeyframePoint>;

// T_parent_child maps a point expressed in child into parent.
struct Pose3d
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  Pose3d() = default;
  Pose3d(const Eigen::Matrix3d &rotation_matrix,
         const Eigen::Vector3d &translation_vector)
      : rotation(rotation_matrix), translation(translation_vector)
  {
    rotation.normalize();
  }
  Pose3d(const Eigen::Quaterniond &rotation_quaternion,
         const Eigen::Vector3d &translation_vector)
      : rotation(rotation_quaternion), translation(translation_vector)
  {
    rotation.normalize();
  }

  bool isFinite() const
  {
    return rotation.coeffs().allFinite() && translation.allFinite() &&
           rotation.norm() > 1.0e-9;
  }

  Pose3d inverse() const
  {
    const Eigen::Quaterniond inverse_rotation = rotation.conjugate();
    return Pose3d(inverse_rotation, -(inverse_rotation * translation));
  }

  Pose3d operator*(const Pose3d &other) const
  {
    return Pose3d(rotation * other.rotation,
                  rotation * other.translation + translation);
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d &point) const
  {
    return rotation * point + translation;
  }
};

enum class KeyframeTrigger : std::uint8_t
{
  kNone = 0,
  kFirst = 1U << 0U,
  kTranslation = 1U << 1U,
  kRotation = 1U << 2U,
  kTime = 1U << 3U,
};

inline std::uint8_t operator|(KeyframeTrigger left, KeyframeTrigger right)
{
  return static_cast<std::uint8_t>(left) |
         static_cast<std::uint8_t>(right);
}

inline bool HasTrigger(std::uint8_t mask, KeyframeTrigger trigger)
{
  return (mask & static_cast<std::uint8_t>(trigger)) != 0U;
}

class Keyframe
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Keyframe>;
  using ConstPtr = std::shared_ptr<const Keyframe>;

  Keyframe(std::uint64_t id, double timestamp,
           const Pose3d &T_odom_body,
           const KeyframeCloud::ConstPtr &cloud_body,
           const Matrix6d &odom_covariance,
           std::uint8_t trigger_mask)
      : id_(id),
        timestamp_(timestamp),
        T_odom_body_(T_odom_body),
        cloud_body_(cloud_body),
        odom_covariance_(odom_covariance),
        trigger_mask_(trigger_mask),
        T_slam_body_(T_odom_body),
        T_global_body_(T_odom_body)
  {
  }

  std::uint64_t id() const { return id_; }
  double timestamp() const { return timestamp_; }
  const Pose3d &T_odom_body() const { return T_odom_body_; }
  const KeyframeCloud::ConstPtr &cloud_body() const { return cloud_body_; }
  const Matrix6d &odom_covariance() const { return odom_covariance_; }
  std::uint8_t trigger_mask() const { return trigger_mask_; }

  Pose3d T_slam_body() const
  {
    std::lock_guard<std::mutex> lock(optimized_pose_mutex_);
    return T_slam_body_;
  }

  void set_T_slam_body(const Pose3d &pose)
  {
    std::lock_guard<std::mutex> lock(optimized_pose_mutex_);
    T_slam_body_ = pose;
  }

  Pose3d T_global_body() const
  {
    std::lock_guard<std::mutex> lock(optimized_pose_mutex_);
    return T_global_body_;
  }

  void set_T_global_body(const Pose3d &pose)
  {
    std::lock_guard<std::mutex> lock(optimized_pose_mutex_);
    T_global_body_ = pose;
  }

private:
  const std::uint64_t id_;
  const double timestamp_;
  const Pose3d T_odom_body_;
  const KeyframeCloud::ConstPtr cloud_body_;
  const Matrix6d odom_covariance_;
  const std::uint8_t trigger_mask_;

  mutable std::mutex optimized_pose_mutex_;
  Pose3d T_slam_body_;
  Pose3d T_global_body_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_KEYFRAME_H
