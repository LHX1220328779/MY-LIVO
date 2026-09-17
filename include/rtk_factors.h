#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/linear/JacobianFactor.h>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <numeric>

// Angular observability after allowing position to move. Hrr alone would
// mistake rotation/translation coupling for an independently observed angle.
// Diagnostic only: a well-conditioned fixed map can still be biased. Never
// use inverse(H) as independent Between covariance or turn this into a hard
// attitude observation. Null directions remain null; no eigen floor.
inline gtsam::Matrix3 nativeAngularObservability(const gtsam::Matrix6 &information)
{
  const gtsam::Matrix6 symmetric = .5 * (information + information.transpose());
  const Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> translation(symmetric.bottomRightCorner<3, 3>());
  if (translation.info() != Eigen::Success || !translation.eigenvalues().allFinite())
    throw std::runtime_error("Invalid native LiDAR position information");
  gtsam::Matrix3 inverse = gtsam::Matrix3::Zero();
  for (int i = 0; i < 3; ++i)
    if (translation.eigenvalues()(i) > std::max(1e-12, 1e-10 * translation.eigenvalues()(2)))
      inverse.noalias() += translation.eigenvectors().col(i) * translation.eigenvectors().col(i).transpose() /
          translation.eigenvalues()(i);
  const gtsam::Matrix3 angular = symmetric.topLeftCorner<3, 3>() -
      symmetric.topRightCorner<3, 3>() * inverse * symmetric.bottomLeftCorner<3, 3>();
  const Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> spectrum(.5 * (angular + angular.transpose()));
  if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite())
    throw std::runtime_error("Invalid native LiDAR angular information");
  if (spectrum.eigenvalues()(2) <= 1e-12) return gtsam::Matrix3::Zero();
  return spectrum.eigenvectors() * (spectrum.eigenvalues().cwiseMax(0) /
      spectrum.eigenvalues()(2)).asDiagonal() * spectrum.eigenvectors().transpose();
}

// A gravity vector observes only two tilt directions. Yaw keeps the existing
// finite gauge variance; never invent a third inertial attitude observation.
inline gtsam::Matrix3 gravityOrientationCovariance(const gtsam::Vector3 &gravity,
    const gtsam::Matrix3 &covariance, double yaw_variance)
{
  const gtsam::Matrix3 derivative = gtsam::skewSymmetric(gravity) / gravity.squaredNorm();
  const gtsam::Matrix3 result = derivative * covariance * derivative.transpose() +
      yaw_variance * gravity.normalized() * gravity.normalized().transpose();
  return .5 * (result + result.transpose());
}

// Relative native LIO is reliable locally, not an unbounded Gaussian truth
// in a mine's failed directions. Robustify the six whitened components
// separately, so a bad yaw component cannot suppress valid roll/pitch rows.
class NativeMotionFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
  gtsam::Pose3 measured_;
  gtsam::Vector6 sigmas_;
  double scale_;
  gtsam::Matrix3 angular_basis_ = gtsam::Matrix3::Identity();
  gtsam::Vector3 angular_strength_ = gtsam::Vector3::Zero();
  double angular_reliability_ = 0;
public:
  NativeMotionFactor(gtsam::Key a, gtsam::Key b, const gtsam::Pose3 &measured,
      const gtsam::Vector6 &sigmas, double scale,
      const gtsam::Matrix3 &angular_observability = gtsam::Matrix3::Zero(),
      double angular_reliability = 0)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(6, 1), a, b),
        measured_(measured), sigmas_(sigmas), scale_(scale), angular_reliability_(angular_reliability)
  {
    if (!std::isfinite(angular_reliability_) || angular_reliability_ < 0 || angular_reliability_ > 1)
      throw std::runtime_error("Invalid native angular reliability");
    if (angular_observability.squaredNorm() == 0 || angular_reliability_ == 0) return;
    const Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> spectrum(
        .5 * (angular_observability + angular_observability.transpose()));
    if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite() ||
        spectrum.eigenvalues()(0) < -1e-9 || spectrum.eigenvalues()(2) > 1 + 1e-9)
      throw std::runtime_error("Invalid native angular direction strengths");
    angular_basis_ = spectrum.eigenvectors().transpose();
    angular_strength_ = angular_reliability_ * spectrum.eigenvalues().cwiseMax(0).cwiseMin(1);
  }

  double angularReliability() const { return angular_reliability_; }

  gtsam::Vector6 physicalError(const gtsam::Pose3 &a, const gtsam::Pose3 &b) const
  {
    const auto error = measured_.inverse() * a.between(b);
    gtsam::Vector6 residual;
    residual << gtsam::Rot3::Logmap(error.rotation()), error.translation();
    return residual;
  }

  gtsam::Vector evaluateError(const gtsam::Pose3 &a, const gtsam::Pose3 &b,
      gtsam::OptionalMatrixType ha = nullptr, gtsam::OptionalMatrixType hb = nullptr) const override
  {
    gtsam::Matrix6 ja, jb, local;
    const auto relative = a.between(b, ja, jb);
    const auto difference = measured_.inverse() * relative;
    gtsam::Matrix3 rotation_derivative;
    gtsam::Vector6 error;
    error << gtsam::Rot3::Logmap(difference.rotation(), rotation_derivative), difference.translation();
    // These are orientation and endpoint-position observations, not a screw
    // displacement. SE3 Log premultiplies position error by J(theta)^-1:
    // a failed multi-metre translation then exerts a fictitious torque on
    // the END pose, even though its orientation cannot move its IMU centre.
    // Express position in the measured end-body frame, with no rotation-log
    // coupling. Its start-pose lever remains the actual observed baseline.
    local.setZero();
    local.topLeftCorner<3, 3>() = rotation_derivative;
    local.bottomRightCorner<3, 3>() = difference.rotation().matrix();
    gtsam::Matrix6 whitening = sigmas_.cwiseInverse().asDiagonal();
    for (int i = 0; i < 3; ++i)
      whitening.block<1, 3>(i, 0) = angular_basis_.row(i) /
          std::sqrt(angular_basis_.row(i).cwiseAbs2().dot(sigmas_.head<3>().cwiseAbs2()));
    const gtsam::Vector6 whitened = whitening * error;
    gtsam::Vector6 residual, derivative;
    for (int i = 0; i < 6; ++i)
    {
      const double value = whitened(i), magnitude = std::abs(value);
      // Convex Huber in each physical component. A redescending Cauchy
      // motion prior creates a second minimum at a failed native increment,
      // even when a much better globally anchored solution exists.
      const bool quadratic = magnitude <= scale_;
      const double huber_loss = quadratic ? magnitude :
          std::sqrt(2 * scale_ * magnitude - scale_ * scale_);
      // When independent measured baseline lengths agree, preserve curvature
      // only in LiDAR-observed angular directions. As baseline disagreement
      // grows, this smoothly falls back to the original component-wise Huber.
      // No INS attitude is introduced and no fixed-map Hessian is inverted.
      const double strength = i < 3 ? angular_strength_(i) : 0;
      const double loss = std::sqrt((1 - strength) * huber_loss * huber_loss +
                                    strength * value * value);
      residual(i) = std::copysign(loss, value);
      derivative(i) = quadratic ? 1.0 :
          ((1 - strength) * scale_ + strength * magnitude) / loss;
    }
    if (ha) *ha = derivative.asDiagonal() * whitening * local * ja;
    if (hb) *hb = derivative.asDiagonal() * whitening * local * jb;
    return residual;
  }
};

// One sparse graph factor per scan pair. A surface contributes its actual
// mean squared point-to-plane/line distance, not only its centroid distance.
// Sufficient statistics retain orientation evidence without allocating a
// residual for every return or treating correlated returns as independent.
class ScanPlaneBatchFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
public:
  struct Constraint
  {
    gtsam::Vector3 normal, center, point;
    double sigma;
    bool linear = false;
    gtsam::Matrix3 source_sqrt = gtsam::Matrix3::Zero();
    gtsam::Matrix3 projector() const
    {
      const gtsam::Matrix3 along = normal * normal.transpose();
      return linear ? gtsam::Matrix3(gtsam::Matrix3::Identity() - along) : along;
    }
    gtsam::Vector3 project(const gtsam::Vector3 &delta) const
    {
      const gtsam::Vector3 along = normal * normal.dot(delta);
      return linear ? gtsam::Vector3(delta - along) : along;
    }
    Eigen::Matrix<double, 3, 4> physicalResidual(const gtsam::Pose3 &relative) const
    {
      Eigen::Matrix<double, 3, 4> result;
      result.col(0) = project(relative.transformFrom(point) - center);
      result.rightCols<3>() = projector() * relative.rotation().matrix() * source_sqrt;
      return result;
    }
  };
private:
  std::vector<Constraint> constraints_;
public:
  ScanPlaneBatchFactor(gtsam::Key a, gtsam::Key b, std::vector<Constraint> constraints)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(std::accumulate(constraints.begin(), constraints.end(),
                std::size_t{0}, [](std::size_t size, const Constraint &c) { return size + (c.linear ? 12 : 4); }), 1), a, b),
        constraints_(std::move(constraints)) {}

  const std::vector<Constraint> &constraints() const { return constraints_; }

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &values) const override
  {
    if (!active(values)) return nullptr;
    gtsam::Matrix a, b;
    const gtsam::Vector rhs = -evaluateError(values.at<gtsam::Pose3>(keys()[0]),
        values.at<gtsam::Pose3>(keys()[1]), &a, &b);
    if (a.rows() <= 6)
      return std::make_shared<gtsam::JacobianFactor>(keys()[0], a, keys()[1], b, rhs);
    // A relative-pose measurement has at most six observable directions:
    // Hb = -Ha * Ad(Ta^-1 Tb). Compress its whitened linear system BEFORE
    // sparse elimination, without inventing information in degenerate axes.
    // The extra zero-Jacobian row preserves the constant residual energy.
    Eigen::HouseholderQR<gtsam::Matrix> qr(a);
    gtsam::Matrix other(a.rows(), 7);
    other.leftCols(6) = b;
    other.col(6) = rhs;
    const gtsam::Matrix projected = qr.householderQ().adjoint() * other;
    gtsam::Matrix compact_a = gtsam::Matrix::Zero(7, 6);
    gtsam::Matrix compact_b = gtsam::Matrix::Zero(7, 6);
    compact_a.topRows(6) = qr.matrixQR().topRows(6).template triangularView<Eigen::Upper>();
    compact_b.topRows(6) = projected.topLeftCorner(6, 6);
    gtsam::Vector compact_rhs(7);
    compact_rhs.head(6) = projected.col(6).head(6);
    compact_rhs(6) = projected.col(6).tail(a.rows() - 6).norm();
    return std::make_shared<gtsam::JacobianFactor>(keys()[0], compact_a,
        keys()[1], compact_b, compact_rhs);
  }

  gtsam::Vector evaluateError(const gtsam::Pose3 &a, const gtsam::Pose3 &b,
      gtsam::OptionalMatrixType ha = nullptr,
      gtsam::OptionalMatrixType hb = nullptr) const override
  {
    const auto relative = a.between(b);
    const auto rotation = relative.rotation().matrix();
    const std::size_t rows = dim();
    gtsam::Vector residual(rows);
    if (ha) ha->resize(rows, 6);
    if (hb) hb->resize(rows, 6);
    std::size_t row = 0;
    for (std::size_t i = 0; i < constraints_.size(); ++i)
    {
      const auto &c = constraints_[i];
      const gtsam::Vector3 local = relative.transformFrom(c.point);
      const gtsam::Matrix3 projection = c.projector() / c.sigma;
      Eigen::Matrix<double, 12, 1> value;
      Eigen::Matrix<double, 12, 6> ja, jb;
      const int count = c.linear ? 12 : 4;
      for (int part = 0; part < 4; ++part)
      {
        const gtsam::Vector3 body = part ? gtsam::Vector3(c.source_sqrt.col(part - 1)) : c.point;
        const gtsam::Vector3 rotated = part ? gtsam::Vector3(rotation * body) : local;
        if (!c.linear)
        {
          // A plane needs one normal row per moment, not a rank-one 3D
          // embedding. Keep the exact same energy with one third of the QR work.
          const Eigen::RowVector3d normal = c.normal.transpose() / c.sigma;
          value(part) = normal.dot(part ? rotated : gtsam::Vector3(local - c.center));
          if (ha)
          {
            ja.block<1, 3>(part, 0) = normal * gtsam::skewSymmetric(rotated);
            ja.block<1, 3>(part, 3) = part ? Eigen::RowVector3d::Zero().eval() : Eigen::RowVector3d(-normal);
          }
          if (hb)
          {
            jb.block<1, 3>(part, 0) = -normal * rotation * gtsam::skewSymmetric(body);
            jb.block<1, 3>(part, 3) = part ? Eigen::RowVector3d::Zero().eval() : Eigen::RowVector3d(normal * rotation);
          }
        }
        else
        {
          value.segment<3>(3 * part) = projection * (part ? rotated : gtsam::Vector3(local - c.center));
          if (ha)
          {
            ja.block<3, 3>(3 * part, 0) = projection * gtsam::skewSymmetric(rotated);
            ja.block<3, 3>(3 * part, 3) = part ? gtsam::Matrix3::Zero().eval() : gtsam::Matrix3(-projection);
          }
          if (hb)
          {
            jb.block<3, 3>(3 * part, 0) = -projection * rotation * gtsam::skewSymmetric(body);
            jb.block<3, 3>(3 * part, 3) = part ? gtsam::Matrix3::Zero().eval() : gtsam::Matrix3(projection * rotation);
          }
        }
      }
      // ||P(T mean - target)||^2 + ||P R sqrt(Cov)||_F^2 is EXACTLY
      // the average squared distance of the shared source returns. It keeps
      // their physical extent: rotating a plane/line about its centroid can
      // no longer have zero geometric cost. No fitted joint plane, fabricated
      // normal prior, extra point-count weight or full-rank pose prior.
      constexpr double huber = 1.5;
      const double length = value.head(count).norm();
      double scale = 1, radial = 0;
      if (length > huber)
      {
        scale = std::sqrt(2 * huber * length - huber * huber) / length;
        radial = (huber / (scale * length) - scale) / (length * length);
      }
      residual.segment(row, count) = scale * value.head(count);
      if (ha) ha->block(row, 0, count, 6) = scale * ja.topRows(count) +
          radial * value.head(count) * (value.head(count).transpose() * ja.topRows(count));
      if (hb) hb->block(row, 0, count, 6) = scale * jb.topRows(count) +
          radial * value.head(count) * (value.head(count).transpose() * jb.topRows(count));
      row += count;
    }
    return residual;
  }
};

inline gtsam::Pose3 interpolateTransform(const gtsam::Pose3 &a,
                                  const gtsam::Pose3 &b, double ratio)
{
  // Lie interpolation is equivariant to a change of world origin. Separate
  // translation interpolation of world corrections is not, especially when
  // working far from the ENU origin.
  return a * gtsam::Pose3::Expmap(ratio * gtsam::Pose3::Logmap(a.between(b)));
}
