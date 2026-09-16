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
  double translation_weight_;
public:
  NativeMotionFactor(gtsam::Key a, gtsam::Key b, const gtsam::Pose3 &measured,
      const gtsam::Vector6 &sigmas, double scale, double translation_weight = 1)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(6, 1), a, b),
        measured_(measured), sigmas_(sigmas), scale_(scale), translation_weight_(translation_weight)
  {
    if (!std::isfinite(translation_weight_) || translation_weight_ <= 0 || translation_weight_ > 1)
      throw std::runtime_error("Invalid native displacement reliability");
  }

  double translationWeight() const { return translation_weight_; }

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
    gtsam::Vector6 residual, derivative;
    for (int i = 0; i < 6; ++i)
    {
      const double value = error(i) / sigmas_(i), magnitude = std::abs(value);
      // Convex Huber in each physical component. A redescending Cauchy
      // motion prior creates a second minimum at a failed native increment,
      // even when a much better globally anchored solution exists.
      const bool quadratic = magnitude <= scale_;
      const double loss = quadratic ? magnitude :
          std::sqrt(2 * scale_ * magnitude - scale_ * scale_);
      residual(i) = std::copysign(loss, value);
      derivative(i) = (quadratic ? 1.0 : scale_ / loss) / sigmas_(i);
      // A measurement-only baseline disagreement can discredit translation
      // without discarding the independently useful native angular increment.
      // This weight is immutable: it cannot create a second minimum by
      // changing with the pose residual during optimization.
      if (i >= 3)
      {
        residual(i) *= std::sqrt(translation_weight_);
        derivative(i) *= std::sqrt(translation_weight_);
      }
    }
    if (ha) *ha = derivative.asDiagonal() * local * ja;
    if (hb) *hb = derivative.asDiagonal() * local * jb;
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
    gtsam::Matrix3 target_sqrt = gtsam::Matrix3::Zero();
    bool sharedLine() const { return linear && !target_sqrt.isZero(0); }
    int rows() const { return linear ? (sharedLine() ? 21 : 12) : 4; }

    Eigen::Matrix<double, 3, 7> lineResidual(const gtsam::Pose3 &relative,
        gtsam::OptionalMatrixType ha = nullptr, gtsam::OptionalMatrixType hb = nullptr) const
    {
      using Moments = Eigen::Matrix<double, 3, 7>;
      const auto rotation = relative.rotation().matrix();
      const gtsam::Vector3 local = relative.transformFrom(point);
      Moments moments;
      moments.col(0) = local - center;
      moments.middleCols<3>(1) = std::sqrt(2.) * target_sqrt;
      moments.rightCols<3>() = std::sqrt(2.) * rotation * source_sqrt;
      // Equal-weight measured distributions, with the unknown common line
      // direction eliminated. This equals four times their joint point-line
      // mean squared distance. Neither scan supplies an exact axis.
      const Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> spectrum(moments * moments.transpose());
      if (spectrum.info() != Eigen::Success)
        throw std::runtime_error("Invalid common line scatter");
      const gtsam::Vector3 direction = spectrum.eigenvectors().col(2);
      const gtsam::Matrix3 projection = gtsam::Matrix3::Identity() - direction * direction.transpose();
      for (int endpoint = 0; endpoint < 2; ++endpoint)
      {
        auto jacobian = endpoint ? hb : ha;
        if (!jacobian) continue;
        jacobian->resize(21, 6);
        for (int axis = 0; axis < 6; ++axis)
        {
          Moments change = Moments::Zero();
          if (axis < 3)
          {
            if (endpoint)
            {
              change.col(0) = -rotation * gtsam::skewSymmetric(point).col(axis);
              for (int part = 0; part < 3; ++part)
                change.col(4 + part) = -std::sqrt(2.) * rotation *
                    gtsam::skewSymmetric(source_sqrt.col(part)).col(axis);
            }
            else
            {
              change.col(0) = gtsam::skewSymmetric(local).col(axis);
              for (int part = 0; part < 3; ++part)
                change.col(4 + part) = gtsam::skewSymmetric(moments.col(4 + part)).col(axis);
            }
          }
          else if (endpoint) change.col(0) = rotation.col(axis - 3);
          else change(axis - 3, 0) = -1;
          const gtsam::Matrix3 scatter_change = change * moments.transpose() + moments * change.transpose();
          gtsam::Vector3 direction_change = gtsam::Vector3::Zero();
          for (int i = 0; i < 2; ++i)
          {
            const double gap = spectrum.eigenvalues()(2) - spectrum.eigenvalues()(i);
            if (gap > 1e-12 * std::max(1., spectrum.eigenvalues()(2)))
              direction_change += spectrum.eigenvectors().col(i) *
                  (spectrum.eigenvectors().col(i).dot(scatter_change * direction) / gap);
          }
          const Moments derivative = projection * change -
              (direction_change * direction.transpose() + direction * direction_change.transpose()) * moments;
          jacobian->col(axis) = Eigen::Map<const gtsam::Vector>(derivative.data(), 21);
        }
      }
      return projection * moments;
    }
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
    gtsam::Matrix physicalResidual(const gtsam::Pose3 &relative) const
    {
      if (sharedLine()) return lineResidual(relative);
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
                std::size_t{0}, [](std::size_t size, const Constraint &c) { return size + c.rows(); }), 1), a, b),
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
      Eigen::Matrix<double, 21, 1> value;
      gtsam::Matrix ja(c.rows(), 6), jb(c.rows(), 6);
      const int count = c.rows();
      if (c.sharedLine())
      {
        const auto physical = c.lineResidual(relative, ha ? &ja : nullptr, hb ? &jb : nullptr);
        value = Eigen::Map<const gtsam::Vector>(physical.data(), 21) / c.sigma;
        if (ha) ja /= c.sigma;
        if (hb) jb /= c.sigma;
      }
      else for (int part = 0; part < 4; ++part)
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
