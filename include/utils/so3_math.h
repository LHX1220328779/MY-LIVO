#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <Eigen/Core>
#include <math.h>
#include <utility>

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

template <typename T> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
  // Rodrigues with its analytic zero-angle limit. Never discard a small
  // IEKF correction while still shrinking the associated covariance.
  const T theta2 = ang.squaredNorm();
  const T theta = std::sqrt(theta2);
  const T a = theta2 < T(1e-8) ? T(1) - theta2 / T(6) + theta2 * theta2 / T(120)
                              : std::sin(theta) / theta;
  const T b = theta2 < T(1e-8) ? T(0.5) - theta2 / T(24) + theta2 * theta2 / T(720)
                              : (T(1) - std::cos(theta)) / theta2;
  Eigen::Matrix<T, 3, 3> K;
  K << SKEW_SYM_MATRX(ang);
  return Eigen::Matrix<T, 3, 3>::Identity() + a * K + b * K * K;
}

template <typename T, typename Ts> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
  Eigen::Matrix<T, 3, 1> increment = ang_vel * dt;
  return Exp(std::move(increment));
}

template <typename T> Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
  return Exp(Eigen::Matrix<T, 3, 1>(v1, v2, v3));
}

/* Logrithm of a Rotation Matrix */
template <typename T> Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T> Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular)
  {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  }
  else
  {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

#endif
