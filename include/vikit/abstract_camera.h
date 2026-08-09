#ifndef FAST_LIVO_VIKIT_ABSTRACT_CAMERA_H
#define FAST_LIVO_VIKIT_ABSTRACT_CAMERA_H

#include <Eigen/Core>

namespace vk
{

class AbstractCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~AbstractCamera() = default;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d &point) const = 0;
  virtual Eigen::Vector3d cam2world(double x, double y) const = 0;
  Eigen::Vector3d cam2world(const Eigen::Vector2d &pixel) const
  {
    return cam2world(pixel.x(), pixel.y());
  }
  virtual double fx() const = 0;
  virtual double fy() const = 0;
  virtual double cx() const = 0;
  virtual double cy() const = 0;
  virtual double scale() const = 0;
  virtual int width() const = 0;
  virtual int height() const = 0;
  bool isInFrame(const Eigen::Vector2i &pixel, int boundary = 0) const
  {
    return pixel.x() >= boundary && pixel.y() >= boundary &&
           pixel.x() < width() - boundary && pixel.y() < height() - boundary;
  }
};

}  // namespace vk

#endif
