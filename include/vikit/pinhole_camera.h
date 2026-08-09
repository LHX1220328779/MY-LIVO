#ifndef FAST_LIVO_VIKIT_PINHOLE_CAMERA_H
#define FAST_LIVO_VIKIT_PINHOLE_CAMERA_H

#include "vikit/abstract_camera.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace vk
{

class PinholeCamera final : public AbstractCamera
{
public:
  PinholeCamera(int width, int height, double fx, double fy, double cx, double cy,
                const cv::Mat &distortion = cv::Mat(), double scale = 1.0)
  : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy), scale_(scale),
    distortion_(distortion.empty() ? cv::Mat::zeros(1, 5, CV_64F) : distortion.clone())
  {
  }

  Eigen::Vector2d world2cam(const Eigen::Vector3d &point) const override
  {
    return Eigen::Vector2d(fx_ * point.x() / point.z() + cx_,
                           fy_ * point.y() / point.z() + cy_);
  }

  Eigen::Vector3d cam2world(double x, double y) const override
  {
    return Eigen::Vector3d((x - cx_) / fx_, (y - cy_) / fy_, 1.0).normalized();
  }

  void undistortImage(const cv::Mat &input, cv::Mat &output) const
  {
    if (cv::norm(distortion_) == 0.0)
    {
      input.copyTo(output);
      return;
    }
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::undistort(input, output, camera_matrix, distortion_);
  }

  double fx() const override { return fx_; }
  double fy() const override { return fy_; }
  double cx() const override { return cx_; }
  double cy() const override { return cy_; }
  double scale() const override { return scale_; }
  int width() const override { return width_; }
  int height() const override { return height_; }

private:
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double scale_;
  cv::Mat distortion_;
};

}  // namespace vk

#endif
