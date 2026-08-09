#ifndef FAST_LIVO_VIKIT_VISION_H
#define FAST_LIVO_VIKIT_VISION_H

#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace vk
{

inline void halfSample(const cv::Mat &source, cv::Mat &destination)
{
  cv::resize(source, destination, destination.size(), 0.0, 0.0, cv::INTER_AREA);
}

inline float interpolateMat_8u(const cv::Mat &image, float x, float y)
{
  const int x0 = static_cast<int>(x);
  const int y0 = static_cast<int>(y);
  const float dx = x - static_cast<float>(x0);
  const float dy = y - static_cast<float>(y0);
  const uint8_t *row0 = image.ptr<uint8_t>(y0);
  const uint8_t *row1 = image.ptr<uint8_t>(y0 + 1);
  return (1.0F - dx) * (1.0F - dy) * row0[x0] +
         dx * (1.0F - dy) * row0[x0 + 1] +
         (1.0F - dx) * dy * row1[x0] + dx * dy * row1[x0 + 1];
}

inline float shiTomasiScore(const cv::Mat &image, int x, int y)
{
  constexpr int half_box = 4;
  if (x < half_box + 1 || y < half_box + 1 ||
      x >= image.cols - half_box - 1 || y >= image.rows - half_box - 1)
  {
    return 0.0F;
  }
  cv::Mat response;
  cv::cornerMinEigenVal(image(cv::Rect(x - half_box, y - half_box,
                                        2 * half_box + 1, 2 * half_box + 1)),
                        response, 3, 3);
  double maximum = 0.0;
  cv::minMaxLoc(response, nullptr, &maximum);
  return static_cast<float>(maximum);
}

}  // namespace vk

#endif
