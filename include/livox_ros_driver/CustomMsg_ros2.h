#ifndef FAST_LIVO_LIVOX_CUSTOM_MSG_ROS2_H
#define FAST_LIVO_LIVOX_CUSTOM_MSG_ROS2_H

#include "ros2_compat.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace livox_ros_driver
{
struct CustomPoint
{
  uint32_t offset_time = 0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
};

struct CustomMsg
{
  using Ptr = std::shared_ptr<CustomMsg>;
  using ConstPtr = std::shared_ptr<const CustomMsg>;
  std_msgs::msg::Header header;
  uint64_t timebase = 0;
  uint32_t point_num = 0;
  uint8_t lidar_id = 0;
  std::vector<CustomPoint> points;
};
}  // namespace livox_ros_driver

#endif
