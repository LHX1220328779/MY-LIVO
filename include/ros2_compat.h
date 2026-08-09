#ifndef FAST_LIVO_ROS2_COMPAT_H
#define FAST_LIVO_ROS2_COMPAT_H

#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Keep the estimator sources readable while using ROS 2 generated messages.
namespace sensor_msgs
{
using Image = msg::Image;
using ImageConstPtr = msg::Image::ConstSharedPtr;
using Imu = msg::Imu;
using PointCloud2 = msg::PointCloud2;
}  // namespace sensor_msgs

namespace geometry_msgs
{
using PoseStamped = msg::PoseStamped;
using Quaternion = msg::Quaternion;
}  // namespace geometry_msgs

namespace nav_msgs
{
using Odometry = msg::Odometry;
using Path = msg::Path;
}  // namespace nav_msgs

namespace visualization_msgs
{
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}  // namespace visualization_msgs

inline double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
  return rclcpp::Time(stamp).seconds();
}

inline builtin_interfaces::msg::Time stampFromSec(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const double integral_seconds = std::floor(seconds);
  stamp.sec = static_cast<int32_t>(integral_seconds);
  stamp.nanosec = static_cast<uint32_t>(std::llround((seconds - integral_seconds) * 1.0e9));
  if (stamp.nanosec >= 1000000000U)
  {
    ++stamp.sec;
    stamp.nanosec -= 1000000000U;
  }
  return stamp;
}

inline geometry_msgs::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  geometry_msgs::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

class Ros2ParameterReader
{
public:
  explicit Ros2ParameterReader(const rclcpp::Node::SharedPtr &node) : node_(node) {}

  template<typename T>
  void param(const std::string &ros1_name, T &value, const T &default_value)
  {
    std::string name = ros1_name;
    for (char &character : name)
    {
      if (character == '/') character = '.';
    }
    value = node_->declare_parameter<T>(name, default_value);
  }

  void param(const std::string &ros1_name, std::vector<int> &value,
             const std::vector<int> &default_value)
  {
    std::string name = ros1_name;
    for (char &character : name)
    {
      if (character == '/') character = '.';
    }
    const std::vector<int64_t> defaults(default_value.begin(), default_value.end());
    const auto values = node_->declare_parameter<std::vector<int64_t>>(name, defaults);
    value.assign(values.begin(), values.end());
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_ASSERT(condition) assert(condition)

#endif
