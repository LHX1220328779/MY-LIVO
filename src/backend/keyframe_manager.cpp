#include "backend/keyframe_manager.h"

#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace my_livo::backend
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

KeyframeManager::KeyframeManager(const Options &options)
    : options_(options),
      rotation_threshold_rad_(options.rotation_threshold_deg * kPi / 180.0)
{
  ValidateOptions(options_);
  if (!options_.csv_path.empty())
  {
    const std::filesystem::path csv_path(options_.csv_path);
    if (csv_path.has_parent_path())
      std::filesystem::create_directories(csv_path.parent_path());
    csv_stream_.open(csv_path, std::ios::out | std::ios::trunc);
    if (!csv_stream_.is_open())
      throw std::runtime_error("Cannot open keyframe CSV: " +
                               csv_path.string());
    csv_stream_
        << "id,timestamp,trigger,tx,ty,tz,qx,qy,qz,qw,cloud_points,"
           "sigma_x,sigma_y,sigma_z,sigma_roll,sigma_pitch,sigma_yaw\n";
  }
}

void KeyframeManager::ValidateOptions(const Options &options)
{
  if (!std::isfinite(options.translation_threshold_m) ||
      options.translation_threshold_m <= 0.0)
    throw std::invalid_argument(
        "Keyframe translation threshold must be finite and positive.");
  if (!std::isfinite(options.rotation_threshold_deg) ||
      options.rotation_threshold_deg <= 0.0 ||
      options.rotation_threshold_deg > 180.0)
    throw std::invalid_argument(
        "Keyframe rotation threshold must be in (0, 180] degrees.");
  if (!std::isfinite(options.maximum_interval_sec) ||
      options.maximum_interval_sec <= 0.0)
    throw std::invalid_argument(
        "Keyframe maximum interval must be finite and positive.");
  if (!std::isfinite(options.minimum_interval_sec) ||
      options.minimum_interval_sec < 0.0 ||
      options.minimum_interval_sec > options.maximum_interval_sec)
    throw std::invalid_argument(
        "Keyframe minimum interval must be in [0, maximum interval].");
  if (!std::isfinite(options.cloud_leaf_size_m) ||
      options.cloud_leaf_size_m < 0.0)
    throw std::invalid_argument(
        "Keyframe cloud leaf size must be finite and non-negative.");
}

void KeyframeManager::ValidateInput(
    double timestamp, const Pose3d &T_odom_body,
    const Matrix6d &odom_covariance)
{
  if (!std::isfinite(timestamp))
    throw std::invalid_argument("Keyframe timestamp is not finite.");
  if (!T_odom_body.isFinite())
    throw std::invalid_argument("Keyframe pose is not finite.");
  if (!odom_covariance.allFinite())
    throw std::invalid_argument("Keyframe odometry covariance is not finite.");
}

void KeyframeManager::ValidateCloud(
    const KeyframeCloud::ConstPtr &cloud_body)
{
  if (!cloud_body || cloud_body->empty())
    throw std::invalid_argument("Keyframe body cloud is empty.");
  for (const KeyframePoint &point : cloud_body->points)
  {
    if (!pcl::isFinite(point))
      throw std::invalid_argument("Keyframe body cloud has a non-finite point.");
  }
}

KeyframeCloud::ConstPtr KeyframeManager::PrepareCloud(
    const KeyframeCloud::ConstPtr &cloud_body) const
{
  KeyframeCloud::Ptr prepared(new KeyframeCloud());
  if (options_.cloud_leaf_size_m > 0.0)
  {
    pcl::VoxelGrid<KeyframePoint> voxel_filter;
    const float leaf = static_cast<float>(options_.cloud_leaf_size_m);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    voxel_filter.setInputCloud(cloud_body);
    voxel_filter.filter(*prepared);
  }
  else
  {
    *prepared = *cloud_body;
  }
  if (prepared->empty())
    throw std::runtime_error(
        "Keyframe cloud became empty after backend downsampling.");
  return prepared;
}

Keyframe::Ptr KeyframeManager::TryCreate(
    double timestamp, const Pose3d &T_odom_body,
    const CloudFactory &cloud_factory,
    const Matrix6d &odom_covariance)
{
  ValidateInput(timestamp, T_odom_body, odom_covariance);
  if (!cloud_factory)
    throw std::invalid_argument("Keyframe cloud factory is empty.");
  std::lock_guard<std::mutex> lock(mutex_);

  if (has_observation_ && timestamp <= last_observation_timestamp_)
    throw std::logic_error(
        "Backend keyframe input timestamp is not strictly increasing.");
  has_observation_ = true;
  last_observation_timestamp_ = timestamp;
  ++statistics_.observed_frames;

  std::uint8_t trigger_mask =
      static_cast<std::uint8_t>(KeyframeTrigger::kFirst);
  if (!keyframes_.empty())
  {
    trigger_mask = static_cast<std::uint8_t>(KeyframeTrigger::kNone);
    const Keyframe &last = *keyframes_.back();
    const double time_delta = timestamp - last.timestamp();
    if (time_delta < options_.minimum_interval_sec)
    {
      ++statistics_.rejected_minimum_interval;
      return nullptr;
    }

    const double translation_delta =
        (T_odom_body.translation -
         last.T_odom_body().translation).norm();
    const double rotation_delta =
        last.T_odom_body().rotation.angularDistance(T_odom_body.rotation);
    if (translation_delta >= options_.translation_threshold_m)
      trigger_mask |=
          static_cast<std::uint8_t>(KeyframeTrigger::kTranslation);
    if (rotation_delta >= rotation_threshold_rad_)
      trigger_mask |=
          static_cast<std::uint8_t>(KeyframeTrigger::kRotation);
    if (time_delta >= options_.maximum_interval_sec)
      trigger_mask |= static_cast<std::uint8_t>(KeyframeTrigger::kTime);

    if (trigger_mask == static_cast<std::uint8_t>(KeyframeTrigger::kNone))
    {
      ++statistics_.rejected_no_trigger;
      return nullptr;
    }
  }

  const KeyframeCloud::ConstPtr cloud_body = cloud_factory();
  ValidateCloud(cloud_body);
  const KeyframeCloud::ConstPtr stored_cloud = PrepareCloud(cloud_body);
  auto keyframe = std::make_shared<Keyframe>(
      keyframes_.size(), timestamp, T_odom_body, stored_cloud,
      odom_covariance, trigger_mask);
  keyframes_.push_back(keyframe);
  statistics_.keyframes = keyframes_.size();
  WriteCsv(*keyframe);
  return keyframe;
}

Keyframe::Ptr KeyframeManager::TryCreate(
    double timestamp, const Pose3d &T_odom_body,
    const KeyframeCloud::ConstPtr &cloud_body,
    const Matrix6d &odom_covariance)
{
  return TryCreate(timestamp, T_odom_body,
                   [cloud_body]() { return cloud_body; },
                   odom_covariance);
}

std::vector<Keyframe::Ptr> KeyframeManager::keyframes() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframes_;
}

Keyframe::Ptr KeyframeManager::latest_keyframe() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframes_.empty() ? nullptr : keyframes_.back();
}

std::size_t KeyframeManager::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframes_.size();
}

KeyframeManager::Statistics KeyframeManager::statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return statistics_;
}

std::string KeyframeManager::TriggerMaskToString(std::uint8_t trigger_mask)
{
  std::ostringstream stream;
  const auto append = [&stream](const char *name) {
    if (stream.tellp() > 0) stream << '|';
    stream << name;
  };
  if (HasTrigger(trigger_mask, KeyframeTrigger::kFirst)) append("first");
  if (HasTrigger(trigger_mask, KeyframeTrigger::kTranslation))
    append("translation");
  if (HasTrigger(trigger_mask, KeyframeTrigger::kRotation)) append("rotation");
  if (HasTrigger(trigger_mask, KeyframeTrigger::kTime)) append("time");
  if (stream.tellp() == 0) stream << "none";
  return stream.str();
}

void KeyframeManager::WriteCsv(const Keyframe &keyframe)
{
  if (!csv_stream_.is_open()) return;
  const Pose3d &pose = keyframe.T_odom_body();
  const Matrix6d &covariance = keyframe.odom_covariance();
  const auto standard_deviation = [&covariance](int index) {
    return std::sqrt(std::max(0.0, covariance(index, index)));
  };
  csv_stream_ << std::setprecision(17) << keyframe.id() << ','
              << keyframe.timestamp() << ','
              << TriggerMaskToString(keyframe.trigger_mask()) << ','
              << pose.translation.x() << ',' << pose.translation.y() << ','
              << pose.translation.z() << ',' << pose.rotation.x() << ','
              << pose.rotation.y() << ',' << pose.rotation.z() << ','
              << pose.rotation.w() << ',' << keyframe.cloud_body()->size();
  for (int index = 0; index < 6; ++index)
    csv_stream_ << ',' << standard_deviation(index);
  csv_stream_ << '\n';
  csv_stream_.flush();
}

}  // namespace my_livo::backend
