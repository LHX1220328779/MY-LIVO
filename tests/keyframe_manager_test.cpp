#include "backend/keyframe_manager.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
using my_livo::backend::HasTrigger;
using my_livo::backend::KeyframeCloud;
using my_livo::backend::KeyframeManager;
using my_livo::backend::KeyframePoint;
using my_livo::backend::KeyframeTrigger;
using my_livo::backend::Matrix6d;
using my_livo::backend::Pose3d;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

KeyframeCloud::Ptr MakeCloud()
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  for (int index = 0; index < 20; ++index)
  {
    KeyframePoint point;
    point.x = static_cast<float>(index) * 0.2F;
    point.y = static_cast<float>(index % 3) * 0.4F;
    point.z = static_cast<float>(index % 2) * 0.3F;
    point.intensity = static_cast<float>(index);
    cloud->push_back(point);
  }
  return cloud;
}

Pose3d MakePose(double x, double yaw_degrees = 0.0)
{
  constexpr double kDegreesToRadians =
      3.14159265358979323846 / 180.0;
  return Pose3d(
      Eigen::Quaterniond(Eigen::AngleAxisd(
          yaw_degrees * kDegreesToRadians, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(x, 0.0, 0.0));
}

void TestTriggersAndCloudOwnership()
{
  KeyframeManager::Options options;
  options.translation_threshold_m = 1.0;
  options.rotation_threshold_deg = 10.0;
  options.minimum_interval_sec = 0.2;
  options.maximum_interval_sec = 2.0;
  options.cloud_leaf_size_m = 0.0;
  KeyframeManager manager(options);
  const Matrix6d covariance = Matrix6d::Identity();
  auto cloud = MakeCloud();

  auto first = manager.TryCreate(10.0, MakePose(0.0), cloud, covariance);
  Require(first && first->id() == 0, "first keyframe was not created");
  Require(HasTrigger(first->trigger_mask(), KeyframeTrigger::kFirst),
          "first keyframe trigger is missing");
  const float stored_x = first->cloud_body()->front().x;
  cloud->front().x = 999.0F;
  Require(first->cloud_body()->front().x == stored_x,
          "keyframe did not take ownership of an immutable cloud copy");

  int rejected_cloud_factory_calls = 0;
  Require(!manager.TryCreate(
              10.1, MakePose(2.0),
              [&]() -> KeyframeCloud::ConstPtr {
                ++rejected_cloud_factory_calls;
                return cloud;
              },
              covariance),
          "minimum interval did not suppress a dense keyframe");
  Require(rejected_cloud_factory_calls == 0,
          "rejected frame unnecessarily materialized its point cloud");
  Require(!manager.TryCreate(10.3, MakePose(0.5), cloud, covariance),
          "sub-threshold motion created a keyframe");

  auto translation =
      manager.TryCreate(10.4, MakePose(1.1), cloud, covariance);
  Require(translation && translation->id() == 1,
          "translation trigger did not create keyframe 1");
  Require(HasTrigger(translation->trigger_mask(),
                     KeyframeTrigger::kTranslation),
          "translation trigger mask is missing");

  auto rotation = manager.TryCreate(10.7, MakePose(1.1, 11.0),
                                    cloud, covariance);
  Require(rotation && rotation->id() == 2,
          "rotation trigger did not create keyframe 2");
  Require(HasTrigger(rotation->trigger_mask(), KeyframeTrigger::kRotation),
          "rotation trigger mask is missing");

  auto time = manager.TryCreate(12.8, MakePose(1.1, 11.0),
                                cloud, covariance);
  Require(time && time->id() == 3,
          "maximum interval did not create keyframe 3");
  Require(HasTrigger(time->trigger_mask(), KeyframeTrigger::kTime),
          "time trigger mask is missing");

  const auto statistics = manager.statistics();
  Require(statistics.observed_frames == 6,
          "observed-frame statistics are incorrect");
  Require(statistics.keyframes == 4,
          "keyframe statistics are incorrect");
  Require(statistics.rejected_minimum_interval == 1,
          "minimum-interval statistics are incorrect");
  Require(statistics.rejected_no_trigger == 1,
          "no-trigger statistics are incorrect");
}

void TestTimestampInvariantAndOptimizedPose()
{
  KeyframeManager::Options options;
  options.cloud_leaf_size_m = 0.0;
  KeyframeManager manager(options);
  const Matrix6d covariance = Matrix6d::Identity();
  const auto cloud = MakeCloud();
  auto keyframe = manager.TryCreate(20.0, MakePose(0.0), cloud, covariance);

  bool timestamp_rejected = false;
  try
  {
    (void)manager.TryCreate(20.0, MakePose(2.0), cloud, covariance);
  }
  catch (const std::logic_error &)
  {
    timestamp_rejected = true;
  }
  Require(timestamp_rejected,
          "non-increasing timestamp did not raise a logic error");

  const Pose3d optimized = MakePose(5.0, 15.0);
  keyframe->set_T_slam_body(optimized);
  Require((keyframe->T_slam_body().translation -
           optimized.translation).norm() < 1.0e-12,
          "optimized keyframe pose was not stored");
  Require((keyframe->T_odom_body().translation -
           Eigen::Vector3d::Zero()).norm() < 1.0e-12,
          "optimized pose changed immutable raw odometry pose");
}
}  // namespace

int main()
{
  try
  {
    TestTriggersAndCloudOwnership();
    TestTimestampInvariantAndOptimizedPose();
  }
  catch (const std::exception &error)
  {
    std::cerr << "keyframe_manager_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "keyframe_manager_test passed\n";
  return 0;
}
