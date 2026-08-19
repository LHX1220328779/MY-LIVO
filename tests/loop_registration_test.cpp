#include "backend/loop_registration.h"

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using my_livo::backend::Keyframe;
using my_livo::backend::KeyframeCloud;
using my_livo::backend::KeyframePoint;
using my_livo::backend::KeyframeTrigger;
using my_livo::backend::LoopCandidate;
using my_livo::backend::LoopRegistration;
using my_livo::backend::LoopRegistrationResult;
using my_livo::backend::LoopRegistrationStatus;
using my_livo::backend::Matrix6d;
using my_livo::backend::Pose3d;

constexpr double kPi = 3.14159265358979323846;

void Require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

Pose3d MakePose(double x, double y, double z, double yaw_deg)
{
  return Pose3d(
      Eigen::Quaterniond(Eigen::AngleAxisd(
          yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(x, y, z));
}

void AddPoint(KeyframeCloud *cloud, double x, double y, double z)
{
  KeyframePoint point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  point.intensity = static_cast<float>(z + 10.0);
  cloud->push_back(point);
}

KeyframeCloud::Ptr MakeAsymmetricEnvironment()
{
  KeyframeCloud::Ptr cloud(new KeyframeCloud());
  for (double x = -8.0; x <= 8.0; x += 0.35)
    for (double y = -7.0; y <= 7.0; y += 0.35)
      AddPoint(cloud.get(), x, y, 0.05 * std::sin(0.4 * x));
  for (double y = -7.0; y <= 7.0; y += 0.30)
    for (double z = 0.2; z <= 4.5; z += 0.25)
      AddPoint(cloud.get(), 5.5, y, z);
  for (double x = -8.0; x <= 8.0; x += 0.30)
    for (double z = 0.2; z <= 3.0; z += 0.25)
      AddPoint(cloud.get(), x, -4.5, z);
  for (double z = 0.0; z <= 5.0; z += 0.10)
  {
    AddPoint(cloud.get(), -2.0, 3.0, z);
    AddPoint(cloud.get(), -2.5, 3.5, z);
  }
  return cloud;
}

Keyframe::Ptr MakeKeyframe(std::uint64_t id, const Pose3d &pose,
                           const KeyframeCloud::ConstPtr &cloud)
{
  return std::make_shared<Keyframe>(
      id, static_cast<double>(id), pose, cloud, Matrix6d::Identity(),
      static_cast<std::uint8_t>(
          id == 0U ? KeyframeTrigger::kFirst
                   : KeyframeTrigger::kTranslation));
}

std::size_t CountLines(const std::filesystem::path &path)
{
  std::ifstream stream(path);
  Require(stream.good(), "registration CSV was not created");
  std::size_t lines = 0;
  std::string line;
  while (std::getline(stream, line)) ++lines;
  return lines;
}

void TestMultiResolutionRegistrationAndWorker()
{
  const Pose3d truth = MakePose(1.8, -0.9, 0.25, 8.0);
  const Pose3d initial = MakePose(2.0, -1.0, 0.30, 9.0);
  const auto target_cloud = MakeAsymmetricEnvironment();
  KeyframeCloud::Ptr source_cloud(new KeyframeCloud());
  const Pose3d T_current_candidate = truth.inverse();
  source_cloud->reserve(target_cloud->size());
  for (const KeyframePoint &target_point : target_cloud->points)
  {
    const Eigen::Vector3d current_point =
        T_current_candidate * Eigen::Vector3d(
                                  target_point.x, target_point.y,
                                  target_point.z);
    AddPoint(source_cloud.get(), current_point.x(), current_point.y(),
             current_point.z());
  }

  auto historical = MakeKeyframe(
      0, Pose3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()),
      target_cloud);
  auto current = MakeKeyframe(1, initial, source_cloud);
  const std::vector<Keyframe::Ptr> keyframes{historical, current};
  LoopCandidate candidate;
  candidate.current_id = 1;
  candidate.candidate_id = 0;
  candidate.current_timestamp = 1.0;
  candidate.candidate_timestamp = 0.0;
  candidate.id_separation = 1;
  candidate.time_separation_sec = 1.0;
  candidate.T_candidate_current_initial = initial;

  const auto temporary = std::filesystem::temp_directory_path();
  const auto summary_path = temporary / "my_livo_loop_registration_test.csv";
  const auto level_path = temporary / "my_livo_loop_registration_levels_test.csv";
  LoopRegistration::Options options;
  options.target_submap_half_width_keyframes = 0;
  options.target_submap_stride_keyframes = 1;
  options.resolutions_m = {2.0, 1.0, 0.5};
  options.voxel_leaf_size_ratio = 0.20;
  options.minimum_voxel_leaf_size_m = 0.20;
  options.transformation_epsilon = 1.0e-4;
  options.step_size = 0.30;
  options.maximum_iterations = 60;
  options.overlap_max_correspondence_distance_m = 0.50;
  options.minimum_source_points = 50;
  options.minimum_target_points = 100;
  options.registration_csv_path = summary_path.string();
  options.level_csv_path = level_path.string();

  LoopRegistrationResult callback_result;
  bool callback_called = false;
  {
    LoopRegistration registration(options);
    registration.SetResultCallback(
        [&](const LoopRegistrationResult &result) {
          callback_result = result;
          callback_called = true;
        });
    Require(registration.Enqueue(candidate, keyframes),
            "valid registration job was not enqueued");
    registration.WaitUntilIdle();
    const auto statistics = registration.statistics();
    Require(statistics.enqueued == 1 && statistics.completed == 1 &&
                statistics.queue_drops == 0,
            "asynchronous registration accounting is incorrect");
  }

  Require(callback_called, "registration result callback was not called");
  Require(callback_result.status == LoopRegistrationStatus::kCompleted,
          "synthetic NDT registration did not complete");
  Require(callback_result.levels.size() == options.resolutions_m.size(),
          "not all NDT resolutions were executed");
  Require(callback_result.converged,
          "final synthetic NDT level did not converge");
  Require(callback_result.overlap > 0.90,
          "synthetic registration overlap is unexpectedly low");
  Require((callback_result.T_candidate_current.translation -
           truth.translation).norm() < 0.20,
          "synthetic NDT translation is inaccurate");
  Require(callback_result.T_candidate_current.rotation.angularDistance(
              truth.rotation) *
              180.0 / kPi <
              2.0,
          "synthetic NDT rotation is inaccurate");
  Require(CountLines(summary_path) == 2,
          "registration summary CSV row count is incorrect");
  Require(CountLines(level_path) == 4,
          "registration level CSV row count is incorrect");
  std::filesystem::remove(summary_path);
  std::filesystem::remove(level_path);
}

void TestInvalidCandidateIsRejected()
{
  LoopRegistration::Options options;
  options.target_submap_half_width_keyframes = 0;
  options.resolutions_m = {1.0};
  LoopRegistration registration(options);
  LoopCandidate invalid;
  invalid.current_id = 0;
  invalid.candidate_id = 0;
  const auto result = registration.Register(invalid, {});
  Require(result.status == LoopRegistrationStatus::kInvalidInput,
          "invalid loop-registration candidate was not rejected");
}
}  // namespace

int main()
{
  try
  {
    TestMultiResolutionRegistrationAndWorker();
    TestInvalidCandidateIsRejected();
  }
  catch (const std::exception &error)
  {
    std::cerr << "loop_registration_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "loop_registration_test passed\n";
  return 0;
}
