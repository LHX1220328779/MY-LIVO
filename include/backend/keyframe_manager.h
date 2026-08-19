#ifndef MY_LIVO_BACKEND_KEYFRAME_MANAGER_H
#define MY_LIVO_BACKEND_KEYFRAME_MANAGER_H

#include "backend/keyframe.h"

#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace my_livo::backend
{

class KeyframeManager
{
public:
  struct Options
  {
    double translation_threshold_m = 1.0;
    double rotation_threshold_deg = 10.0;
    double maximum_interval_sec = 2.0;
    double minimum_interval_sec = 0.2;
    double cloud_leaf_size_m = 0.5;
    std::string csv_path;
  };

  struct Statistics
  {
    std::uint64_t observed_frames = 0;
    std::uint64_t keyframes = 0;
    std::uint64_t rejected_minimum_interval = 0;
    std::uint64_t rejected_no_trigger = 0;
  };

  explicit KeyframeManager(const Options &options);

  using CloudFactory = std::function<KeyframeCloud::ConstPtr()>;

  Keyframe::Ptr TryCreate(double timestamp,
                          const Pose3d &T_odom_body,
                          const CloudFactory &cloud_factory,
                          const Matrix6d &odom_covariance);
  Keyframe::Ptr TryCreate(double timestamp,
                          const Pose3d &T_odom_body,
                          const KeyframeCloud::ConstPtr &cloud_body,
                          const Matrix6d &odom_covariance);

  std::vector<Keyframe::Ptr> keyframes() const;
  Statistics statistics() const;

  static std::string TriggerMaskToString(std::uint8_t trigger_mask);

private:
  static void ValidateOptions(const Options &options);
  static void ValidateInput(double timestamp,
                            const Pose3d &T_odom_body,
                            const Matrix6d &odom_covariance);
  static void ValidateCloud(const KeyframeCloud::ConstPtr &cloud_body);
  KeyframeCloud::ConstPtr PrepareCloud(
      const KeyframeCloud::ConstPtr &cloud_body) const;
  void WriteCsv(const Keyframe &keyframe);

  Options options_;
  const double rotation_threshold_rad_;

  mutable std::mutex mutex_;
  std::vector<Keyframe::Ptr> keyframes_;
  Statistics statistics_;
  bool has_observation_ = false;
  double last_observation_timestamp_ = 0.0;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_KEYFRAME_MANAGER_H
