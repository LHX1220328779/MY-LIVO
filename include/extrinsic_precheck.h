#pragma once

// Static-start extrinsic self-check for the multi-LiDAR rear-axle rig.
//
// While the vehicle is still (first seconds of a run, the same window the IMU
// initialisation needs), the overlapping ground returns of every LiDAR are
// compared against the reference LiDAR in the common rear-axle frame. Ground
// overlap observes only the relative roll, pitch and height of each sensor, so
// only those three degrees of freedom are estimated (yaw and the horizontal
// offsets stay exactly as calibrated; they are refined offline from driven
// data). Corrections inside the configured gates are applied as a small rigid
// motion about the LiDAR's own position; anything larger is reported and left
// untouched so that a wrong file cannot be silently "repaired".

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace livo
{

struct ExtrinsicPrecheckConfig
{
  bool enabled = false;
  bool apply = true;                // false: report only, never touch the extrinsics
  int min_frames = 5;               // static frames per source needed to solve
  int max_frames = 12;              // stop accumulating after this many static frames
  double max_rotation_deg = 1.0;    // upper gate for |rot_x|, |rot_y| corrections
  double max_height_m = 0.15;       // upper gate for |z| corrections
  // Corrections below these magnitudes are inside the method's own uncertainty
  // (ground planarity over the static footprint) and are reported only.
  double min_apply_rotation_deg = 0.2;
  double min_apply_height_m = 0.04;
  double static_gyro_deg_s = 0.5;   // mean |gyro| below this: vehicle is still
  double static_acc_std_mps2 = 0.4; // accelerometer scatter below this: still
  double cell_size_m = 1.5;         // ground grid resolution in the rear frame
  double min_range_m = 3.0;         // planar radius window around the rear axle
  double max_range_m = 45.0;
  std::array<double, 4> body_box_m = {-2.8, 2.8, -4.5, 9.5}; // x_min x_max y_min y_max
  bool allow_extrapolated_ground = false; // use the fitted global plane where the
                                          // reference LiDAR has no returns
  std::size_t point_stride = 2;     // keep every n-th return while accumulating
};

struct ExtrinsicPrecheckResult
{
  std::string topic;
  std::size_t frames = 0;
  bool reference = false;
  bool solved = false;
  bool within_gates = false;
  bool significant = false;         // larger than the method's uncertainty
  bool applied = false;
  int matches_before = 0;
  int matches_after = 0;
  double median_before_m = 0.0;
  double median_after_m = 0.0;
  double rot_x_deg = 0.0;           // rotation about the rear-frame x axis (pitch-like, y forward)
  double rot_y_deg = 0.0;           // rotation about the rear-frame y axis (roll-like)
  double dz_m = 0.0;
  std::string note;
  Eigen::Matrix4d correction = Eigen::Matrix4d::Identity(); // rear frame, p' = C p
  Eigen::Matrix4d rear_from_lidar_before = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d rear_from_lidar_after = Eigen::Matrix4d::Identity();
};

class ExtrinsicPrecheck
{
public:
  void configure(const ExtrinsicPrecheckConfig &config,
                 const std::vector<std::string> &topics,
                 const std::vector<Eigen::Matrix4d> &rear_from_lidar,
                 std::size_t reference_index);

  bool enabled() const { return config_.enabled && !finished_; }
  bool finished() const { return finished_; }
  std::size_t referenceIndex() const { return reference_index_; }

  // Static-motion test on a window of raw IMU samples (rad/s, m/s^2).
  bool isStatic(const std::vector<Eigen::Vector3d> &gyro,
                const std::vector<Eigen::Vector3d> &acc) const;

  // Accumulate one static frame of one source; points are already expressed
  // in the rear-axle frame with the current calibration.
  void addFrame(std::size_t source, const std::vector<Eigen::Vector3f> &points);
  std::size_t frames(std::size_t source) const { return frames_[source]; }
  bool saturated() const;      // every source reached max_frames
  bool sufficient() const;     // every source reached min_frames

  // Solve roll/pitch/z per non-reference source. Marks the check finished.
  std::vector<ExtrinsicPrecheckResult> solve();
  const std::vector<ExtrinsicPrecheckResult> &results() const { return results_; }
  const ExtrinsicPrecheckConfig &config() const { return config_; }

  // JSON report written next to the run outputs; readable by the UI.
  std::string reportJson(const std::string &status, double stamp) const;

  // Static helpers shared with unit checks.
  static Eigen::Matrix4d rearCorrection(double rot_x, double rot_y, double dz,
                                        const Eigen::Vector3d &about);

private:
  struct GroundCell
  {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    int count = 0;
  };
  struct GroundModel
  {
    double cell = 1.5;
    std::vector<std::int64_t> keys;
    std::vector<GroundCell> cells;   // sorted by key
    bool global_valid = false;
    Eigen::Vector3d global_normal = Eigen::Vector3d::UnitZ();
    double global_offset = 0.0;      // n.p + offset = 0
    const GroundCell *find(std::int64_t key) const;
  };

  std::int64_t cellKey(const Eigen::Vector3d &p) const;
  GroundModel buildGroundModel(const std::vector<Eigen::Vector3f> &points) const;
  void solveSource(std::size_t source, const GroundModel &model,
                   ExtrinsicPrecheckResult &result) const;

  ExtrinsicPrecheckConfig config_;
  std::vector<std::string> topics_;
  std::vector<Eigen::Matrix4d> rear_from_lidar_;
  std::vector<std::vector<Eigen::Vector3f>> points_;
  std::vector<std::size_t> frames_;
  std::vector<ExtrinsicPrecheckResult> results_;
  std::size_t reference_index_ = 0;
  bool finished_ = false;
};

} // namespace livo
