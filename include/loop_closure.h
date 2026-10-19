#pragma once

// Submap-to-submap loop closure for the RTK/INS fusion backend (final optimization).
//
// Revisits of the same road section on different passes are detected from the solved keyframe positions
// (RTK keeps them within centimetres, so no place recognition is needed), a local submap is built around
// each keyframe from its archived native scans (native relative poses over +-submap_half_path metres are
// exact to millimetres), and the later submap is registered onto the earlier one with robust point-to-plane
// ICP. A verified registration becomes a relative-pose factor between the two keyframes whose information
// is the ICP Hessian (rescaled to a bounded number of effective returns), so unobservable directions stay
// unconstrained. RTK still anchors the absolute positions; the loop factors make overlapping passes agree
// at the registration precision (point-to-plane rms of a few cm), which RTK alone (2-3 cm per fix, time-
// correlated) cannot guarantee.

#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

struct LoopClosureConfig
{
  bool enabled = true;
  double radius = 12.0;           // max horizontal distance between the two keyframes (m)
  double min_path = 60.0;         // min driven path between them (m): a revisit, not a neighbour
  double min_age = 30.0;          // min time between them (s)
  double spacing = 4.0;           // min path between two loop pairs of the same pass (m)
  double submap_half_path = 12.0; // submap extent along the path on each side of the keyframe (m)
  double scan_spacing = 0.3;      // min time between scans merged into a submap (s)
  double voxel = 0.15;            // submap voxel (m)
  double max_range = 40.0;        // returns farther than this from their LiDAR are not used (m)
  double max_correction = 0.15;   // reject registrations moving the pair more than this (m): RTK keeps passes
  double max_correction_deg = 0.3; // within a few cm, a larger move is a wrong / biased association
  double max_rms = 0.08;          // inlier point-to-plane rms after registration (m)
  double min_overlap = 0.15;      // fraction of source returns with a valid plane correspondence (opposite-
                                  // direction passes share ~20 % of their voxels: different occlusions)
  double point_sigma = 0.05;      // point-to-plane noise used for the information matrix (m)
  int effective_points = 300;     // information of at most this many independent returns per loop
  double floor_translation = 0.02;  // systematic registration error added to the ICP covariance (m)
  double floor_rotation_deg = 0.03; // (deg); bounds the loop information so a loop never bends a pass
  int max_pairs = 600;
  // RTK bound of the final solve: if a keyframe ends farther than rtk_bound (m) from its RTK reference, the loop
  // factors with an end within rtk_bound_keys keyframes of it are dropped and the graph is solved again (loops may
  // only redistribute the RTK error; the delivered route must stay inside the 10 cm budget).
  double rtk_bound = 0.08;
  int rtk_bound_keys = 20;
};

struct LoopScanRef
{
  std::uint64_t offset = 0;   // byte offset of the scan in scans.bin
  std::uint32_t count = 0;    // returns
  double stamp = 0;
  gtsam::Pose3 raw;           // native scan-end IMU pose
  double path = 0;            // native path length from the first scan (m)
};

struct LoopPair
{
  std::size_t a = 0, b = 0;   // keyframe indices (a earlier)
  gtsam::Pose3 initial, measured;  // relative pose a->b before / after registration (IMU body frames)
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();  // GTSAM [rot, trans]
  double rms_before = 0, rms_after = 0, overlap = 0, correction = 0, correction_deg = 0;
  Eigen::Vector3d shift = Eigen::Vector3d::Zero();  // registered minus initial position of b, in a's body frame
  Eigen::Vector3d rotation_deg = Eigen::Vector3d::Zero();  // registered vs initial rotation of b, b body axes (deg)
  double min_eigen_translation = 0, min_eigen_rotation = 0;
  int correspondences = 0;
  bool accepted = false;
  std::string reason;
};

// Detect revisit pairs among keyframes (positions in the solved frame, native path per keyframe).
std::vector<std::pair<std::size_t, std::size_t>> detectLoopPairs(
    const LoopClosureConfig &config, const std::vector<Eigen::Vector3d> &positions,
    const std::vector<double> &paths, const std::vector<double> &stamps, const std::vector<bool> &usable);

// Register every pair (OpenMP across pairs). scans: every archived scan in time order; delivered: the delivered
// (corrected) pose of every scan; keyframe_scan: index of the scan of each keyframe; optimized: current solved
// keyframe poses (initial guesses; equal to delivered at the keyframe scans).
std::vector<LoopPair> registerLoopPairs(
    const LoopClosureConfig &config, const std::string &archive_path,
    const Eigen::Matrix3d &lidar_rotation, const Eigen::Vector3d &lidar_translation,
    const std::vector<LoopScanRef> &scans, const std::vector<gtsam::Pose3> &delivered,
    const std::vector<std::size_t> &keyframe_scan, const std::vector<gtsam::Pose3> &optimized,
    const std::vector<std::pair<std::size_t, std::size_t>> &pairs);
