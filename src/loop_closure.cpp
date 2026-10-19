#include "loop_closure.h"

#include <Eigen/Eigenvalues>
#include <gtsam/geometry/Rot3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_map>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Submap
{
  Cloud::Ptr cloud{new Cloud};
  std::vector<Eigen::Vector3f> normals;  // zero = no reliable plane at this point
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  std::size_t raw_points = 0;
};

// Scans within +-half path of the anchor scan, at least scan_spacing apart, expressed in the anchor's IMU frame
// with the DELIVERED poses (the geometry the loop factors must make consistent), voxel-averaged.
bool buildSubmap(const LoopClosureConfig &config, std::ifstream &archive, const Eigen::Matrix3d &lidar_rotation,
                 const Eigen::Vector3d &lidar_translation, const std::vector<LoopScanRef> &scans,
                 const std::vector<gtsam::Pose3> &delivered, std::size_t anchor, Submap &out)
{
  const gtsam::Pose3 anchor_inverse = delivered[anchor].inverse();
  std::vector<std::size_t> chosen{anchor};
  double last = scans[anchor].stamp;
  for (std::size_t s = anchor; s-- > 0;)
  {
    if (scans[anchor].path - scans[s].path > config.submap_half_path) break;
    if (last - scans[s].stamp >= config.scan_spacing) { chosen.push_back(s); last = scans[s].stamp; }
  }
  last = scans[anchor].stamp;
  for (std::size_t s = anchor + 1; s < scans.size(); ++s)
  {
    if (scans[s].path - scans[anchor].path > config.submap_half_path) break;
    if (scans[s].stamp - last >= config.scan_spacing) { chosen.push_back(s); last = scans[s].stamp; }
  }
  struct Cell { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); int n = 0; };
  std::unordered_map<std::uint64_t, Cell> cells;
  cells.reserve(1 << 18);
  std::vector<float> packed;
  const double inv_voxel = 1.0 / config.voxel;
  for (std::size_t s : chosen)
  {
    const auto &scan = scans[s];
    if (scan.count == 0) continue;
    packed.resize(static_cast<std::size_t>(scan.count) * 5);
    archive.clear();
    archive.seekg(static_cast<std::streamoff>(scan.offset));
    archive.read(reinterpret_cast<char *>(packed.data()), static_cast<std::streamsize>(packed.size() * sizeof(float)));
    if (!archive) return false;
    const gtsam::Pose3 to_anchor = anchor_inverse * delivered[s];
    const Eigen::Matrix3d rotation = to_anchor.rotation().matrix() * lidar_rotation;
    const Eigen::Vector3d translation = to_anchor.rotation().matrix() * lidar_translation + to_anchor.translation();
    for (std::size_t i = 0; i < scan.count; ++i)
    {
      const Eigen::Vector3d p(packed[5 * i], packed[5 * i + 1], packed[5 * i + 2]);
      const double range = p.norm();
      if (!std::isfinite(range) || range < 1.0 || range > config.max_range) continue;
      const Eigen::Vector3d q = rotation * p + translation;
      const auto key = [&](double v) {
        return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v * inv_voxel)) & 0x1FFFFF);
      };
      auto &cell = cells[(key(q.x()) << 42) | (key(q.y()) << 21) | key(q.z())];
      cell.sum += q;
      ++cell.n;
      ++out.raw_points;
    }
  }
  out.cloud->reserve(cells.size());
  for (const auto &entry : cells)
  {
    const Eigen::Vector3d c = entry.second.sum / entry.second.n;
    out.cloud->push_back(pcl::PointXYZ(static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z())));
  }
  if (out.cloud->size() < 500) return false;
  out.tree.setInputCloud(out.cloud);
  out.normals.assign(out.cloud->size(), Eigen::Vector3f::Zero());
  std::vector<int> ids;
  std::vector<float> d2;
  for (std::size_t i = 0; i < out.cloud->size(); ++i)
  {
    // 1 m patches: foliage and bushes look locally planar at the voxel scale and, seen from opposite sides,
    // their near-side surfaces would pull the two passes apart; only extended ground and vertical structure
    // (walls, kerbs, posts, sign boards) defines a loop constraint.
    if (out.tree.nearestKSearch(out.cloud->points[i], 20, ids, d2) < 12 || d2.back() > 1.0f) continue;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int id : ids) mean += out.cloud->points[id].getVector3fMap().cast<double>();
    mean /= ids.size();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int id : ids)
    {
      const Eigen::Vector3d d = out.cloud->points[id].getVector3fMap().cast<double>() - mean;
      cov += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov / ids.size());
    const auto &ev = eig.eigenvalues();  // ascending
    if (ev(2) <= 0 || ev(0) > 0.01 * (ev(0) + ev(1) + ev(2)) || ev(1) < 0.15 * ev(2)) continue;
    const Eigen::Vector3d n = eig.eigenvectors().col(0);
    if (std::abs(n.z()) < 0.9 && std::abs(n.z()) > 0.25) continue;  // ground or vertical only
    out.normals[i] = n.cast<float>();
  }
  return true;
}

// Robust point-to-plane ICP: source (b frame) onto target (a frame), T maps b -> a, right perturbation
// T <- T * Exp([w, v]) (GTSAM Pose3 tangent order), so the Hessian is the information of a
// BetweenFactor<Pose3>(a, b, T).
void registerPair(const LoopClosureConfig &config, const Submap &target, const Submap &source, LoopPair &pair)
{
  gtsam::Pose3 T = pair.initial;
  std::vector<int> ids(1);
  std::vector<float> d2(1);
  const double huber = 2.0 * config.point_sigma;
  auto evaluate = [&](const gtsam::Pose3 &pose, double gate, Eigen::Matrix<double, 6, 6> *H, Eigen::Matrix<double, 6, 1> *g,
                      double &rms, int &inliers) {
    const Eigen::Matrix3d R = pose.rotation().matrix();
    const Eigen::Vector3d t = pose.translation();
    double sum_w = 0, sum_wr2 = 0;
    inliers = 0;
    if (H) { H->setZero(); g->setZero(); }
    for (const auto &point : source.cloud->points)
    {
      const Eigen::Vector3d s = point.getVector3fMap().cast<double>();
      const Eigen::Vector3d q = R * s + t;
      pcl::PointXYZ qp(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()));
      if (target.tree.nearestKSearch(qp, 1, ids, d2) < 1 || d2[0] > gate * gate) continue;
      const Eigen::Vector3f &nf = target.normals[ids[0]];
      if (nf.squaredNorm() < 0.5f) continue;
      const Eigen::Vector3d n = nf.cast<double>();
      const double r = n.dot(q - target.cloud->points[ids[0]].getVector3fMap().cast<double>());
      const double w = std::abs(r) <= huber ? 1.0 : huber / std::abs(r);
      ++inliers;
      sum_w += w;
      sum_wr2 += w * r * r;
      if (H)
      {
        Eigen::Matrix<double, 1, 6> J;
        const Eigen::RowVector3d nR = n.transpose() * R;
        J.leftCols<3>() = -nR * gtsam::skewSymmetric(s);
        J.rightCols<3>() = nR;
        *H += w * J.transpose() * J;
        *g += w * J.transpose() * r;
      }
    }
    rms = sum_w > 0 ? std::sqrt(sum_wr2 / sum_w) : std::numeric_limits<double>::infinity();
  };
  int inliers = 0;
  double rms = 0;
  evaluate(T, 0.3, nullptr, nullptr, rms, inliers);
  pair.rms_before = rms;
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> g;
  for (double gate : {1.0, 0.5, 0.3, 0.2})
    for (int iteration = 0; iteration < 10; ++iteration)
    {
      evaluate(T, gate, &H, &g, rms, inliers);
      if (inliers < 100) { pair.reason = "too few plane correspondences"; return; }
      const Eigen::Matrix<double, 6, 1> step = -(H + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(g);
      if (!step.allFinite()) { pair.reason = "singular registration"; return; }
      T = T * gtsam::Pose3::Expmap(step);
      if (step.head<3>().norm() < 1e-5 && step.tail<3>().norm() < 1e-4) break;
    }
  evaluate(T, 0.2, &H, &g, rms, inliers);
  pair.measured = T;
  pair.rms_after = rms;
  pair.correspondences = inliers;
  pair.overlap = static_cast<double>(inliers) / std::max<std::size_t>(1, source.cloud->size());
  const gtsam::Pose3 change = pair.initial.between(T);
  pair.shift = T.translation() - pair.initial.translation();
  pair.correction = change.translation().norm();
  pair.rotation_deg = gtsam::Rot3::Logmap(change.rotation()) * 180.0 / M_PI;
  pair.correction_deg = pair.rotation_deg.norm();
  // Hessian of unit-weight residuals -> information of sigma-noise returns, capped at effective_points returns:
  // neighbouring voxels of one surface are not independent measurements.
  // ICP information (sigma-noise returns, at most effective_points independent ones: neighbouring voxels of
  // one surface are correlated) plus a systematic floor: covariance = H^-1 + floor. Unobservable directions keep
  // ~zero information.
  const Eigen::Matrix<double, 6, 6> H_icp = H / (config.point_sigma * config.point_sigma) *
                                            std::min(1.0, static_cast<double>(config.effective_points) / std::max(1, inliers));
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eh(H_icp);
  Eigen::Matrix<double, 6, 1> inv_eval;
  for (int k = 0; k < 6; ++k) inv_eval(k) = eh.eigenvalues()(k) > 1e-9 ? 1.0 / eh.eigenvalues()(k) : 1e12;
  Eigen::Matrix<double, 6, 6> covariance = eh.eigenvectors() * inv_eval.asDiagonal() * eh.eigenvectors().transpose();
  const double fr = config.floor_rotation_deg * M_PI / 180.0, ft = config.floor_translation;
  covariance.diagonal() += (Eigen::Matrix<double, 6, 1>() << fr * fr, fr * fr, fr * fr, ft * ft, ft * ft, ft * ft).finished();
  pair.information = covariance.inverse();
  pair.information = 0.5 * (pair.information + pair.information.transpose()).eval();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> et(pair.information.bottomRightCorner<3, 3>());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> er(pair.information.topLeftCorner<3, 3>());
  pair.min_eigen_translation = et.eigenvalues()(0);
  pair.min_eigen_rotation = er.eigenvalues()(0);
  if (pair.overlap < config.min_overlap) pair.reason = "overlap below threshold";
  else if (rms > config.max_rms) pair.reason = "rms above threshold";
  else if (pair.correction > config.max_correction || pair.correction_deg > config.max_correction_deg)
    pair.reason = "correction too large (wrong association)";
  else
  {
    pair.accepted = true;
    pair.reason = "ok";
  }
}
}  // namespace

std::vector<std::pair<std::size_t, std::size_t>> detectLoopPairs(
    const LoopClosureConfig &config, const std::vector<Eigen::Vector3d> &positions,
    const std::vector<double> &paths, const std::vector<double> &stamps, const std::vector<bool> &usable)
{
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  double last_kept_path = -1e18;
  for (std::size_t j = 0; j < positions.size(); ++j)
  {
    if (!usable[j] || paths[j] - last_kept_path < config.spacing) continue;
    double best = config.radius;
    std::size_t best_i = j;
    for (std::size_t i = 0; i < j; ++i)
    {
      if (!usable[i] || paths[j] - paths[i] < config.min_path || stamps[j] - stamps[i] < config.min_age) continue;
      const double d = (positions[j] - positions[i]).head<2>().norm();
      if (d < best && std::abs(positions[j].z() - positions[i].z()) < 3.0) { best = d; best_i = i; }
    }
    if (best_i != j)
    {
      pairs.emplace_back(best_i, j);
      last_kept_path = paths[j];
    }
  }
  if (static_cast<int>(pairs.size()) > config.max_pairs)
  {
    std::vector<std::pair<std::size_t, std::size_t>> thinned;
    for (int k = 0; k < config.max_pairs; ++k)
      thinned.push_back(pairs[static_cast<std::size_t>(k) * pairs.size() / config.max_pairs]);
    pairs.swap(thinned);
  }
  return pairs;
}

std::vector<LoopPair> registerLoopPairs(
    const LoopClosureConfig &config, const std::string &archive_path,
    const Eigen::Matrix3d &lidar_rotation, const Eigen::Vector3d &lidar_translation,
    const std::vector<LoopScanRef> &scans, const std::vector<gtsam::Pose3> &delivered,
    const std::vector<std::size_t> &keyframe_scan, const std::vector<gtsam::Pose3> &optimized,
    const std::vector<std::pair<std::size_t, std::size_t>> &pairs)
{
  std::vector<LoopPair> results(pairs.size());
#pragma omp parallel for schedule(dynamic, 1)
  for (int n = 0; n < static_cast<int>(pairs.size()); ++n)
  {
    LoopPair &pair = results[n];
    pair.a = pairs[n].first;
    pair.b = pairs[n].second;
    pair.initial = optimized[pair.a].between(optimized[pair.b]);
    pair.measured = pair.initial;
    std::ifstream archive(archive_path, std::ios::binary);
    Submap target, source;
    if (!archive || !buildSubmap(config, archive, lidar_rotation, lidar_translation, scans, delivered,
                                 keyframe_scan[pair.a], target) ||
        !buildSubmap(config, archive, lidar_rotation, lidar_translation, scans, delivered,
                     keyframe_scan[pair.b], source))
    {
      pair.reason = "submap unavailable";
      continue;
    }
    registerPair(config, target, source, pair);
  }
  return results;
}
