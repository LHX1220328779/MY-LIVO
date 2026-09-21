#include "extrinsic_precheck.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace livo
{
namespace
{

constexpr double kDeg = M_PI / 180.0;

double median(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  return values[mid];
}

Eigen::Matrix3d smallRotation(double rot_x, double rot_y)
{
  Eigen::Vector3d w(rot_x, rot_y, 0.0);
  const double angle = w.norm();
  if (angle < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
}

std::string matrixJson(const Eigen::Matrix4d &m)
{
  std::ostringstream out;
  out << std::setprecision(10) << '[';
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out << (r == 0 && c == 0 ? "" : ", ") << m(r, c);
  out << ']';
  return out.str();
}

std::string quoted(const std::string &s)
{
  std::string out = "\"";
  for (char ch : s)
  {
    if (ch == '"' || ch == '\\') out += '\\';
    out += ch;
  }
  return out + "\"";
}

} // namespace

const ExtrinsicPrecheck::GroundCell *ExtrinsicPrecheck::GroundModel::find(std::int64_t key) const
{
  const auto it = std::lower_bound(keys.begin(), keys.end(), key);
  if (it == keys.end() || *it != key) return nullptr;
  return &cells[static_cast<std::size_t>(it - keys.begin())];
}

void ExtrinsicPrecheck::configure(const ExtrinsicPrecheckConfig &config,
                                  const std::vector<std::string> &topics,
                                  const std::vector<Eigen::Matrix4d> &rear_from_lidar,
                                  std::size_t reference_index)
{
  config_ = config;
  topics_ = topics;
  rear_from_lidar_ = rear_from_lidar;
  reference_index_ = reference_index;
  points_.assign(topics.size(), {});
  frames_.assign(topics.size(), 0);
  results_.clear();
  finished_ = !config.enabled || topics.size() < 2;
}

bool ExtrinsicPrecheck::isStatic(const std::vector<Eigen::Vector3d> &gyro,
                                 const std::vector<Eigen::Vector3d> &acc) const
{
  if (gyro.size() < 10 || acc.size() < 10) return false;
  Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
  for (const auto &g : gyro) gyro_mean += g;
  gyro_mean /= static_cast<double>(gyro.size());
  Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
  for (const auto &a : acc) acc_mean += a;
  acc_mean /= static_cast<double>(acc.size());
  double acc_var = 0.0;
  for (const auto &a : acc) acc_var += (a - acc_mean).squaredNorm();
  acc_var /= static_cast<double>(acc.size());
  return gyro_mean.norm() < config_.static_gyro_deg_s * kDeg &&
         std::sqrt(acc_var) < config_.static_acc_std_mps2;
}

void ExtrinsicPrecheck::addFrame(std::size_t source, const std::vector<Eigen::Vector3f> &points)
{
  if (finished_ || source >= points_.size()) return;
  if (frames_[source] >= static_cast<std::size_t>(config_.max_frames)) return;
  const auto &box = config_.body_box_m;
  const double min_r2 = config_.min_range_m * config_.min_range_m;
  const double max_r2 = config_.max_range_m * config_.max_range_m;
  auto &store = points_[source];
  store.reserve(store.size() + points.size() / config_.point_stride + 1);
  for (std::size_t i = 0; i < points.size(); i += config_.point_stride)
  {
    const Eigen::Vector3f &p = points[i];
    if (!p.allFinite()) continue;
    const double r2 = double(p.x()) * p.x() + double(p.y()) * p.y();
    if (r2 < min_r2 || r2 > max_r2) continue;
    if (p.x() > box[0] && p.x() < box[1] && p.y() > box[2] && p.y() < box[3]) continue;
    store.push_back(p);
  }
  ++frames_[source];
}

bool ExtrinsicPrecheck::saturated() const
{
  for (std::size_t i = 0; i < frames_.size(); ++i)
    if (frames_[i] < static_cast<std::size_t>(config_.max_frames)) return false;
  return !frames_.empty();
}

bool ExtrinsicPrecheck::sufficient() const
{
  for (std::size_t i = 0; i < frames_.size(); ++i)
    if (frames_[i] < static_cast<std::size_t>(config_.min_frames)) return false;
  return !frames_.empty();
}

std::int64_t ExtrinsicPrecheck::cellKey(const Eigen::Vector3d &p) const
{
  const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x() / config_.cell_size_m));
  const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y() / config_.cell_size_m));
  return ix * 1000003LL + iy;
}

ExtrinsicPrecheck::GroundModel ExtrinsicPrecheck::buildGroundModel(
    const std::vector<Eigen::Vector3f> &points) const
{
  GroundModel model;
  model.cell = config_.cell_size_m;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> bins;
  for (std::size_t i = 0; i < points.size(); ++i)
    bins[cellKey(points[i].cast<double>())].push_back(i);

  std::vector<std::pair<std::int64_t, GroundCell>> cells;
  for (auto &bin : bins)
  {
    auto &idx = bin.second;
    if (idx.size() < 20) continue;
    // Lowest surface in the cell: ground, not berms/vehicles/dust above it.
    std::vector<double> z;
    z.reserve(idx.size());
    for (std::size_t i : idx) z.push_back(points[i].z());
    std::vector<double> zs = z;
    const std::size_t k = zs.size() / 20;
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    const double z_low = zs[k] + 0.35;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    std::vector<std::size_t> kept;
    for (std::size_t j = 0; j < idx.size(); ++j)
      if (z[j] < z_low) { kept.push_back(idx[j]); mean += points[idx[j]].cast<double>(); }
    if (kept.size() < 12) continue;
    mean /= static_cast<double>(kept.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (std::size_t i : kept)
    {
      const Eigen::Vector3d d = points[i].cast<double>() - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(kept.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    const Eigen::Vector3d lambda = eig.eigenvalues(); // ascending
    if (lambda[0] > 0.03 * 0.03 || lambda[1] < 4.0 * lambda[0] + 1e-6) continue;
    Eigen::Vector3d n = eig.eigenvectors().col(0);
    if (n.z() < 0.0) n = -n;
    if (n.z() < 0.85) continue;
    GroundCell cell;
    cell.centroid = mean;
    cell.normal = n;
    cell.count = static_cast<int>(kept.size());
    cells.emplace_back(bin.first, cell);
  }
  std::sort(cells.begin(), cells.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });
  model.keys.reserve(cells.size());
  model.cells.reserve(cells.size());
  for (auto &c : cells) { model.keys.push_back(c.first); model.cells.push_back(c.second); }

  // Robust global plane through the cell centroids (only used when the caller
  // allows extrapolation beyond the reference LiDAR's own footprint).
  if (model.cells.size() >= 8)
  {
    std::vector<double> w(model.cells.size(), 1.0);
    for (int iter = 0; iter < 6; ++iter)
    {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      double wsum = 0.0;
      for (std::size_t i = 0; i < model.cells.size(); ++i)
      { mean += w[i] * model.cells[i].centroid; wsum += w[i]; }
      if (wsum <= 0.0) break;
      mean /= wsum;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (std::size_t i = 0; i < model.cells.size(); ++i)
      {
        const Eigen::Vector3d d = model.cells[i].centroid - mean;
        cov += w[i] * d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
      Eigen::Vector3d n = eig.eigenvectors().col(0);
      if (n.z() < 0.0) n = -n;
      model.global_normal = n;
      model.global_offset = -n.dot(mean);
      model.global_valid = n.z() > 0.85;
      for (std::size_t i = 0; i < model.cells.size(); ++i)
      {
        const double r = std::abs(n.dot(model.cells[i].centroid) + model.global_offset);
        w[i] = std::min(1.0, 0.10 / std::max(r, 1e-6));
      }
    }
  }
  return model;
}

Eigen::Matrix4d ExtrinsicPrecheck::rearCorrection(double rot_x, double rot_y, double dz,
                                                  const Eigen::Vector3d &about)
{
  Eigen::Matrix4d c = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d r = smallRotation(rot_x, rot_y);
  c.block<3, 3>(0, 0) = r;
  c.block<3, 1>(0, 3) = about - r * about + Eigen::Vector3d(0.0, 0.0, dz);
  return c;
}

void ExtrinsicPrecheck::solveSource(std::size_t source, const GroundModel &model,
                                    ExtrinsicPrecheckResult &result) const
{
  const auto &points = points_[source];
  const Eigen::Vector3d about = rear_from_lidar_[source].block<3, 1>(0, 3);
  Eigen::Vector3d x = Eigen::Vector3d::Zero(); // rot_x, rot_y, dz
  const double stats_gate = 0.5;
  const double huber = 0.05;
  Eigen::Matrix3d prior = Eigen::Matrix3d::Zero();
  prior(0, 0) = prior(1, 1) = 1.0 / ((1.0 * kDeg) * (1.0 * kDeg));
  prior(2, 2) = 1.0 / (0.10 * 0.10);

  auto evaluate = [&](const Eigen::Vector3d &state, double gate, bool accumulate,
                      Eigen::Matrix3d *H, Eigen::Vector3d *g, std::vector<double> *abs_res) {
    const Eigen::Matrix3d r = smallRotation(state[0], state[1]);
    const Eigen::Vector3d shift(0.0, 0.0, state[2]);
    int matches = 0;
    for (const Eigen::Vector3f &pf : points)
    {
      const Eigen::Vector3d p = pf.cast<double>();
      const Eigen::Vector3d q = r * (p - about) + about + shift;
      const GroundCell *cell = model.find(cellKey(q));
      Eigen::Vector3d n;
      double res;
      if (cell)
      {
        n = cell->normal;
        res = n.dot(q - cell->centroid);
      }
      else if (config_.allow_extrapolated_ground && model.global_valid)
      {
        n = model.global_normal;
        res = n.dot(q) + model.global_offset;
      }
      else
        continue;
      if (std::abs(res) > gate) continue;
      ++matches;
      if (abs_res) abs_res->push_back(std::abs(res));
      if (accumulate)
      {
        const Eigen::Vector3d lever = (q - about).cross(n);
        Eigen::Vector3d jac(lever.x(), lever.y(), n.z());
        const double w = std::min(1.0, huber / std::max(std::abs(res), 1e-9));
        *H += w * jac * jac.transpose();
        *g += w * jac * res;
      }
    }
    return matches;
  };

  std::vector<double> before;
  result.matches_before = evaluate(x, stats_gate, false, nullptr, nullptr, &before);
  result.median_before_m = median(before);
  if (result.matches_before < 500)
  {
    result.note = "insufficient ground overlap with the reference LiDAR while static";
    return;
  }
  for (int iter = 0; iter < 15; ++iter)
  {
    Eigen::Matrix3d H = prior;
    Eigen::Vector3d g = prior * x;
    const double gate = iter < 3 ? 0.5 : 0.25;
    const int matches = evaluate(x, gate, true, &H, &g, nullptr);
    if (matches < 200) break;
    const Eigen::Vector3d step = -H.ldlt().solve(g);
    x += step;
    if (step.head<2>().norm() < 1e-6 && std::abs(step[2]) < 1e-5) break;
  }
  std::vector<double> after;
  result.matches_after = evaluate(x, stats_gate, false, nullptr, nullptr, &after);
  result.median_after_m = median(after);
  result.rot_x_deg = x[0] / kDeg;
  result.rot_y_deg = x[1] / kDeg;
  result.dz_m = x[2];
  result.solved = true;
  result.correction = rearCorrection(x[0], x[1], x[2], about);
  result.rear_from_lidar_after = result.correction * result.rear_from_lidar_before;
  result.within_gates =
      std::abs(result.rot_x_deg) <= config_.max_rotation_deg &&
      std::abs(result.rot_y_deg) <= config_.max_rotation_deg &&
      std::abs(result.dz_m) <= config_.max_height_m;
  result.significant =
      std::abs(result.rot_x_deg) >= config_.min_apply_rotation_deg ||
      std::abs(result.rot_y_deg) >= config_.min_apply_rotation_deg ||
      std::abs(result.dz_m) >= config_.min_apply_height_m;
  if (result.median_after_m >= result.median_before_m)
  {
    result.within_gates = false;
    result.note = "no residual improvement; calibration kept";
  }
  else if (!result.within_gates)
    result.note = "correction exceeds the configured gates; calibration kept, please recalibrate";
  else if (!result.significant)
    result.note = "within the method's tolerance; calibration confirmed";
}

std::vector<ExtrinsicPrecheckResult> ExtrinsicPrecheck::solve()
{
  results_.clear();
  finished_ = true;
  if (points_.empty()) return results_;
  const GroundModel model = buildGroundModel(points_[reference_index_]);
  for (std::size_t s = 0; s < points_.size(); ++s)
  {
    ExtrinsicPrecheckResult result;
    result.topic = topics_[s];
    result.frames = frames_[s];
    result.rear_from_lidar_before = rear_from_lidar_[s];
    result.rear_from_lidar_after = rear_from_lidar_[s];
    if (s == reference_index_)
    {
      result.reference = true;
      result.note = "reference LiDAR (" + std::to_string(model.cells.size()) + " ground cells)";
    }
    else if (model.cells.size() < 8)
      result.note = "reference LiDAR did not observe enough planar ground";
    else if (frames_[s] == 0)
      result.note = "no static frames received from this LiDAR";
    else
      solveSource(s, model, result);
    if (result.solved && result.within_gates && result.significant && config_.apply)
    {
      result.applied = true;
      rear_from_lidar_[s] = result.rear_from_lidar_after;
    }
    results_.push_back(result);
  }
  return results_;
}

std::string ExtrinsicPrecheck::reportJson(const std::string &status, double stamp) const
{
  std::ostringstream out;
  out << std::setprecision(10);
  out << "{\n  \"status\": " << quoted(status) << ",\n  \"stamp\": " << std::fixed
      << std::setprecision(6) << stamp << std::defaultfloat << std::setprecision(10)
      << ",\n  \"reference_topic\": "
      << quoted(topics_.empty() ? std::string() : topics_[reference_index_])
      << ",\n  \"config\": {\"apply\": " << (config_.apply ? "true" : "false")
      << ", \"min_frames\": " << config_.min_frames << ", \"max_frames\": " << config_.max_frames
      << ", \"max_rotation_deg\": " << config_.max_rotation_deg
      << ", \"max_height_m\": " << config_.max_height_m
      << ", \"min_apply_rotation_deg\": " << config_.min_apply_rotation_deg
      << ", \"min_apply_height_m\": " << config_.min_apply_height_m
      << ", \"static_gyro_deg_s\": " << config_.static_gyro_deg_s
      << ", \"cell_size_m\": " << config_.cell_size_m
      << ", \"allow_extrapolated_ground\": " << (config_.allow_extrapolated_ground ? "true" : "false")
      << "},\n  \"sources\": [\n";
  for (std::size_t i = 0; i < results_.size(); ++i)
  {
    const auto &r = results_[i];
    out << "    {\"topic\": " << quoted(r.topic) << ", \"frames\": " << r.frames
        << ", \"reference\": " << (r.reference ? "true" : "false")
        << ", \"solved\": " << (r.solved ? "true" : "false")
        << ", \"within_gates\": " << (r.within_gates ? "true" : "false")
        << ", \"significant\": " << (r.significant ? "true" : "false")
        << ", \"applied\": " << (r.applied ? "true" : "false")
        << ",\n     \"matches_before\": " << r.matches_before
        << ", \"matches_after\": " << r.matches_after
        << ", \"median_before_cm\": " << 100.0 * r.median_before_m
        << ", \"median_after_cm\": " << 100.0 * r.median_after_m
        << ", \"rot_x_deg\": " << r.rot_x_deg << ", \"rot_y_deg\": " << r.rot_y_deg
        << ", \"dz_cm\": " << 100.0 * r.dz_m
        << ",\n     \"note\": " << quoted(r.note)
        << ",\n     \"rear_from_lidar_before\": " << matrixJson(r.rear_from_lidar_before)
        << ",\n     \"rear_from_lidar_after\": " << matrixJson(r.rear_from_lidar_after)
        << "}" << (i + 1 < results_.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
  return out.str();
}

} // namespace livo
