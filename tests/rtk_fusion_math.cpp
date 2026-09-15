#include "rtk_factors.h"
#include "voxel_map.h"
#include "rtk_fusion.h"
#include "IMU_Processing.h"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <iostream>
#include <stdexcept>

namespace
{
// Offline causal audit of an EXISTING run. No parameters, ROI, discarded
// observations or counterfactual trajectory are fed back into the mapper.
class MotionRowsAuditFactor final : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
  NativeMotionFactor original_;
  int removed_row_;
public:
  explicit MotionRowsAuditFactor(const NativeMotionFactor &factor, int removed_row = 3)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
          gtsam::noiseModel::Isotropic::Sigma(6, 1), factor.keys()[0], factor.keys()[1]),
        original_(factor), removed_row_(removed_row) {}
  gtsam::Vector evaluateError(const gtsam::Pose3 &a, const gtsam::Pose3 &b,
      gtsam::OptionalMatrixType ha = nullptr, gtsam::OptionalMatrixType hb = nullptr) const override
  {
    auto r = original_.evaluateError(a, b, ha, hb);
    r.segment<3>(removed_row_).setZero();
    if (ha) ha->middleRows(removed_row_, 3).setZero();
    if (hb) hb->middleRows(removed_row_, 3).setZero();
    return r;
  }
};


void auditSavedMotion(const std::filesystem::path &run, const std::filesystem::path &seed_file,
                     const std::filesystem::path &destination,
                     const std::filesystem::path &velocity_geometry = {})
{
  using gtsam::Pose3;
  using gtsam::Rot3;
  const auto output = projectResultDirectory(destination);
  std::filesystem::create_directories(output);
  const auto table = [](const std::filesystem::path &path, char separator) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot read audit input " + path.string());
    std::vector<std::vector<double>> rows;
    for (std::string line; std::getline(stream, line);)
    {
      if (line.empty() || line[0] == '#' || (!std::isdigit(line[0]) && line[0] != '-')) continue;
      if (separator == ',') std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream values(line);
      std::vector<double> row;
      for (std::string value; values >> value;) row.push_back(std::stod(value));
      rows.push_back(std::move(row));
    }
    if (rows.empty()) throw std::runtime_error("Empty audit input " + path.string());
    return rows;
  };
  const auto pose = [](const std::vector<double> &r) {
    return Pose3(Rot3::Quaternion(r.at(7), r.at(4), r.at(5), r.at(6)), V3D(r.at(1), r.at(2), r.at(3)));
  };
  const auto raw_rows = table(run / "trajectory_lio.txt", ' ');
  const auto refined_rows = table(run / "trajectory_optimized.txt", ' ');
  const auto keys = table(run / "keyframes.csv", ',');
  const auto seed = seed_file == "-" ? std::vector<double>{0, 0, 0, 0, 0, 0, 0, 1} : table(seed_file, ' ').front();
  const V3D lever(.904, -4.679, -1.699);
  std::map<double, std::size_t> indices;
  std::vector<Pose3> raw, refined;
  gtsam::Values values;
  const auto at = [&](const auto &rows, double stamp) {
    const auto found = std::min_element(rows.begin(), rows.end(), [stamp](const auto &a, const auto &b) {
      return std::abs(a[0] - stamp) < std::abs(b[0] - stamp);
    });
    if (std::abs(found->at(0) - stamp) > 1e-5) throw std::runtime_error("Audit key has no exact scan");
    return pose(*found);
  };
  for (std::size_t i = 0; i < keys.size(); ++i)
  {
    indices[keys[i][0]] = i;
    raw.push_back(at(raw_rows, keys[i][0]));
    refined.push_back(at(refined_rows, keys[i][0]));
    values.insert(gtsam::Symbol('x', i), refined.back());
  }
  gtsam::NonlinearFactorGraph graph;
  Pose3 reference_seed = pose(seed);
  gtsam::Vector6 prior_sigma; prior_sigma << .01, .01, .05, 10, 10, 10;
  graph.emplace_shared<gtsam::PriorFactor<Pose3>>(gtsam::Symbol('x', 0),
      Pose3(reference_seed.rotation(), reference_seed.translation() - reference_seed.rotation().rotate(lever)),
      gtsam::noiseModel::Diagonal::Sigmas(prior_sigma));
  // New runs record the actual datum/covariance, not an assumed gauge.
  // Keep the legacy seed contract only for historical forensic comparison.
  {
    std::ifstream stream(run / "optimization.csv");
    Pose3 initial_prior;
    gtsam::Matrix6 covariance;
    bool have_pose = false, have_covariance = false;
    for (std::string line; std::getline(stream, line);)
    {
      std::istringstream fields(line);
      std::string marker, name; fields >> marker >> name;
      if (marker != "#") continue;
      if (name == "initial_prior_pose")
      {
        std::vector<double> row(8);
        for (double &v : row) if (!(fields >> v)) throw std::runtime_error("Incomplete recorded initial pose");
        initial_prior = pose(row); have_pose = true;
      }
      else if (name == "initial_prior_covariance")
      {
        for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j)
          if (!(fields >> covariance(i, j))) throw std::runtime_error("Incomplete recorded initial covariance");
        have_covariance = true;
      }
    }
    if (have_pose != have_covariance) throw std::runtime_error("Partial initial observation log");
    if (!have_pose && seed_file == "-") throw std::runtime_error("Legacy run requires its original initialization seed");
    if (have_pose) graph[0] = std::make_shared<gtsam::PriorFactor<Pose3>>(gtsam::Symbol('x', 0), initial_prior,
        gtsam::noiseModel::Gaussian::Covariance(covariance));
    if (seed_file == "-")
    {
      if (!keys[0][1] && velocity_geometry != "--verify-objective")
        throw std::runtime_error("Startup counterfactual requires a qualified first reference");
      reference_seed = Pose3(initial_prior.rotation(), V3D(keys[0][3], keys[0][4], keys[0][5]));
    }
  }
  std::vector<std::size_t> failed_factors;
  std::ofstream influence(output / "increment_influence.csv");
  influence << std::setprecision(17) << "a_stamp,b_stamp,translation_error_m,translation_cost,angular_cost,translation_torque_a,translation_torque_b,removed_in_counterfactual\n";
  for (std::size_t i = 0; i < keys.size(); ++i)
  {
    const auto &row = keys[i];
    if (row[1] != 0)
    {
      M3D cov; cov << row[8], row[9], row[10], row[9], row[11], row[12], row[10], row[12], row[13];
      const auto noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(2.8 * std::sqrt(row[14])),
          gtsam::noiseModel::Gaussian::Covariance(cov));
      graph.emplace_shared<gtsam::GPSFactorArm>(gtsam::Symbol('x', i), V3D(row[3], row[4], row[5]), lever, noise);
    }
    if (!i) continue;
    const double dt = row[0] - keys[i - 1][0];
    gtsam::Vector6 sigmas;
    sigmas << .02 * M_PI / 180, .02 * M_PI / 180, .03 * M_PI / 180, .03, .03, .02;
    sigmas *= std::sqrt(dt / .5);
    const auto factor = std::make_shared<NativeMotionFactor>(gtsam::Symbol('x', i - 1), gtsam::Symbol('x', i),
        raw[i - 1].between(raw[i]), sigmas, std::sqrt(dt));
    gtsam::Matrix a, b;
    const auto r = factor->evaluateError(refined[i - 1], refined[i], &a, &b);
    const double displacement_error = factor->physicalError(refined[i - 1], refined[i]).tail<3>().norm();
    // A forensic ablation of unequivocally metre-scale failed increments,
    // not a proposed online threshold or a filter used by production.
    const bool remove = displacement_error > 1.0;
    if (remove) failed_factors.push_back(graph.size());
    influence << keys[i - 1][0] << ',' << row[0] << ',' << displacement_error << ','
        << .5 * r.tail<3>().squaredNorm() << ',' << .5 * r.head<3>().squaredNorm() << ','
        << (a.bottomLeftCorner(3, 3).transpose() * r.tail<3>()).norm() << ','
        << (b.bottomLeftCorner(3, 3).transpose() * r.tail<3>()).norm() << ',' << remove << '\n';
    graph.push_back(factor);
  }
  std::map<std::pair<std::size_t, std::size_t>, std::vector<ScanPlaneBatchFactor::Constraint>> patches;
  for (const auto &r : table(run / "geometry_constraints.csv", ','))
  {
    const auto a = indices.at(r[0]), b = indices.at(r[1]);
    ScanPlaneBatchFactor::Constraint surface{refined[a].rotation().unrotate(V3D(r[8], r[9], r[10])),
        refined[a].transformTo(V3D(r[2], r[3], r[4])), refined[b].transformTo(V3D(r[5], r[6], r[7])), r[11], r.at(14) != 0};
    if (r.size() >= 24)
      for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
        surface.source_sqrt(row, col) = r[15 + 3 * row + col];
    if (r.size() >= 34)
      for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
        surface.target_sqrt(row, col) = r[25 + 3 * row + col];
    patches[{a,b}].push_back(std::move(surface));
  }
  for (auto &p : patches) graph.emplace_shared<ScanPlaneBatchFactor>(gtsam::Symbol('x', p.first.first),
      gtsam::Symbol('x', p.first.second), std::move(p.second));
  double groups[4] = {};
  for (const auto &f : graph)
    groups[dynamic_cast<gtsam::GPSFactorArm *>(f.get()) ? 0 : dynamic_cast<NativeMotionFactor *>(f.get()) ? 1 :
           dynamic_cast<ScanPlaneBatchFactor *>(f.get()) ? 2 : 3] += f->error(values);
  std::cout << std::setprecision(17) << "Reconstructed fixed-graph costs RTK/motion/geometry/gauge="
      << groups[0] << ',' << groups[1] << ',' << groups[2] << ',' << groups[3] << '\n';
  std::ifstream optimization(run / "optimization.csv");
  std::vector<std::string> last;
  std::map<double, double> online_costs;
  for (std::string line; std::getline(optimization, line);)
  {
    std::istringstream fields(line);
    std::vector<std::string> row;
    for (std::string field; std::getline(fields, field, ',');) row.push_back(field);
    if (row.size() == 10 && row[1] == "final") last = std::move(row);
    else if (row.size() == 10 && row[1] == "online") online_costs[std::stod(row[0])] = std::stod(row[4]);
  }
  if (last.empty()) throw std::runtime_error("Saved final objective is required for causal audit");
  for (int i = 0; i < 4; ++i)
  {
    const double recorded = std::stod(last[i + 5]);
    if (!std::isfinite(groups[i]) || std::abs(recorded - groups[i]) > 5e-7 * std::max(1., std::abs(recorded)))
      throw std::runtime_error("Audit does not reproduce the saved factor model/parameters; refusing counterfactual conclusions");
  }
  std::cout << "All four cost groups reproduce optimization.csv within rounding tolerance.\n";
  if (velocity_geometry == "--verify-objective") return;
  if (velocity_geometry == "--online-convergence")
  {
    // A verified-observation graph has immutable factors, created no earlier
    // than their last key. Its exact prefix is therefore the online objective
    // at that key, without future observations or any changed correspondence.
    std::ifstream settings(run / "parameters.txt");
    std::string model; std::getline(settings, model);
    if (model.find("association_lifetime=verified_observation") == std::string::npos)
      throw std::runtime_error("Historical rematching prevents exact online-prefix reconstruction");
    std::map<std::size_t, double> online;
    for (const auto &row : table(run / "diagnostics.csv", ','))
    {
      if (row.size() < 20 || !std::isfinite(row[18])) continue;
      const auto found = indices.lower_bound(row[16] - 1e-6);
      if (found != indices.end() && std::abs(found->first - row[16]) < 1e-5)
        online[found->second] = std::max(online[found->second], row[18]);
    }
    std::vector<std::pair<double, std::size_t>> ranked;
    for (const auto &item : online) ranked.emplace_back(item.second, item.first);
    std::sort(ranked.rbegin(), ranked.rend());
    std::ofstream report(output / "online_prefix_convergence.csv");
    report << std::setprecision(17) << "last_stamp,keys,online_rtk_m,prefix_optimum_rtk_m,iterations,online_cost,seed_cost,prefix_cost,scaled_gradient_inf\n";
    for (std::size_t sample = 0; sample < std::min<std::size_t>(5, ranked.size()); ++sample)
    {
      const std::size_t last_key = ranked[sample].second;
      gtsam::NonlinearFactorGraph prefix;
      for (const auto &f : graph)
        if (std::all_of(f->keys().begin(), f->keys().end(), [last_key](gtsam::Key k) {
              return gtsam::Symbol(k).index() <= last_key; })) prefix.push_back(f);
      gtsam::Values initial;
      for (std::size_t i = 0; i <= last_key; ++i) initial.insert(gtsam::Symbol('x', i), refined[i]);
      gtsam::LevenbergMarquardtParams params;
      params.maxIterations = 100; params.relativeErrorTol = 1e-9; params.absoluteErrorTol = 1e-9;
      params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
      gtsam::LevenbergMarquardtOptimizer optimizer(prefix, initial, params);
      const auto solution = optimizer.optimize();
      const auto linear = prefix.linearize(solution);
      const auto gradient = linear->gradientAtZero(), diagonal = linear->hessianDiagonal();
      double norm = 0;
      for (const auto &item : gradient) norm = std::max(norm,
          (item.second.array().abs() / diagonal.at(item.first).array().max(1e-20).sqrt()).maxCoeff());
      const V3D reference(keys[last_key][3], keys[last_key][4], keys[last_key][5]);
      report << keys[last_key][0] << ',' << last_key + 1 << ',' << ranked[sample].first << ','
          << (solution.at<Pose3>(gtsam::Symbol('x', last_key)).transformFrom(lever) - reference).norm() << ','
          << optimizer.iterations() << ',' << online_costs.at(keys[last_key][0]) << ','
          << prefix.error(initial) << ',' << prefix.error(solution) << ',' << norm << '\n';
    }
    std::cout << "Saved fixed-measurement prefix solutions; no future factors and no production poses changed.\n";
    return;
  }
  if (velocity_geometry == "--geometry-baseline")
  {
    std::ofstream report(output / "geometry_baseline.csv");
    report << std::setprecision(17) << "a_stamp,b_stamp,rows,cost_before,cost_after,pose_change_m,pose_change_deg,rtk_baseline_m,registered_baseline_m,baseline_difference_m,geometry_baseline_sigma_m,rtk_baseline_sigma_m,baseline_z_score\n";
    gtsam::Vector6 units; units << 1./30, 1./30, 1./30, 1, 1, 1;
    for (const auto &entry : graph)
    {
      const auto factor = std::dynamic_pointer_cast<ScanPlaneBatchFactor>(entry);
      if (!factor) continue;
      const auto a = gtsam::Symbol(factor->keys()[0]).index(), b = gtsam::Symbol(factor->keys()[1]).index();
      if (!keys[a][1] || !keys[b][1]) continue;
      const auto original = refined[a].between(refined[b]);
      auto relative = original;
      const auto cost = [&](const Pose3 &p) { return .5 * factor->evaluateError(Pose3(), p).squaredNorm(); };
      for (int iteration = 0; iteration < 30; ++iteration)
      {
        gtsam::Matrix jac;
        const auto residual = factor->evaluateError(Pose3(), relative, nullptr, &jac);
        jac = (jac * units.asDiagonal()).eval();
        Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> spectrum(jac.transpose() * jac);
        const gtsam::Vector6 gradient = jac.transpose() * residual;
        gtsam::Vector6 step = gtsam::Vector6::Zero();
        for (int i = 0; i < 6; ++i)
          if (spectrum.eigenvalues()(i) > std::max(1e-9, 1e-6 * spectrum.eigenvalues()(5)))
            step -= spectrum.eigenvectors().col(i) * (spectrum.eigenvectors().col(i).dot(gradient) / spectrum.eigenvalues()(i));
        step = step.cwiseProduct(units);
        step /= std::max({1., step.head<3>().norm() / .02, step.tail<3>().norm() / .3});
        bool accepted = false;
        for (int trial = 0; trial < 8; ++trial)
        {
          const auto proposal = relative.retract(step);
          if (cost(proposal) <= cost(relative)) { relative = proposal; accepted = true; break; }
          step *= .5;
        }
        if (!accepted || step.norm() < 1e-7) break;
      }
      const V3D delta = relative.transformFrom(lever) - lever;
      const V3D reference = V3D(keys[b][3], keys[b][4], keys[b][5]) - V3D(keys[a][3], keys[a][4], keys[a][5]);
      if (delta.norm() < 1e-6 || reference.norm() < 1e-6) continue;
      Eigen::Matrix<double, 1, 6> derivative;
      derivative.head<3>() = -delta.normalized().transpose() * relative.rotation().matrix() * gtsam::skewSymmetric(lever);
      derivative.tail<3>() = delta.normalized().transpose() * relative.rotation().matrix();
      derivative = (derivative * units.asDiagonal()).eval();
      gtsam::Matrix jac;
      factor->evaluateError(Pose3(), relative, nullptr, &jac);
      jac = (jac * units.asDiagonal()).eval();
      Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> spectrum(jac.transpose() * jac);
      double geometry_variance = 0;
      for (int i = 0; i < 6; ++i)
      {
        const double response = derivative.dot(spectrum.eigenvectors().col(i));
        if (spectrum.eigenvalues()(i) <= std::max(1e-9, 1e-6 * spectrum.eigenvalues()(5)))
        { if (std::abs(response) > 1e-6) geometry_variance = std::numeric_limits<double>::infinity(); }
        else geometry_variance += response * response / spectrum.eigenvalues()(i);
      }
      M3D covariance = M3D::Zero();
      for (auto i : {a, b})
      {
        const auto &r = keys[i]; M3D c;
        c << r[8], r[9], r[10], r[9], r[11], r[12], r[10], r[12], r[13];
        covariance += c;
      }
      const double position_variance = reference.normalized().dot(covariance * reference.normalized());
      const auto change = original.between(relative);
      report << keys[a][0] << ',' << keys[b][0] << ',' << factor->constraints().size() << ',' << cost(original) << ',' << cost(relative) << ','
          << change.translation().norm() << ',' << Rot3::Logmap(change.rotation()).norm() * 180 / M_PI << ','
          << reference.norm() << ',' << delta.norm() << ',' << delta.norm() - reference.norm() << ',' << std::sqrt(geometry_variance) << ','
          << std::sqrt(position_variance) << ',' << (delta.norm() - reference.norm()) / std::sqrt(geometry_variance + position_variance) << '\n';
    }
    std::cout << "Saved same-pair geometry-only registration preferences and rotation-invariant rear-axle distance consistency. No graph/map changed.\n";
    return;
  }
  const std::string gravity_argument = "--initial-gravity=";
  const bool gravity_anchor = velocity_geometry.string().rfind(gravity_argument, 0) == 0;
  if (!velocity_geometry.empty() && !gravity_anchor)
  {
    std::map<std::pair<std::size_t, std::size_t>, std::vector<ScanPlaneBatchFactor::Constraint>> adjusted;
    for (const auto &r : table(velocity_geometry, ','))
    {
      const auto a = indices.at(r[0]), b = indices.at(r[1]);
      adjusted[{a,b}].push_back({refined[a].rotation().unrotate(V3D(r[8], r[9], r[10])),
          refined[a].transformTo(V3D(r[2], r[3], r[4])), refined[b].transformTo(V3D(r[5], r[6], r[7])), r[11], r.at(14) != 0});
    }
    auto diagnostic = graph;
    for (std::size_t i = 0; i < graph.size(); ++i)
    {
      const auto original = std::dynamic_pointer_cast<ScanPlaneBatchFactor>(graph[i]);
      if (!original) continue;
      auto &rows = adjusted.at({gtsam::Symbol(original->keys()[0]).index(), gtsam::Symbol(original->keys()[1]).index()});
      if (rows.size() != original->constraints().size()) throw std::runtime_error("Velocity audit changed correspondence count");
      for (std::size_t j = 0; j < rows.size(); ++j)
        if (std::abs(rows[j].sigma - original->constraints()[j].sigma) > 1e-12 ||
            rows[j].linear != original->constraints()[j].linear)
          throw std::runtime_error("Velocity audit changed original information or feature type");
      diagnostic[i] = std::make_shared<ScanPlaneBatchFactor>(original->keys()[0], original->keys()[1], std::move(rows));
    }
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = 100; params.relativeErrorTol = 1e-9;
    params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
    gtsam::LevenbergMarquardtOptimizer optimizer(diagnostic, values, params);
    const auto solution = optimizer.optimize();
    std::ofstream comparison(output / "deskew_geometry_counterfactual.csv");
    comparison << std::setprecision(17) << "stamp,original_rtk_m,velocity_geometry_rtk_m,rotation_change_deg\n";
    std::ofstream poses(output / "diagnostic_key_poses.txt");
    poses << std::setprecision(17) << "# DIAGNOSTIC physical IMU keys; velocity-only source substitution; NOT a production map\n";
    for (std::size_t i = 0; i < keys.size(); ++i)
    {
      const auto changed = solution.at<Pose3>(gtsam::Symbol('x', i));
      const V3D reference(keys[i][3], keys[i][4], keys[i][5]);
      comparison << keys[i][0] << ',' << (refined[i].transformFrom(lever) - reference).norm() << ','
          << (changed.transformFrom(lever) - reference).norm() << ','
          << Rot3::Logmap(refined[i].rotation().between(changed.rotation())).norm() * 180 / M_PI << '\n';
      const auto q = changed.rotation().toQuaternion();
      poses << keys[i][0] << ' ' << changed.translation().transpose() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
    std::cout << "Only scan velocity contribution changed on fixed source supports: cost " << diagnostic.error(values)
              << " -> " << diagnostic.error(solution) << "; iterations=" << optimizer.iterations() << '\n';
    return;
  }
  auto counterfactual = graph;
  if (gravity_anchor)
  {
    const auto source = std::filesystem::path(velocity_geometry.string().substr(gravity_argument.size()));
    const auto recorded = table(source / "trajectory_lio.txt", ' ');
    if (recorded.size() != raw_rows.size()) throw std::runtime_error("Different native gravity source length");
    for (std::size_t i = 0; i < raw_rows.size(); ++i)
      for (std::size_t j = 0; j < raw_rows[i].size(); ++j)
        if (std::abs(recorded[i][j] - raw_rows[i][j]) > 1e-9)
          throw std::runtime_error("Gravity source does not have the identical unassisted frontend");
    const auto observation = table(source.parent_path().parent_path() / "lio_geometry.csv", ',').front();
    if (observation.size() != 35 || std::abs(observation[0] - keys[0][0]) > 1e-5)
      throw std::runtime_error("Missing initial gravity measurement");
    const V3D gravity(observation[32], observation[33], observation[34]);
    const auto rotation = Rot3(Eigen::Quaterniond::FromTwoVectors(gravity, V3D(0, 0, -1))) * raw[0].rotation();
    const V3D axis = rotation.unrotate(V3D(0, 0, -1));
    // Same recorded stationary initialization: 200 samples, configured
    // discrete acceleration variance 0.1, initial bias variance 1e-5.
    // This is a finite inertial observation, NOT a fixed first pose.
    const double variance = (.1 / 200 + 1e-5) / (G_m_s2 * G_m_s2);
    gtsam::Matrix6 covariance = gtsam::Matrix6::Identity() * 100;
    covariance.topLeftCorner<3, 3>() = variance * (M3D::Identity() - axis * axis.transpose()) +
        .05 * .05 * axis * axis.transpose();
    counterfactual[0] = std::make_shared<gtsam::PriorFactor<Pose3>>(gtsam::Symbol('x', 0),
        Pose3(rotation, reference_seed.translation() - rotation.rotate(lever)), gtsam::noiseModel::Gaussian::Covariance(covariance));
    std::cout << "Counterfactual replaces ONLY arbitrary initial RP gauge with recorded stationary gravity and configured finite uncertainty; sigma="
        << std::sqrt(variance) * 180 / M_PI << " deg; no continuous attitude prior or frozen history.\n";
  }
  else
  {
    for (auto i : failed_factors)
      counterfactual[i] = std::make_shared<MotionRowsAuditFactor>(*std::dynamic_pointer_cast<NativeMotionFactor>(graph[i]));
    std::cout << "Counterfactual removes ONLY translation rows of " << failed_factors.size()
              << " metre-scale native failures; all RTK, angular and geometric observations unchanged.\n";
  }
  std::ofstream comparison(output / "startup_counterfactual.csv");
  comparison << std::setprecision(17) << "stamp,original_saved_rtk_m,original_reoptimized_rtk_m,counterfactual_rtk_m\n";
  std::array<gtsam::Values, 2> solutions;
  for (int trial = 0; trial < 2; ++trial)
  {
    const auto &objective = trial ? counterfactual : graph;
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = 100; params.relativeErrorTol = 1e-9; params.absoluteErrorTol = 1e-9;
    params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
    gtsam::LevenbergMarquardtOptimizer optimizer(objective, values, params);
    solutions[trial] = optimizer.optimize();
    std::cout << (trial ? "Counterfactual" : "Original fixed graph") << " cost "
        << objective.error(values) << " -> " << objective.error(solutions[trial]) << "; iterations=" << optimizer.iterations() << '\n';
  }
  for (std::size_t i = 0; i < keys.size(); ++i)
  {
    if (!keys[i][1]) continue;
    const V3D p(keys[i][3], keys[i][4], keys[i][5]);
    comparison << keys[i][0] << ',' << (refined[i].transformFrom(lever) - p).norm();
    for (const auto &s : solutions)
      comparison << ',' << (s.at<Pose3>(gtsam::Symbol('x', i)).transformFrom(lever) - p).norm();
    comparison << '\n';
  }
  if (gravity_anchor) return;
  std::ofstream prefix(output / "prefix_counterfactual.csv");
  prefix << std::setprecision(17) << "last_stamp,keys,seed,cost_before,cost_after,first_rtk_m,first_tilt_from_native_deg\n";
  for (double duration : {16., 25., 60., 80., 118.})
  {
    std::size_t count = 0;
    while (count < keys.size() && keys[count][0] <= keys.front()[0] + duration + 1e-6) ++count;
    gtsam::NonlinearFactorGraph local;
    for (const auto &f : graph)
      if (std::all_of(f->keys().begin(), f->keys().end(), [count](gtsam::Key k) { return gtsam::Symbol(k).index() < count; }))
        local.push_back(f);
    for (int seed_kind = 0; seed_kind < (duration == 16. ? 2 : 1); ++seed_kind)
    {
      gtsam::Values initial;
      const Pose3 first(reference_seed.rotation(), reference_seed.translation() - reference_seed.rotation().rotate(lever));
      for (std::size_t i = 0; i < count; ++i)
        initial.insert(gtsam::Symbol('x', i), seed_kind ? first * raw[0].between(raw[i]) : refined[i]);
      gtsam::LevenbergMarquardtParams params;
      params.maxIterations = 100; params.relativeErrorTol = 1e-9;
      params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
      gtsam::LevenbergMarquardtOptimizer optimizer(local, initial, params);
      const auto solution = optimizer.optimize();
      const auto first_pose = solution.at<Pose3>(gtsam::Symbol('x', 0));
      const double tilt = std::acos(std::clamp((first_pose.rotation() * raw[0].rotation().inverse()).matrix()(2, 2), -1., 1.));
      prefix << keys[count - 1][0] << ',' << count << ',' << seed_kind << ',' << local.error(initial) << ','
          << local.error(solution) << ',' << (first_pose.transformFrom(lever) - reference_seed.translation()).norm() << ','
          << tilt * 180 / M_PI << '\n';
    }
  }
  std::ofstream families(output / "late_family_counterfactual.csv");
  families << std::setprecision(17) << "removed_family,factors_changed,cost_before,cost_after,first_rtk_m,first_tilt_deg\n";
  // Same observed platform boundary for all families; this is NEVER an
  // online time rule. Test which later factor family actually moves history.
  for (const std::string family : {"angular_motion", "all_motion", "geometry"})
  {
    gtsam::NonlinearFactorGraph local;
    std::size_t changed = 0;
    for (const auto &f : graph)
    {
      const auto last = gtsam::Symbol(f->keys().back()).index();
      const bool late = keys[last][0] >= 1785910280.2007458;
      const auto motion = std::dynamic_pointer_cast<NativeMotionFactor>(f);
      const bool geometry = dynamic_cast<ScanPlaneBatchFactor *>(f.get());
      if (late && motion && family == "angular_motion")
      { local.push_back(std::make_shared<MotionRowsAuditFactor>(*motion, 0)); ++changed; }
      else if (late && ((motion && family == "all_motion") || (geometry && family == "geometry"))) ++changed;
      else local.push_back(f);
    }
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = 100; params.relativeErrorTol = 1e-9;
    params.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
    const auto solution = gtsam::LevenbergMarquardtOptimizer(local, values, params).optimize();
    const auto first = solution.at<Pose3>(gtsam::Symbol('x', 0));
    families << family << ',' << changed << ',' << local.error(values) << ',' << local.error(solution) << ','
        << (first.transformFrom(lever) - reference_seed.translation()).norm() << ','
        << std::acos(std::clamp((first.rotation() * raw[0].rotation().inverse()).matrix()(2, 2), -1., 1.)) * 180 / M_PI << '\n';
  }
}

void require(double error, double tolerance, const char *name)
{
  std::cout << name << ": " << error << " (limit " << tolerance << ")\n";
  if (!std::isfinite(error) || error > tolerance) throw std::runtime_error(name);
}

void initialGravityObservation()
{
  ImuProcess imu;
  imu.set_imu_init_frame_num(200);
  imu.cov_acc = V3D(.1, .2, .3);
  const V3D acceleration(.12, -.18, 9.90);
  StatesGroup state;
  const auto original_covariance = state.cov;
  LidarMeasureGroup input;
  input.lio_vio_flg = LIO;
  MeasureGroup measurement;
  measurement.lio_time = 2;
  for (int i = 0; i < 200; ++i)
  {
    auto sample = std::make_shared<sensor_msgs::Imu>();
    sample->header.stamp = stampFromSec((i + 1) * .01);
    sample->linear_acceleration.x = acceleration.x();
    sample->linear_acceleration.y = acceleration.y();
    sample->linear_acceleration.z = acceleration.z();
    measurement.imu.push_back(sample);
  }
  input.measures.push_back(measurement);
  imu.Process2(input, state, std::make_shared<PointCloudXYZI>());
  require(imu.imu_need_init ? 1 : 0, 0, "stationary gravity covariance uses completed initialization");
  const V3D axis = acceleration.normalized();
  const M3D tangent = M3D::Identity() - axis * axis.transpose();
  const M3D noise = (imu.cov_acc * (G_m_s2 * G_m_s2 / acceleration.squaredNorm() / 200)).asDiagonal();
  require((imu.initial_gravity_covariance - tangent *
      (noise + original_covariance.block<3, 3>(13, 13)) * tangent).norm(), 1e-12,
      "initial gravity: average independent noise but not common bias uncertainty");
  require((state.cov.block<3, 3>(16, 16) - original_covariance.block<3, 3>(16, 16)).norm(), 0,
      "gravity observation export leaves original frontend covariance unchanged");
  const V3D gravity = state.gravity;
  const M3D covariance = gravityOrientationCovariance(gravity, imu.initial_gravity_covariance, .0025);
  M3D jacobian;
  constexpr double epsilon = 1e-5;
  for (int i = 0; i < 3; ++i)
  {
    V3D delta = V3D::Zero(); delta(i) = epsilon;
    const auto plus = gtsam::Rot3(Eigen::Quaterniond::FromTwoVectors(gravity + delta, gravity));
    const auto minus = gtsam::Rot3(Eigen::Quaterniond::FromTwoVectors(gravity - delta, gravity));
    jacobian.col(i) = (gtsam::Rot3::Logmap(plus) - gtsam::Rot3::Logmap(minus)) / (2 * epsilon);
  }
  require((covariance - jacobian * imu.initial_gravity_covariance * jacobian.transpose() -
      .0025 * axis * axis.transpose()).norm(), 1e-10,
      "initial gravity covariance matches direction-to-rotation numerical Jacobian");
  require(std::abs(axis.dot(covariance * axis) - .0025), 1e-12,
      "stationary gravity adds no yaw information beyond the existing finite gauge");
  const M3D rotation = gtsam::Rot3::RzRyRx(.7, -.3, 1.4).matrix();
  require((gravityOrientationCovariance(rotation * gravity,
      rotation * imu.initial_gravity_covariance * rotation.transpose(), .0025) - rotation * covariance * rotation.transpose()).norm(),
      1e-12, "initial gravity uncertainty is coordinate-frame equivariant");
  require(Eigen::SelfAdjointEigenSolver<M3D>(covariance).eigenvalues().minCoeff() > 0 ? 0 : 1, 0,
      "initial tilt and yaw remain finite, never a frozen orientation");
}

void qualityAndRecovery()
{
  auto node = std::make_shared<rclcpp::Node>("fusion_quality_validation");
  const V3D lever(0.904, -4.679, -1.699);
  RtkFusion fusion(node, M3D::Identity(), V3D::Zero(), lever,
                   std::string(ROOT_DIR) + "results/validation");
  fusion.local_frontend = true;
  fusion.frontend_alignment_known = false;
  fusion.graph_in_enu = false;
  fusion.initializeGravity(1000.7, V3D(0, 0, -G_m_s2), M3D::Identity() * .0005);
  const gtsam::Pose3 alignment(gtsam::Rot3::Rz(0.3), V3D(20, -30, 4));
  auto add = [&](double stamp, const std::string &mode, double variance = 0, int satellites = 34) {
    diagnostic_msgs::msg::DiagnosticArray status;
    status.header.stamp = stampFromSec(stamp);
    status.status.resize(1);
    diagnostic_msgs::msg::KeyValue field;
    field.key = "ins_pos_mode"; field.value = mode; status.status[0].values.push_back(field);
    field.key = "gps_num_sats"; field.value = std::to_string(satellites); status.status[0].values.push_back(field);
    fusion.observeStatus(status);
    nav_msgs::Odometry odometry;
    odometry.header.stamp = status.header.stamp;
    const auto q = alignment.rotation().toQuaternion();
    odometry.pose.pose.orientation.x = q.x(); odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z(); odometry.pose.pose.orientation.w = q.w();
    const V3D p = alignment.transformFrom(V3D(V3D(stamp - 1000, 0, 0) + lever)) +
        (mode == "4" ? V3D::Zero().eval() : V3D(100, -100, 40));
    odometry.pose.pose.position.x = p.x(); odometry.pose.pose.position.y = p.y(); odometry.pose.pose.position.z = p.z();
    odometry.twist.twist.linear.x = mode == "4" ? 1 : 100;
    for (int axis = 0; axis < 3; ++axis) odometry.pose.covariance[7 * axis] = variance;
    fusion.observe(odometry);
  };
  auto check = [&](double stamp, bool accepted, int reason, const char *label) {
    RtkFusion::Observation observation;
    int actual = -1;
    const bool valid = fusion.referenceAt(stamp, observation, &actual);
    require(valid == accepted && actual == reason ? 0 : 1, 0, label);
  };
  for (int i = 0; i <= 600; ++i)
  {
    const std::string mode = i < 100 || (i >= 300 && i < 400) ? "1" :
        (i >= 410 && i < 420 ? "5" : "4");
    add(1000 + i * 0.01, mode);
  }
  check(1000.8, false, 5, "non-fixed startup rejects 100 m position and 100 m/s velocity corruption");
  check(1001.2, false, 7, "first fixed solution waits for stability");
  check(1001.7, true, 0, "stable fixed accepted");
  check(1003.5, false, 5, "loss switches off external observations immediately");
  check(1004.05, false, 7, "short fixed flicker does not recover");
  check(1004.15, false, 5, "non-4 mode during recovery stays rejected");
  check(1004.6, false, 7, "recovery timer restarts after flicker");
  check(1004.8, true, 0, "continuous stable recovery accepted");
  check(1002.995, false, 5, "interpolation never crosses loss boundary");

  // Directly exercise the production graph with exact native motion and a
  // delayed ENU anchor. No queue jobs are submitted; only this thread owns it.
  const auto cloud = std::make_shared<PointCloudXYZI>();
  for (double stamp : {1000.8, 1001.2, 1001.7, 1002.2, 1003.5, 1004.05, 1004.8, 1005.3})
  {
    RtkFusion::Frame frame{stamp, stamp - 0.1,
        gtsam::Pose3(gtsam::Rot3(), V3D(stamp - 1000, 0, 0)), cloud};
    fusion.optimizeKeyframe(frame);
    fusion.frames.push_back(frame);
  }
  require(fusion.accepted_positions == 4 ? 0 : 1, 0, "exact fixed-only graph factor coverage through outage");
  require(fusion.keyframes.size() == 8 && fusion.graph_in_enu ? 0 : 1, 0, "history survives delayed alignment and reacquisition");
  for (const auto &key : fusion.keyframes)
    require(gtsam::Pose3::Logmap((alignment * key.raw).between(key.optimized)).norm(),
            1e-7, "delayed ENU alignment preserves every historical relative pose");
  StatesGroup state;
  state.pos_end = V3D(5.3, 0, 0);
  RtkFusion::Observation local;
  require(fusion.frontendReferenceAt(1005.3, state, local) ? 0 : 1, 0, "frontend uses shared quality gate");
  require((local.position - state.pos_end - lever).norm(), 1e-10, "ENU position transformed into unchanged frontend map");
  require(fusion.estimate.size() == fusion.keyframes.size() ? 0 : 1, 0,
          "global graph contains only pose states, no duplicate IMU velocity or bias states");
  const auto one = fusion.motionSigmas(1.0);
  const auto half = fusion.motionSigmas(0.5);
  gtsam::Vector6 drift;
  drift << .001, -.002, .003, .01, -.02, .03;
  require(std::abs(drift.cwiseQuotient(one).squaredNorm() -
      2 * (0.5 * drift).cwiseQuotient(half).squaredNorm()), 1e-9,
      "motion Gaussian-core increment weighting scales with elapsed time");
  // Sparse endpoints have no roll excursion. A non-key native scan does:
  // correction interpolation must preserve that measured high-rate motion.
  auto frame = fusion.frames.back();
  frame.stamp = 1002.0;
  frame.begin = 1001.9;
  frame.raw = gtsam::Pose3(gtsam::Rot3::RzRyRx(.03, -.02, .01), V3D(2, .1, .2));
  const auto expected_pose = alignment * frame.raw;
  require(gtsam::Pose3::Logmap(expected_pose.between(fusion.poseAt(frame))).norm(),
          1e-7, "constant global correction preserves all non-key native roll pitch steering and translation");
  auto native_scan = std::make_shared<PointCloudXYZI>();
  const std::array<V3D, 3> fixed_world{V3D(5, 4, 1), V3D(50, -10, 2), V3D(-5, 20, 3)};
  for (std::size_t i = 0; i < fixed_world.size(); ++i)
  {
    const auto local_point = frame.raw.transformTo(fixed_world[i]);
    PointType point;
    point.x = local_point.x(); point.y = local_point.y(); point.z = local_point.z();
    point.curvature = 50 * i; point.intensity = i;
    native_scan->push_back(point);
  }
  frame.cloud = native_scan;
  pcl::PointCloud<pcl::PointXYZI> projected;
  fusion.appendCloud(frame, projected);
  for (std::size_t i = 0; i < projected.size(); ++i)
    require((V3D(projected[i].x, projected[i].y, projected[i].z) -
        alignment.transformFrom(fixed_world[i])).norm(), 8e-6,
        "native deskew receives one rigid scan-end transform independent of point timestamp");
  for (double time : {999.0, 1000.8, 1001.7, 1002.0, 1005.3, 1007.0})
    require(gtsam::Pose3::Logmap(alignment.between(fusion.correctionAt(time))).norm(), 1e-7,
        "constant correction preserves history and tail without extrapolating drift");

  // A factor's local geometry must belong to its own scan-end pose.
  // Borrowing another scan invents rigidity when their C(t) values differ.
  RtkFusion::Frame support{1002.0, 1001.9, gtsam::Pose3(), std::make_shared<PointCloudXYZI>()};
  RtkFusion::Frame previous_support{1001.9, 1001.8,
      gtsam::Pose3(gtsam::Rot3::Rz(.05), V3D(.2, .1, .1)), std::make_shared<PointCloudXYZI>()};
  const auto make_support = [](const gtsam::Pose3 &pose, double offset) {
    auto cloud = std::make_shared<PointCloudXYZI>();
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j)
    {
      const V3D p = pose.transformTo(V3D(offset + .4 * i, offset + .4 * j, .5));
      PointType point; point.x = p.x(); point.y = p.y(); point.z = p.z();
      cloud->push_back(point);
    }
    return cloud;
  };
  support.cloud = make_support(support.raw, .2);
  previous_support.cloud = make_support(previous_support.raw, 1.0);
  require(fusion.extractPlanes(support).size(), 0, "one sparse scan cannot manufacture a supported plane");
  fusion.recent_frames.push_back(previous_support);
  require(fusion.extractPlanes(support).size(), 0,
      "other scans cannot manufacture a rigid surface owned by this pose");
  auto complete = make_support(support.raw, .2);
  *complete += *make_support(support.raw, 1.0);
  support.cloud = complete;
  const auto supported = fusion.extractPlanes(support);
  require(supported.size() == 1 ? 0 : 1, 0, "a surface supported by the actual single scan is retained");
  require(std::abs(supported.front().center.z() - .5) +
      supported.front().normal.cross(V3D::UnitZ()).norm(), 1e-6,
      "single-scan surface coordinates equal the coordinates used by actual export");
  fusion.recent_frames.clear();

  auto shaft = std::make_shared<PointCloudXYZI>();
  for (int ring = 0; ring < 8; ++ring) for (int side = 0; side < 4; ++side)
  {
    PointType point;
    point.x = .5 + .08 * std::cos(side * M_PI / 2);
    point.y = .5 + .08 * std::sin(side * M_PI / 2);
    point.z = .2 + .2 * ring;
    shaft->push_back(point);
  }
  support.cloud = shaft;
  const auto linear = fusion.extractPlanes(support);
  require(linear.size() == 1 && linear.front().linear ? 0 : 1, 0,
      "supported slender patch is a candidate requiring independent cross-scan validation");
  require(linear.front().projector().col(2).norm(), 1e-6,
      "a vertical line candidate contains no invented axial information");
  auto stripes = std::make_shared<PointCloudXYZI>();
  for (int stripe = 0; stripe < 5; ++stripe) for (int sample = 0; sample < 40; ++sample)
  {
    PointType point;
    point.x = .2 + .1 * sample; point.y = .3 + 2 * stripe; point.z = .5;
    stripes->push_back(point);
  }
  support.cloud = stripes;
  require(fusion.extractPlanes(support).size(), 0,
      "separate LiDAR stripes on one measured broad plane are not physical line landmarks");

  // Exercise production initialization with a 25 m failed native increment.
  // The same factors select the lower-cost seed; no position clamp/new prior.
  RtkFusion::Keyframe failed;
  failed.stamp = 1005.8;
  failed.raw = gtsam::Pose3(gtsam::Rot3(), V3D(30.8, 0, 0));
  failed.optimized = alignment * failed.raw;
  failed.reference = alignment.transformFrom(V3D(V3D(5.8, 0, 0) + lever));
  failed.has_position = true;
  const gtsam::Symbol next('x', fusion.keyframes.size()), previous('x', fusion.keyframes.size() - 1);
  gtsam::NonlinearFactorGraph conditional;
  conditional.emplace_shared<NativeMotionFactor>(previous, next,
      fusion.keyframes.back().raw.between(failed.raw), fusion.motionSigmas(.5), std::sqrt(.5));
  conditional.emplace_shared<gtsam::GPSFactorArm>(next, failed.reference, lever,
      gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(2.8),
          gtsam::noiseModel::Diagonal::Sigmas(V3D(.03, .03, .05))));
  gtsam::Values guess; guess.insert(next, failed.optimized);
  auto full = fusion.estimate; full.insert(next, failed.optimized);
  const double before_initialization = conditional.error(full);
  fusion.keyframes.push_back(failed);
  fusion.initializePose(conditional, guess);
  full.update(next, guess.at<gtsam::Pose3>(next));
  require(conditional.error(full) < before_initialization ? 0 : 1, 0,
          "failed-native initialization lowers the exact existing motion/RTK objective");
  require((fusion.keyframes.back().optimized.transformFrom(lever) - failed.reference).norm(), .06,
          "valid fixed RTK initializes a failed native increment inside geometric capture range");
  fusion.keyframes.pop_back();

  auto &anchor = fusion.keyframes[0];
  auto &current = fusion.keyframes[1];
  const auto relative = anchor.optimized.between(current.optimized);
  const auto index_surfaces = [&](RtkFusion::Keyframe &key) {
    key.cells.clear();
    for (std::size_t index = 0; index < key.planes.size(); ++index)
    {
      const V3D &p = key.planes[index].center;
      key.cells[{static_cast<std::int64_t>(std::floor(p.x() / fusion.plane_voxel)),
                 static_cast<std::int64_t>(std::floor(p.y() / fusion.plane_voxel)),
                 static_cast<std::int64_t>(std::floor(p.z() / fusion.plane_voxel))}].push_back(index);
    }
  };
  const auto surface = [](const V3D &center, const V3D &normal) {
    RtkFusion::Plane plane{center, normal, 0};
    plane.tangent = normal.unitOrthogonal();
    for (int u = -2; u <= 2; ++u) for (int v = -2; v <= 2; ++v)
    {
      plane.support.push_back(center + .15 * (u * plane.tangent + v * normal.cross(plane.tangent)));
      plane.extendSupport(plane.support.back());
    }
    plane.sqrt_covariance.col(0) = std::sqrt(.045) * plane.tangent;
    plane.sqrt_covariance.col(1) = std::sqrt(.045) * normal.cross(plane.tangent);
    return plane;
  };
  for (int axis = 0; axis < 3; ++axis)
    for (int i = 0; i < (axis == 2 ? 60 : 10); ++i)
    {
      const V3D center(2 * i, 10 * axis, 0), normal = V3D::Unit(axis);
      anchor.planes.push_back(surface(center, normal));
      const auto other = surface(relative.transformTo(center), relative.rotation().unrotate(normal));
      current.planes.push_back(other);
      current.planes.push_back(other); // duplicates must not buy more weight
    }
  fusion.max_pair_factors = 24;
  index_surfaces(anchor); index_surfaces(current);
  gtsam::NonlinearFactorGraph graph;
  require(fusion.addGeometry(0, 1, graph, RtkFusion::GeometryKind::Local) == 24 ? 0 : 1, 0,
          "surface association keeps a bounded geometry budget");
  const auto factor = std::dynamic_pointer_cast<ScanPlaneBatchFactor>(graph.at(0));
  std::array<int, 3> counts{};
  for (const auto &constraint : factor->constraints())
  {
    Eigen::Index axis;
    constraint.normal.cwiseAbs().maxCoeff(&axis);
    ++counts[axis];
  }
  require(counts == std::array<int, 3>{8, 8, 8} ? 0 : 1, 0,
          "scarce vertical surfaces survive dominant road returns and duplicate matches");
  for (const auto &row : factor->constraints())
    require(row.project(relative.transformFrom(row.point) - row.center).norm(), 1e-10,
        "exact common-support centroids retain zero displacement");
  // Covariance subtraction at 100 m can leave O(epsilon * range^2)
  // variance. Check ENERGY, not its square root with a linear-error tolerance.
  require(factor->evaluateError(anchor.optimized, current.optimized).squaredNorm(), 1e-9,
          "exact planar surface energy stays within covariance roundoff");
  anchor.planes.clear(); current.planes.clear();
  for (int i = 0; i < 24; ++i)
  {
    const V3D center(4 * i, 0, 0);
    anchor.planes.push_back(surface(center, V3D::UnitZ()));
    current.planes.push_back(surface(relative.transformTo(V3D(center + V3D(.8, 0, 0))),
        relative.rotation().unrotate(V3D(.1, 0, 1).normalized())));
  }
  graph.resize(0);
  index_surfaces(anchor); index_surfaces(current);
  require(fusion.addGeometry(0, 1, graph, RtkFusion::GeometryKind::Local), 0,
          "nearby disjoint surface footprints cannot create extrapolated constraints");
  for (std::size_t i = 0; i < current.planes.size(); ++i)
  {
    current.planes[i] = surface(relative.transformTo(V3D(anchor.planes[i].center + V3D(0, 0, .03))),
        relative.rotation().unrotate(V3D((i % 2 ? -1. : 1.) * V3D::UnitZ())));
  }
  index_surfaces(current);
  require(fusion.addGeometry(0, 1, graph, RtkFusion::GeometryKind::Local) == 24 ? 0 : 1, 0,
          "mutually supported planes retain real translation error");
  // Most patches share a correctable 30 cm displacement. Four parallel but
  // different surfaces cannot be made consistent by that same rigid pose.
  current.planes.clear();
  for (std::size_t i = 0; i < anchor.planes.size(); ++i)
    current.planes.push_back(surface(relative.transformTo(V3D(anchor.planes[i].center +
        V3D(0, 0, i % 6 == 0 ? .9 : .3))), relative.rotation().unrotate(V3D::UnitZ())));
  index_surfaces(current);
  graph.resize(0);
  const auto consistent_surfaces = fusion.addGeometry(0, 1, graph, RtkFusion::GeometryKind::Local);
  fusion.association_log.flush();
  std::cout << "Common-support retained surface count: " << consistent_surfaces << std::endl;
  require(consistent_surfaces == 20 ? 0 : 1, 0,
          "common registration separates parallel wrong surfaces from coherent 30 cm drift");
  const auto verified = std::dynamic_pointer_cast<ScanPlaneBatchFactor>(graph.at(0));
  for (const auto &row : verified->constraints())
    require(std::abs(std::abs(row.normal.dot(relative.transformFrom(row.point) - row.center)) - .3),
            1e-10, "association validation retains the original physical residual, not a fitted zero");
  anchor.planes.clear(); current.planes.clear();
  anchor.cells.clear(); current.cells.clear();
  fusion.max_pair_factors = 120;
  const auto previous_alignment = fusion.frontend_to_enu;
  require(!fusion.frontendReferenceAt(1003.5, state, local) ? 0 : 1, 0, "bad reference cannot aid frontend");
  require(gtsam::Pose3::Logmap(previous_alignment.between(fusion.frontend_to_enu)).norm(), 0,
          "outage cannot overwrite the frontend map alignment");

  const auto accepted_before_dense_scans = fusion.accepted_positions;
  double information_before = 0;
  for (const auto &key : fusion.keyframes) information_before += key.position_information_weight;
  const auto anchors_before_dense_scans = fusion.geometry_anchors.size();
  for (int i = 1; i <= 5; ++i)
  {
    const double stamp = 1005.3 + .1 * i;
    RtkFusion::Frame dense{stamp, stamp - .1,
        gtsam::Pose3(gtsam::Rot3(), V3D(stamp - 1000, 0, 0)), cloud};
    fusion.optimizeKeyframe(dense);
    fusion.frames.push_back(dense);
  }
  require(fusion.accepted_positions == accepted_before_dense_scans + 5 ? 0 : 1, 0,
          "valid observations cover every key including short intervals");
  double information_after = 0;
  for (const auto &key : fusion.keyframes) information_after += key.position_information_weight;
  require(std::abs(information_after - information_before - 1), 1e-9,
          "10 Hz keys across 0.5 seconds contribute one nominal RTK observation, not five");
  require(fusion.geometry_anchors.size() == anchors_before_dense_scans + 1 ? 0 : 1, 0,
          "dense local scan states retain independently spaced long-baseline anchors");

  add(1006.01, "4", 4.0);
  check(1006.01, false, 8, "mode 4 with metre-scale covariance rejected");
  add(1006.02, "4", -1.0);
  check(1006.02, false, 8, "negative covariance is not treated as unknown precision");
  add(1006.03, "4oops");
  check(1006.03, false, 5, "malformed quality invalidates previous fixed status");
  add(1006.04, "4", 0, 5);
  check(1006.04, false, 6, "insufficient satellites rejected");
  // Observation without a new quality packet must not keep the old fix alive.
  nav_msgs::Odometry stale;
  stale.header.stamp = stampFromSec(1006.3);
  stale.pose.pose.orientation.w = 1;
  fusion.observe(stale);
  check(1006.3, false, 4, "stale quality fails closed");
  fusion.covariance_scale = 4;
  fusion.reported_model_sigma_xy = .01;
  fusion.reported_model_sigma_z = .02;
  add(1006.4, "4", .0004);
  require((fusion.observations.back().covariance -
      V3D(.0017, .0017, .002).asDiagonal().toDenseMatrix()).norm(), 1e-14,
      "reported covariance uses receiver scale plus explicit model, not fallback");
  require(fusion.observations.back().covariance_source == 2 ? 0 : 1, 0,
      "reported covariance source remains traceable");
  add(1006.41, "4", 0);
  require((fusion.observations.back().covariance -
      V3D(.0009, .0009, .0025).asDiagonal().toDenseMatrix()).norm(), 1e-14,
      "unknown covariance uses total fallback without duplicate scale or model");
  require(fusion.observations.back().covariance_source == 1 ? 0 : 1, 0,
      "unknown covariance is not described as receiver precision");
  fusion.reported_model_sigma_xy = fusion.reported_model_sigma_z = 0;
  nav_msgs::Odometry singular;
  singular.header.stamp = stampFromSec(1006.42);
  singular.pose.covariance[0] = .001;
  fusion.observe(singular);
  require(!fusion.observations.back().position_valid ? 0 : 1, 0,
      "partly zero singular reported covariance cannot masquerade as perfect axes");
  fusion.finish();
}

void pointCovarianceFrames()
{
  VoxelMapManager map;
  map.state_.rot_end = gtsam::Rot3::RzRyRx(.2, -.1, 1.3).matrix();
  map.state_.pos_end = V3D(12, -270, 45);
  map.extR_ = gtsam::Rot3::RzRyRx(-.12, .08, -.4).matrix();
  map.extT_ = V3D(.904, -4.679, -1.699);
  Eigen::Matrix<double, 6, 6> root = Eigen::Matrix<double, 6, 6>::Zero();
  root.diagonal() << .002, .005, .003, .03, .05, .02;
  root(4, 0) = .025; root(5, 1) = -.015; root(3, 2) = .01;
  const Eigen::Matrix<double, 6, 6> covariance = root * root.transpose();
  map.state_.cov.topLeftCorner<6, 6>() = covariance;
  const V3D point(14, 38, -2);
  const V3D body = map.extR_ * point + map.extT_;
  const M3D measurement = V3D(.0004, .0016, .0025).asDiagonal();
  const auto cross = gtsam::skewSymmetric(body);
  const M3D actual = map.pointCovarianceWorld(cross, measurement);
  Eigen::Matrix<double, 3, 6> jacobian;
  constexpr double step = 1e-6;
  for (int axis = 0; axis < 6; ++axis)
  {
    gtsam::Vector6 delta = gtsam::Vector6::Zero(); delta(axis) = step;
    const V3D plus = map.state_.rot_end * gtsam::Rot3::Expmap(delta.head<3>()).rotate(body) +
        map.state_.pos_end + delta.tail<3>();
    const V3D minus = map.state_.rot_end * gtsam::Rot3::Expmap(-delta.head<3>()).rotate(body) +
        map.state_.pos_end - delta.tail<3>();
    jacobian.col(axis) = (plus - minus) / (2 * step);
  }
  const M3D rotation = map.state_.rot_end * map.extR_;
  require((actual - rotation * measurement * rotation.transpose() - jacobian * covariance * jacobian.transpose()).norm(),
          2e-9, "physical-IMU point covariance matches the full pose Jacobian including cross terms");
  const M3D world_change = gtsam::Rot3::RzRyRx(-.3, .4, -.9).matrix();
  const auto old_state = map.state_;
  Eigen::Matrix<double, 6, 6> basis = Eigen::Matrix<double, 6, 6>::Identity();
  basis.bottomRightCorner<3, 3>() = world_change;
  map.state_.rot_end = world_change * old_state.rot_end;
  map.state_.cov.topLeftCorner<6, 6>() = basis * covariance * basis.transpose();
  require((map.pointCovarianceWorld(cross, measurement) - world_change * actual * world_change.transpose()).norm(),
          1e-12, "world-frame change rotates point uncertainty without changing physical confidence");
  map.state_ = old_state;
  const V3D shift(-.904, 4.679, 1.699);
  basis.setIdentity();
  basis.bottomLeftCorner<3, 3>() = -old_state.rot_end * gtsam::skewSymmetric(shift);
  map.state_.cov.topLeftCorner<6, 6>() = basis * covariance * basis.transpose();
  map.state_.pos_end += old_state.rot_end * shift;
  map.extT_ -= shift;
  require((map.pointCovarianceWorld(gtsam::skewSymmetric(body - shift), measurement) - actual).norm(),
          1e-12, "changing the body reference point preserves point uncertainty with its lever and cross covariance");
  require(std::max(0., -Eigen::SelfAdjointEigenSolver<M3D>(actual).eigenvalues().minCoeff()),
          1e-14, "joint pose and measurement covariance remains positive semidefinite");
}

}

int main(int argc, char **argv)
{
  if ((argc == 5 || argc == 6) && std::string(argv[1]) == "--audit-saved-motion")
  {
    auditSavedMotion(argv[2], argv[3], argv[4], argc == 6 ? argv[5] : "");
    return 0;
  }
  using namespace gtsam;
  pointCovarianceFrames();
  const auto result_root = std::filesystem::path(ROOT_DIR) / "results";
  require(projectResultDirectory(result_root / "validation") == result_root / "validation" ? 0 : 1,
          0, "validation output stays under the canonical results root");
  for (const auto &invalid : {std::filesystem::path(ROOT_DIR) / "result/validation",
       result_root / "../Log", std::filesystem::path(ROOT_DIR) / "results_elsewhere",
       std::filesystem::path("/tmp/my_livo_validation")})
  {
    bool rejected = false;
    try { projectResultDirectory(invalid); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected ? 0 : 1, 0, "reject out-of-results paths before any file creation");
  }
  for (double angle : {0.0, 1e-12, 1e-7, 1e-5, 1e-4, 0.1, 3.0})
  {
    const V3D increment = angle * V3D(0.2, -0.3, 0.8).normalized();
    const M3D expected = Rot3::Expmap(increment).matrix();
    require((Exp(increment.x(), increment.y(), increment.z()) - expected).norm(),
            1e-12, "SO3 update retains small-angle corrections");
    require((Exp(increment, 1.0) - expected).norm(), 1e-12, "SO3 propagation matches state retraction");
  }
  const Pose3 a(Rot3::RzRyRx(0.12, -0.08, 1.2), Point3(-65, -280, 45));
  const Pose3 b(Rot3::RzRyRx(0.13, -0.07, 1.21), Point3(-64, -278, 44.9));
  Vector6 motion_sigmas, motion_error;
  motion_sigmas << .0005, .0005, .0007, .03, .03, .02;
  {
    gtsam::Matrix6 information = gtsam::Matrix6::Zero();
    information.diagonal() << 90, 60, 0, 5, 5, 5;
    const auto observable = nativeAngularObservability(information);
    require(observable(2, 2), 1e-14, "unobserved native yaw acquires no fabricated curvature");
    require((observable - nativeAngularObservability(100 * information)).norm(), 1e-12,
        "native angular strength is independent of duplicated point count");
    information(0, 3) = information(3, 0) = std::sqrt(90 * 5.);
    require(nativeAngularObservability(information)(0, 0), 1e-12,
        "rotation explained by translation is not independent angular evidence");
  }
  for (double bad_yaw : {0.0, .001, .25})
  {
    motion_error << .00001, -.00002, bad_yaw, .003, -.015, .2;
    const NativeMotionFactor factor(0, 1, a.between(b) * Pose3::Expmap(motion_error), motion_sigmas, 3);
    gtsam::Matrix ha, hb;
    factor.evaluateError(a, b, &ha, &hb);
    const std::function<gtsam::Vector(const Pose3 &, const Pose3 &)> residual =
        [&](const Pose3 &x, const Pose3 &y) { return factor.evaluateError(x, y); };
    require((ha - numericalDerivative21<gtsam::Vector, Pose3, Pose3>(residual, a, b, 1e-7)).norm(), 5e-4,
        "native-motion anchor Jacobian retains physical rotation and endpoint displacement");
    require((hb - numericalDerivative22<gtsam::Vector, Pose3, Pose3>(residual, a, b, 1e-7)).norm(), 5e-4,
        "component robust native-motion current Jacobian through large yaw error");
    const auto expected_difference = Pose3::Expmap(motion_error).inverse();
    Vector6 expected_physical;
    expected_physical << Rot3::Logmap(expected_difference.rotation()), expected_difference.translation();
    require((factor.physicalError(a, b) - expected_physical).norm(), 1e-10,
        "motion diagnostics retain actual endpoint displacement, not screw coordinates");
    require(hb.bottomLeftCorner<3, 3>().norm(), 1e-12,
        "native position error never invents an end-pose attitude observation");
    // A yaw outlier never scales the entire residual as a block: the other
    // physical components retain their own likelihood and derivative.
    const gtsam::Vector actual = factor.evaluateError(a, b);
    for (int i : {0, 1, 3, 4, 5})
    {
      const double value = expected_physical(i) / motion_sigmas(i);
      const double magnitude = std::abs(value);
      require(std::abs(actual(i) * actual(i) - (magnitude <= 3 ? value * value : 6 * magnitude - 9)), 1e-8,
          "yaw conflict does not downweight unrelated native-motion components");
    }
  }
  for (double drift_rate : {.01, .10, 5.0})
  {
    const auto cost = [&](double dt) {
      const NativeMotionFactor factor(0, 1, Pose3(), motion_sigmas * std::sqrt(dt), std::sqrt(dt));
      const auto residual = factor.evaluateError(Pose3(), Pose3(Rot3(), Point3(0, drift_rate * dt, 0)));
      return .5 * residual.squaredNorm();
    };
    require(std::abs(cost(1) - cost(.3) - cost(.7)), 1e-10,
            "native robust correction-rate energy invariant to uneven interval splitting");
  }
  const gtsam::Vector3 normal = gtsam::Vector3(0.2, -0.3, 0.8).normalized();
  {
    const M3D target_extent = gtsam::Vector3(.4, .7, .02).asDiagonal();
    const M3D source_extent = Rot3::RzRyRx(.03, -.01, .1).matrix() * target_extent;
    const ScanPlaneBatchFactor::Constraint observed{V3D::UnitZ(), V3D(10, 3, 1), V3D(9.9, 3.1, 1.05),
        .07, false, source_extent, target_extent};
    const ScanPlaneBatchFactor shared(0, 1, {observed});
    gtsam::Matrix ja, jb;
    shared.evaluateError(a, b, &ja, &jb);
    const std::function<gtsam::Vector(const Pose3 &, const Pose3 &)> residual =
        [&](const Pose3 &x, const Pose3 &y) { return shared.evaluateError(x, y); };
    require((ja - numericalDerivative21<gtsam::Vector, Pose3, Pose3>(residual, a, b, 1e-6)).norm(), 1e-5,
        "shared-plane anchor Jacobian includes the estimated normal derivative");
    require((jb - numericalDerivative22<gtsam::Vector, Pose3, Pose3>(residual, a, b, 1e-6)).norm(), 1e-5,
        "shared-plane source Jacobian includes the estimated normal derivative");
    const ScanPlaneBatchFactor::Constraint swapped{V3D::UnitZ(), observed.point, observed.center,
        .07, false, target_extent, source_extent};
    require(std::abs(observed.physicalResidual(a.between(b)).squaredNorm() -
        swapped.physicalResidual(b.between(a)).squaredNorm()), 1e-10,
        "common physical plane energy is invariant to exchanging the two scans");
    const ScanPlaneBatchFactor::Constraint parallel{V3D::UnitZ(), V3D::Zero(), V3D(0, 0, .03),
        .07, false, V3D(1, 1, 0).asDiagonal(), V3D(1, 1, 0).asDiagonal()};
    require(std::abs(parallel.physicalResidual(Pose3()).norm() - .03), 1e-12,
        "profiling the common plane preserves physical parallel-plane separation scale");
  }
  gtsam::Matrix ha, hb;
  const Pose3 world(Rot3::RzRyRx(-0.3, 0.1, 0.5), Point3(1000, -800, 250));
  const ScanPlaneBatchFactor batch(0, 1, {
      {normal, Point3(20, -5, 0), Point3(20.1, -5, 0.02), 0.05, false,
          Rot3::RzRyRx(.1, .3, .5).matrix() * gtsam::Vector3(.4, .7, .01).asDiagonal()},
      {normal, a.transformTo(b.transformFrom(Point3(20, -5, 0))), Point3(20, -5, 0), 0.05}});
  require((batch.evaluateError(world * a, world * b) - batch.evaluateError(a, b)).norm(),
          1e-10, "scan-plane common-world gauge invariance");
  batch.evaluateError(a, b, &ha, &hb);
  const std::function<gtsam::Vector(const Pose3 &, const Pose3 &)> batch_residual =
      [&](const Pose3 &x, const Pose3 &y) { return batch.evaluateError(x, y); };
  require((ha - numericalDerivative21<gtsam::Vector, Pose3, Pose3>(batch_residual, a, b, 1e-6)).norm(),
          1e-5, "batched Huber plane anchor Jacobian, inlier and outlier");
  require((hb - numericalDerivative22<gtsam::Vector, Pose3, Pose3>(batch_residual, a, b, 1e-6)).norm(),
          1e-5, "batched Huber plane current Jacobian, inlier and outlier");
  const ScanPlaneBatchFactor ground(0, 1, {
      {gtsam::Vector3::UnitZ(), Point3(10, 0, -2), Point3(10, 0, -2), .05, false,
          gtsam::Vector3(.7, .4, 0).asDiagonal()},
      {gtsam::Vector3::UnitZ(), Point3(0, 10, -2), Point3(0, 10, -2), .05}});
  ground.evaluateError(Pose3(), Pose3(), &ha, &hb);
  require(hb.col(2).norm() + hb.col(3).norm() + hb.col(4).norm(), 0,
          "ground surfaces never manufacture yaw or horizontal-position information");
  require(hb.leftCols(2).norm() > 100 ? 0 : 1, 0,
          "spatially separated ground returns observe relative roll and pitch");

  const ScanPlaneBatchFactor pole(0, 1, {
      {gtsam::Vector3::UnitZ(), Point3(20, -5, 0), Point3(20, -5, 2), .05, true,
          gtsam::Vector3(0, 0, 2).asDiagonal()},
      {gtsam::Vector3::UnitZ(), Point3(20, -5, 2), Point3(20, -5, 4), .05, true}});
  pole.evaluateError(Pose3(), Pose3(), &ha, &hb);
  require(pole.evaluateError(Pose3(), Pose3()).norm() + hb.col(5).norm(), 0,
      "point-to-line projection ignores axial centroid separation and axial translation");
  require(hb.col(3).norm() > 10 && hb.col(4).norm() > 10 ? 0 : 1, 0,
      "vertical pole observes both horizontal directions");
  const std::function<gtsam::Vector(const Pose3 &, const Pose3 &)> pole_residual =
      [&](const Pose3 &x, const Pose3 &y) { return pole.evaluateError(x, y); };
  pole.evaluateError(a, b, &ha, &hb);
  require((ha - numericalDerivative21<gtsam::Vector, Pose3, Pose3>(pole_residual, a, b, 1e-6)).norm(),
      1e-5, "vector-Huber line anchor Jacobian through large transverse error");
  require((hb - numericalDerivative22<gtsam::Vector, Pose3, Pose3>(pole_residual, a, b, 1e-6)).norm(),
      1e-5, "vector-Huber line current Jacobian through large transverse error");
  require((pole.evaluateError(world * a, world * b) - pole.evaluateError(a, b)).norm(),
      1e-10, "line residual is invariant to a common change of world frame");

  // Actual returns, not an invented normal observation: moment compression
  // must preserve their mean squared distance and observe centroid-pivot tilt.
  for (bool line : {false, true})
  {
    const std::vector<gtsam::Vector3> points = line ? std::vector<gtsam::Vector3>{
        {0, 0, -3}, {0, 0, -1}, {0, 0, 1}, {0, 0, 3}} : std::vector<gtsam::Vector3>{
        {-2, -1, 0}, {2, -1, 0}, {-2, 1, 0}, {2, 1, 0}};
    gtsam::Matrix3 covariance = gtsam::Matrix3::Zero();
    for (const auto &point : points) covariance += point * point.transpose() / points.size();
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> eigen(covariance);
    ScanPlaneBatchFactor::Constraint surface{gtsam::Vector3::UnitZ(), gtsam::Vector3::Zero(), gtsam::Vector3::Zero(), .05, line,
        eigen.eigenvectors() * eigen.eigenvalues().cwiseMax(0.).cwiseSqrt().asDiagonal()};
    const Pose3 rotated(Rot3::RzRyRx(.1, -.05, 0), gtsam::Vector3::Zero());
    double actual = 0;
    for (const auto &point : points) actual += surface.project(rotated.transformFrom(point)).squaredNorm() / points.size();
    require(std::abs(surface.physicalResidual(rotated).squaredNorm() - actual), 1e-12,
        "surface sufficient statistics equal the actual returns' mean squared geometric distance");
    require(surface.project(rotated.transformFrom(surface.point)).norm(), 0,
        "centroid-only residual misses a surface rotating about its centre");
    require(actual > .001 ? 0 : 1, 0,
        "actual surface extent observes tilt even with coincident centroids");
  }


  for (bool ground_only : {false, true})
  {
    std::vector<ScanPlaneBatchFactor::Constraint> rows;
    for (int i = 0; i < 120; ++i)
      rows.push_back({ground_only ? gtsam::Vector3::UnitZ().eval() : gtsam::Vector3(i % 3 - 1., .3, 1.).normalized(),
          Point3(i % 10, i / 10, -2), Point3(i % 10 + .02, i / 10, -2.01), .05,
          !ground_only && i % 4 == 0});
    const ScanPlaneBatchFactor dense(0, 1, std::move(rows));
    Values values;
    values.insert(0, ground_only ? Pose3() : a);
    values.insert(1, ground_only ? Pose3() : b);
    const auto residual = dense.evaluateError(values.at<Pose3>(0), values.at<Pose3>(1), &ha, &hb);
    gtsam::Matrix augmented(residual.size(), 13);
    augmented << ha, hb, -residual;
    const gtsam::Matrix expected = augmented.transpose() * augmented;
    const auto compressed = std::dynamic_pointer_cast<JacobianFactor>(dense.linearize(values));
    require(compressed->rows() == 7 ? 0 : 1, 0, "120 surface rows compress to six pose directions plus constant energy");
    require((compressed->augmentedInformation() - expected).norm() / expected.norm(), 1e-12,
        ground_only ? "QR compression preserves ground degeneracy and complete quadratic cost" :
                      "QR compression preserves robust Hessian, gradient and constant cost");
    VoxelMapManager geometry;
    geometry.effct_feat_num_ = static_cast<int>(residual.size());
    geometry.lidar_information_body_ = hb.transpose() * hb;
    const auto normalized = geometry.normalizedLidarInformation(30);
    gtsam::Vector6 increment;
    increment << .001, -.002, .003, .02, -.03, .01;
    gtsam::Vector6 metric_increment = increment;
    metric_increment.head<3>() *= 30;
    require(std::abs(metric_increment.dot(normalized * metric_increment) -
        increment.dot(geometry.lidar_information_body_ * increment) / residual.size()), 1e-12,
        "metric normalization preserves per-return quadratic information");
    geometry.effct_feat_num_ *= 2;
    geometry.lidar_information_body_ *= 2;
    require((normalized - geometry.normalizedLidarInformation(30)).norm(), 1e-12,
        "duplicating returns cannot improve normalized geometric observability");
    if (ground_only)
      require(normalized.col(2).norm() + normalized.col(3).norm() + normalized.col(4).norm(), 0,
          "LiDAR-only ground diagnostic preserves yaw and horizontal null directions");
  }

  const Point3 lever(0.904, -4.679, -1.699);
  const GPSFactorArm gps(0, a.transformFrom(lever), lever,
      noiseModel::Isotropic::Sigma(3, 0.03));
  gtsam::Matrix h;
  require(gps.evaluateError(a, &h).norm(), 1e-12, "rear-axle physical lever sign");
  const std::function<gtsam::Vector(const Pose3 &)> gps_residual =
      [&](const Pose3 &x) { return gps.evaluateError(x); };
  require((h - numericalDerivative11<gtsam::Vector, Pose3>(gps_residual, a, 1e-5)).norm(),
          1e-7, "rear-axle position/attitude coupling Jacobian");

  const Pose3 c0(Rot3::RzRyRx(0.01, 0.02, -0.04), Point3(0.1, -0.2, 0.03));
  const Pose3 c1(Rot3::RzRyRx(0.015, 0.023, -0.03), Point3(0.15, -0.1, 0.04));
  require(Pose3::Logmap(c0.between(interpolateTransform(c0, c1, 0))).norm(),
          1e-12, "correction interpolation first endpoint");
  require(Pose3::Logmap(c1.between(interpolateTransform(c0, c1, 1))).norm(),
          1e-12, "correction interpolation last endpoint");
  const Pose3 expected = world * interpolateTransform(c0, c1, 0.37) * world.inverse();
  const Pose3 shifted = interpolateTransform(world * c0 * world.inverse(),
                                             world * c1 * world.inverse(), 0.37);
  require(Pose3::Logmap(expected.between(shifted)).norm(), 1e-9,
          "correction interpolation ENU-origin invariance");
  for (double range : {20.0, 30.0, 60.0})
  {
    const double displacement = (Rot3::Rz(M_PI / 1800).rotate(Point3(range, 0, 0)) -
                                 Point3(range, 0, 0)).norm();
    require(std::abs(displacement - 2 * range * std::sin(M_PI / 3600)),
            1e-12, "far-range angular error budget");
  }
  configureRosLogDirectory();
  rclcpp::init(argc, argv);
  initialGravityObservation();
  qualityAndRecovery();
  rclcpp::shutdown();
}
