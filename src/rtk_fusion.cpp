#include "rtk_fusion.h"
#include "rtk_factors.h"

#include <Eigen/Eigenvalues>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <charconv>
#include <chrono>
#include <future>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace
{

gtsam::ISAM2Params fusionSolverParameters(double rotation = 0.0001, double translation = 0.002,
                                          double misalignment = 0.0001)
{
  gtsam::ISAM2Params params;
  params.relinearizeSkip = 1;
  params.findUnusedFactorSlots = true;
  gtsam::ISAM2ThresholdMap thresholds;
  gtsam::Vector6 threshold;
  threshold << rotation, rotation, rotation, translation, translation, translation;
  thresholds['x'] = threshold;
  // Installation misalignment (Rot3). It is shared by EVERY attitude factor, so each relinearization of it
  // re-eliminates the whole Bayes tree; its threshold is configured separately (rtk/relinearize_misalignment_deg).
  thresholds['m'] = gtsam::Vector3::Constant(misalignment);
  params.relinearizeThreshold = thresholds;
  params.optimizationParams = gtsam::ISAM2DoglegParams();
  params.factorization = gtsam::ISAM2Params::QR;
  return params;
}

// Range-priority cell of a saved-map row (float coordinates, as written).
std::uint64_t priorityCell(float x, float y, float z, double voxel)
{
  const auto cell = [voxel](float v) {
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(static_cast<double>(v) / voxel)) & 0x1FFFFF);
  };
  return (cell(x) << 42) | (cell(y) << 21) | cell(z);
}

// Signed z cell index of a priorityCell key (21-bit two's complement field).
std::int64_t priorityCellZ(std::uint64_t key)
{
  return static_cast<std::int64_t>((key & 0x1FFFFF) ^ 0x100000) - 0x100000;
}
}  // namespace

RtkFusion::RtkFusion(const rclcpp::Node::SharedPtr &node_in,
    const M3D &rotation, const V3D &translation, const V3D &reference_lever,
    const std::string &output_root, double reference_time_shift,
    double position_time_offset_override, double attitude_time_offset_override)
    : node(node_in), lidar_rotation(rotation), lidar_translation(translation),
      lever(reference_lever), solver(fusionSolverParameters())
{
  Ros2ParameterReader parameters(node);
  parameters.param<bool>("rtk/require_status", require_status, true);
  parameters.param<bool>("rtk/save_map", save_map, false);
  parameters.param<bool>("rtk/export_map_chunks", export_map_chunks, true);
  parameters.param<bool>("common/img_en", color_enabled, false);
  bool frontend_position_aiding = false;
  parameters.param<bool>("rtk/frontend_position_aiding", frontend_position_aiding, false);
  if (frontend_position_aiding)
    throw std::runtime_error("Global correction requires unassisted native LIO; frontend_position_aiding must be false");
  parameters.param<double>("rtk/time_offset_sec", time_offset, 0.0);
  parameters.param<double>("rtk/attitude_time_offset_sec", attitude_time_offset, time_offset);
  // The calibration file is the single source of truth for the INS association
  // offsets when it carries them (time_offsets.rtk_position / ins_attitude).
  if (std::isfinite(position_time_offset_override))
  {
    RCLCPP_INFO(node->get_logger(), "INS position association offset %+.4f s from the calibration file (yaml %+.4f s)",
                position_time_offset_override, time_offset);
    time_offset = position_time_offset_override;
  }
  if (std::isfinite(attitude_time_offset_override))
  {
    RCLCPP_INFO(node->get_logger(), "INS attitude association offset %+.4f s from the calibration file (yaml %+.4f s)",
                attitude_time_offset_override, attitude_time_offset);
    attitude_time_offset = attitude_time_offset_override;
  }
  if (!std::isfinite(attitude_time_offset)) throw std::runtime_error("rtk/attitude_time_offset_sec must be finite");
  // Calibration-driven split of the lidar time semantics: frame stamps run on the
  // hardware IMU base (time_offsets.lidar_frontend) while the INS association keeps
  // the axle-calibrated alignment via this shift (= lidar - lidar_frontend).
  if (!std::isfinite(reference_time_shift))
    throw std::runtime_error("reference_time_shift must be finite");
  time_offset += reference_time_shift;
  attitude_time_offset += reference_time_shift;
  parameters.param("rtk/accepted_position_modes", accepted_modes, std::vector<int>{4});
  const std::pair<const char *, double *> doubles[] = {
      {"status_max_age_sec", &status_age}, {"interpolation_max_gap_sec", &interpolation_gap},
      {"horizontal_sigma_m", &horizontal_sigma}, {"vertical_sigma_m", &vertical_sigma},
      {"maximum_horizontal_sigma_m", &max_horizontal_sigma},
      {"maximum_vertical_sigma_m", &max_vertical_sigma},
      {"covariance_scale", &covariance_scale}, {"huber_delta", &huber_delta},
      {"recovery_stable_interval_sec", &recovery_stable_interval},
      {"keyframe_interval_sec", &key_interval}, {"keyframe_min_interval_sec", &key_min_interval},
      {"position_factor_interval_sec", &position_factor_interval},
      {"geometry_anchor_interval_sec", &geometry_anchor_interval},
      {"keyframe_translation_m", &key_translation}, {"keyframe_rotation_deg", &key_rotation},
      {"keyframe_discontinuity_m", &key_discontinuity},
      {"plane_voxel_m", &plane_voxel}, {"plane_max_thickness_m", &plane_thickness},
      {"plane_model_sigma_m", &plane_noise_floor},
      {"motion_huber_rate_per_sqrt_sec", &motion_huber_rate},
      {"revisit_radius_m", &revisit_radius}, {"revisit_min_age_sec", &revisit_age},
      {"match_max_distance_m", &match_distance}, {"publish_interval_sec", &publish_interval}};
  for (const auto &parameter : doubles)
  {
    const std::string name = std::string("rtk/") + parameter.first;
    parameters.param<double>(name, *parameter.second, *parameter.second);
    if (!std::isfinite(*parameter.second) || *parameter.second <= 0)
      throw std::runtime_error(name + " must be finite and positive");
  }
  parameters.param<double>("rtk/reported_model_sigma_xy_m", reported_model_sigma_xy, 0.0);
  parameters.param<double>("rtk/reported_model_sigma_z_m", reported_model_sigma_z, 0.0);
  if (!std::isfinite(reported_model_sigma_xy) || reported_model_sigma_xy < 0 ||
      !std::isfinite(reported_model_sigma_z) || reported_model_sigma_z < 0)
    throw std::runtime_error("Reported-position model sigmas must be finite and nonnegative");
  const std::pair<const char *, int *> integers[] = {
      {"minimum_satellites", &minimum_satellites}, {"max_planes_per_frame", &max_planes},
      {"max_factors_per_pair", &max_pair_factors}, {"local_neighbors", &local_neighbors},
      {"local_map_frames", &local_map_frames}, {"queue_capacity", &queue_capacity},
      {"export_chunk_points", &export_chunk_points}};
  for (const auto &parameter : integers)
  {
    const std::string name = std::string("rtk/") + parameter.first;
    parameters.param<int>(name, *parameter.second, *parameter.second);
    if (*parameter.second <= 0) throw std::runtime_error(name + " must be positive");
  }
  parameters.param<double>("rtk/save_map_max_range", save_map_max_range, 0.0);
  if (!std::isfinite(save_map_max_range) || save_map_max_range < 0.0)
    throw std::runtime_error("rtk/save_map_max_range must be finite and nonnegative (0 disables the cap)");
  parameters.param<double>("rtk/save_rgb_max_range", save_rgb_max_range, -1.0);
  if (!std::isfinite(save_rgb_max_range))
    throw std::runtime_error("rtk/save_rgb_max_range must be finite (<0 = same as save_map_max_range, 0 = no cap)");
  if (save_rgb_max_range < 0.0) save_rgb_max_range = save_map_max_range;
  parameters.param<bool>("rtk/save_map_priority_enabled", save_map_priority_enabled, save_map_priority_enabled);
  parameters.param<double>("rtk/save_map_priority_ratio", save_map_priority_ratio, save_map_priority_ratio);
  parameters.param<double>("rtk/save_map_priority_margin_m", save_map_priority_margin, save_map_priority_margin);
  parameters.param<double>("rtk/save_map_priority_voxel_m", save_map_priority_voxel, save_map_priority_voxel);
  parameters.param<double>("rtk/save_map_priority_free_range_m", save_map_priority_free_range,
                           save_map_priority_free_range);
  if (!std::isfinite(save_map_priority_ratio) || (save_map_priority_ratio != 0.0 && save_map_priority_ratio < 1.0) ||
      !std::isfinite(save_map_priority_margin) || save_map_priority_margin < 0.0 ||
      !std::isfinite(save_map_priority_voxel) || save_map_priority_voxel <= 0.0 ||
      !std::isfinite(save_map_priority_free_range) || save_map_priority_free_range < 0.0)
    throw std::runtime_error(
        "rtk/save_map_priority_*: ratio must be 0 (off) or >= 1, margin >= 0, voxel > 0, free range >= 0");
  RCLCPP_INFO(node->get_logger(),
      "Saved geometry map range priority: %s (ratio %.2f, margin %.1f m, cell %.2f m, seen-free cells from returns "
      "within %.1f m; rtk.save_map_priority_enabled, 0 ratio = off, 0 free range = no seen-free check)",
      save_map_priority_enabled && save_map_priority_ratio > 0.0 ? "ON" : "off (every return within the cap)",
      save_map_priority_ratio, save_map_priority_margin, save_map_priority_voxel, save_map_priority_free_range);
  RCLCPP_INFO(node->get_logger(),
      "Saved-map product range cap: geometry %.1f m, colour %.1f m (0 = keep every return; only map_N.pcd / rgb_map_N.pcd are "
      "filtered, the archive and the estimator are not). Change with rtk.save_map_max_range / rtk.save_rgb_max_range, "
      "run_wuhu_round.sh --map-range M --rgb-range M, or launch save_map_max_range:=M save_rgb_max_range:=M",
      save_map_max_range, save_rgb_max_range);
  {
    parameters.param<double>("rtk/revisit_min_path_m", revisit_min_path, revisit_min_path);
    parameters.param<double>("rtk/keyframe_stationary_interval_sec", key_stationary_interval, key_stationary_interval);
    parameters.param<double>("rtk/stationary_translation_m", stationary_translation, stationary_translation);
    parameters.param<double>("rtk/stationary_speed_mps", stationary_speed, stationary_speed);
    double hold_rotation_deg = standstill_hold_rotation * 180 / M_PI;
    parameters.param<double>("rtk/standstill_hold_rotation_deg", hold_rotation_deg, hold_rotation_deg);
    parameters.param<double>("rtk/standstill_hold_translation_m", standstill_hold_translation,
                             standstill_hold_translation);
    standstill_hold_rotation = hold_rotation_deg * M_PI / 180;
    double stationary_rotation_deg = stationary_rotation * 180 / M_PI;
    parameters.param<double>("rtk/stationary_rotation_deg", stationary_rotation_deg, stationary_rotation_deg);
    double relin_rotation_deg = relinearize_rotation * 180 / M_PI;
    double relin_misalignment_deg = relinearize_misalignment * 180 / M_PI;
    parameters.param<double>("rtk/relinearize_rotation_deg", relin_rotation_deg, relin_rotation_deg);
    parameters.param<double>("rtk/relinearize_translation_m", relinearize_translation, relinearize_translation);
    parameters.param<double>("rtk/relinearize_misalignment_deg", relin_misalignment_deg, relin_misalignment_deg);
    parameters.param<int>("rtk/index_rebuild_keys", index_rebuild_keys, index_rebuild_keys);
    parameters.param<int>("rtk/online_log_interval_keys", online_log_interval, online_log_interval);
    for (double value : {revisit_min_path, key_stationary_interval, stationary_translation, stationary_speed,
                         stationary_rotation_deg})
      if (!std::isfinite(value) || value < 0)
        throw std::runtime_error("rtk revisit/stationary parameters must be finite and nonnegative");
    for (double value : {relin_rotation_deg, relinearize_translation, relin_misalignment_deg})
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("rtk/relinearize_* thresholds must be finite and positive");
    if (index_rebuild_keys < 1 || online_log_interval < 1)
      throw std::runtime_error("rtk/index_rebuild_keys and rtk/online_log_interval_keys must be >= 1");
    if (key_stationary_interval > 0 && key_stationary_interval < key_interval)
      throw std::runtime_error("rtk/keyframe_stationary_interval_sec must be 0 (off) or >= keyframe_interval_sec");
    stationary_rotation = stationary_rotation_deg * M_PI / 180;
    relinearize_rotation = relin_rotation_deg * M_PI / 180;
    relinearize_misalignment = relin_misalignment_deg * M_PI / 180;
    solver = gtsam::ISAM2(solverParameters());
    RCLCPP_INFO(node->get_logger(),
        "RTK graph budget: standstill knots every %.1f s (< %.2f m / %.2f deg / %.2f m/s, 0 = off), revisit >= %.0f s "
        "and >= %.0f m driven, relinearize %.4f deg / %.4f m / misalignment %.4f deg, candidate index rebuild every %d "
        "key(s), online cost log every %d key(s)",
        key_stationary_interval, stationary_translation, stationary_rotation_deg, stationary_speed, revisit_age,
        revisit_min_path, relin_rotation_deg, relinearize_translation, relin_misalignment_deg, index_rebuild_keys,
        online_log_interval);
  }
  {
    auto &L = loop_config;
    parameters.param<bool>("rtk/loop_closure_enabled", L.enabled, L.enabled);
    const std::pair<const char *, double *> loop_doubles[] = {
        {"loop_radius_m", &L.radius}, {"loop_min_path_m", &L.min_path}, {"loop_min_age_sec", &L.min_age},
        {"loop_spacing_m", &L.spacing}, {"loop_submap_half_path_m", &L.submap_half_path},
        {"loop_scan_spacing_sec", &L.scan_spacing}, {"loop_voxel_m", &L.voxel}, {"loop_max_range_m", &L.max_range},
        {"loop_max_correction_m", &L.max_correction}, {"loop_max_correction_deg", &L.max_correction_deg},
        {"loop_max_rms_m", &L.max_rms}, {"loop_min_overlap", &L.min_overlap}, {"loop_point_sigma_m", &L.point_sigma},
        {"loop_floor_translation_m", &L.floor_translation}, {"loop_floor_rotation_deg", &L.floor_rotation_deg},
        {"loop_rtk_bound_m", &L.rtk_bound}};
    for (const auto &item : loop_doubles)
    {
      parameters.param<double>(std::string("rtk/") + item.first, *item.second, *item.second);
      if (!std::isfinite(*item.second) || *item.second <= 0)
        throw std::runtime_error(std::string("rtk/") + item.first + " must be finite and positive");
    }
    parameters.param<int>("rtk/loop_effective_points", L.effective_points, L.effective_points);
    parameters.param<int>("rtk/loop_max_pairs", L.max_pairs, L.max_pairs);
    parameters.param<int>("rtk/loop_rtk_bound_keys", L.rtk_bound_keys, L.rtk_bound_keys);
    if (L.effective_points < 1 || L.max_pairs < 0 || L.rtk_bound_keys < 0)
      throw std::runtime_error("rtk/loop_effective_points must be >= 1 and rtk/loop_max_pairs >= 0");
  }
  if (!require_status || accepted_modes != std::vector<int>{4})
    throw std::runtime_error("RTK fusion requires current quality mode 4; bypassing quality or accepting other modes is forbidden");
  if (!std::isfinite(time_offset) || horizontal_sigma > max_horizontal_sigma ||
      vertical_sigma > max_vertical_sigma ||
      key_min_interval > key_interval || match_distance > plane_voxel ||
      max_planes < 20 || max_pair_factors > max_planes)
    throw std::runtime_error("Invalid RTK timing, mode list or geometry limits");
  key_rotation *= M_PI / 180.0;
  std::vector<double> rotation_sigmas, translation_sigmas;
  parameters.param("rtk/motion_rotation_sigma_deg_sqrt_sec", rotation_sigmas,
      std::vector<double>{motion_rotation_sigma.x() * 180 / M_PI,
                          motion_rotation_sigma.y() * 180 / M_PI,
                          motion_rotation_sigma.z() * 180 / M_PI});
  parameters.param("rtk/motion_translation_sigma_m_sqrt_sec", translation_sigmas,
      std::vector<double>{motion_translation_sigma.x(), motion_translation_sigma.y(), motion_translation_sigma.z()});
  for (const auto &values : {rotation_sigmas, translation_sigmas})
    if (values.size() != 3 || std::any_of(values.begin(), values.end(),
        [](double value) { return !std::isfinite(value) || value <= 0; }))
      throw std::runtime_error("Relative-motion sigmas require three finite positive RFU components");
  for (int axis = 0; axis < 3; ++axis)
  {
    motion_rotation_sigma(axis) = rotation_sigmas[axis] * M_PI / 180;
    motion_translation_sigma(axis) = translation_sigmas[axis];
  }
  parameters.param<bool>("rtk/observations_enabled", observations_enabled, observations_enabled);
  parameters.param<bool>("rtk/attitude_enabled", attitude_enabled, attitude_enabled);
  parameters.param<double>("rtk/attitude_huber_delta", attitude_huber, attitude_huber);
  parameters.param<double>("rtk/attitude_min_speed_mps", attitude_min_speed, attitude_min_speed);
  parameters.param<double>("rtk/attitude_motion_age_sec", attitude_motion_age, attitude_motion_age);
  if (!std::isfinite(attitude_min_speed) || attitude_min_speed < 0 ||
      !std::isfinite(attitude_motion_age) || attitude_motion_age < 0)
    throw std::runtime_error("rtk/attitude_min_speed_mps and attitude_motion_age_sec must be finite and nonnegative");
  std::vector<double> attitude_sigmas, misalignment_sigmas;
  parameters.param("rtk/attitude_sigma_deg", attitude_sigmas,
      std::vector<double>{attitude_sigma.x() * 180 / M_PI, attitude_sigma.y() * 180 / M_PI,
                          attitude_sigma.z() * 180 / M_PI});
  parameters.param("rtk/attitude_misalignment_prior_deg", misalignment_sigmas,
      std::vector<double>{misalignment_prior_sigma.x() * 180 / M_PI,
                          misalignment_prior_sigma.y() * 180 / M_PI,
                          misalignment_prior_sigma.z() * 180 / M_PI});
  for (const auto &values : {attitude_sigmas, misalignment_sigmas})
    if (values.size() != 3 || std::any_of(values.begin(), values.end(),
        [](double value) { return !std::isfinite(value) || value <= 0; }))
      throw std::runtime_error("INS attitude sigmas require three finite positive body-axis components");
  if (!std::isfinite(attitude_huber) || attitude_huber <= 0)
    throw std::runtime_error("rtk/attitude_huber_delta must be finite and positive");
  for (int axis = 0; axis < 3; ++axis)
  {
    attitude_sigma(axis) = attitude_sigmas[axis] * M_PI / 180;
    misalignment_prior_sigma(axis) = misalignment_sigmas[axis] * M_PI / 180;
  }
  std::string configured_directory;
  parameters.param<string>("rtk/output_directory", configured_directory, "");
  const auto base = projectResultDirectory(configured_directory.empty()
      ? std::filesystem::path(output_root) / "Log/fusion"
      : std::filesystem::path(configured_directory));
  std::filesystem::create_directories(base);
  directory = base / std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
  if (!std::filesystem::create_directory(directory))
    throw std::runtime_error("Fusion run directory already exists");
  diagnostics.exceptions(std::ios::badbit | std::ios::failbit);
  diagnostics.open(directory / "diagnostics.csv");
  diagnostics << "stamp,keyframes,position_factors,position_pending_attempts,planes,factors,rtk_residual_m,processing_ms,frontend_rtk_residual_m,frontend_speed_mps,reference_failure,global_aligned,prepare_ms,association_ms,solve_ms,refresh_ms,published_stamp,publish_lag_sec,latest_key_rtk_residual_m,unoptimized_tail_rtk_residual_m,refinement_steps,refinement_max_translation_m,refinement_max_rotation_deg,published_imu_reference_difference_m,relinearized,reeliminated,cliques\n";
  path_log.exceptions(std::ios::badbit | std::ios::failbit);
  path_log.open(directory / "online_path.csv");
  path_log << "stamp,assessed_frames,latest_imu_difference_m,max_imu_difference_m,worst_stamp,mean_imu_difference_m,over_10cm_frames,over_20cm_frames\n";
  optimization_log.exceptions(std::ios::badbit | std::ios::failbit);
  history_log.exceptions(std::ios::badbit | std::ios::failbit);
  optimization_log.open(directory / "optimization.csv");
  optimization_log << "stamp,phase,iteration,cost_before,cost_after,rtk_cost,local_motion_cost,geometry_cost,other_cost,scaled_gradient_inf,attitude_cost\n";
  history_log.open(directory / "history.csv");
  history_log << "update_stamp,key_stamp,age_sec,translation_change_m,rotation_change_deg,rtk_residual_m\n";
  association_log.exceptions(std::ios::badbit | std::ios::failbit);
  association_log.open(directory / "geometry_association.csv");
  association_log << "a_stamp,b_stamp,candidates,consistent,initial_rms_m,registered_rms_m,rejected_rms_m,registration_translation_m,registration_rotation_deg,coarse_candidates,shared_support_fraction,baseline_difference_m,baseline_sigma_m,factor_accepted\n";
  geometry_history.exceptions(std::ios::badbit | std::ios::failbit);
  geometry_history.open(directory / "geometry_history.csv");
  geometry_history << "update_stamp,phase,a_stamp,b_stamp,rows,rms_m,p95_m,max_m,initial_rms_m,native_rms_m,residual_change_rms_m,relative_motion_max_m,anchor_world_motion_max_m\n";
  // Persist effective parameters, not a copy of a possibly unrelated YAML.
  std::ofstream settings(directory / "parameters.txt");
  settings.exceptions(std::ios::badbit | std::ios::failbit);
  settings << "model=native_lio_global_correction; attitude_observation=ins_body_attitude_with_installation_misalignment; map_projection=native_deskew_world_point_with_point_time_correction; surface_factor_model=scan_end_rigid; surface_residual=common_support_mean_squared_point_distance; motion_residual=so3_and_endpoint_position; motion_rotation_loss=component_huber; motion_translation_loss=component_huber; local_geometry=bounded_shared_support; overlap_selection=observed_surface_covisibility; geometry_validation=observable_baseline_consistency; association_lifetime=verified_observation\n";
  // EFFECTIVE INS association offsets (after the calibration-driven reference shift).
  // Written first on purpose: the evaluation greps the first match.
  settings << std::setprecision(17)
           << "rtk.time_offset_sec=" << time_offset << '\n'
           << "rtk.attitude_time_offset_sec=" << attitude_time_offset << '\n';
  for (const auto &name : node->list_parameters({}, 0).names)
  {
    const auto parameter = node->get_parameter(name);
    settings << name << "=";
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      settings << std::setprecision(17) << parameter.as_double();
    else settings << parameter.value_to_string();
    settings << '\n';
  }
  if (save_map)
  {
    archive.exceptions(std::ios::badbit | std::ios::failbit);
    frame_index.exceptions(std::ios::badbit | std::ios::failbit);
    archive.open(directory / "scans.bin", std::ios::binary);
    archive.write("LIVORTK3", 8);
    // sources.bin LIVOSRC1: one uint8 per scans.bin return, same order; value = index of
    // the configured LiDAR whose (deskewed) beam origin is nearest, 255 = unknown.
    source_archive.exceptions(std::ios::badbit | std::ios::failbit);
    source_archive.open(directory / "sources.bin", std::ios::binary);
    source_archive.write("LIVOSRC1", 8);
    if (color_enabled)
    {
      color_archive.exceptions(std::ios::badbit | std::ios::failbit);
      color_archive.open(directory / "colors.bin", std::ios::binary);
      color_archive.write("LIVORGB1", 8);
    }
    frame_index.open(directory / "frames.csv");
    frame_index << std::setprecision(17);
    frame_index << "# scans.bin LIVORTK3: little-endian float32 x,y,z,intensity,offset_ms; NATIVE deskewed virtual LiDAR returns; one scan-end transform\n";
    frame_index << "# lidar_to_imu_translation " << lidar_translation.transpose() << '\n';
    frame_index << "# lidar_to_imu_rotation_row_major";
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column) frame_index << ' ' << lidar_rotation(row, column);
    frame_index << "\nstamp,begin,offset_bytes,count,tx,ty,tz,qx,qy,qz,qw,color_offset_bytes,color_count\n";
  }
  path_publisher = node->create_publisher<nav_msgs::Path>("/rtk/path", rclcpp::QoS(1).transient_local());
  odom_publisher = node->create_publisher<nav_msgs::Odometry>("/rtk/odometry", 10);
  map_publisher = node->create_publisher<sensor_msgs::PointCloud2>("/rtk/map", rclcpp::QoS(1).transient_local());
  current_publisher = node->create_publisher<sensor_msgs::PointCloud2>("/rtk/cloud_current", 1);
  graph_publisher = node->create_publisher<visualization_msgs::MarkerArray>("/rtk/graph", rclcpp::QoS(1).transient_local());
  final_map_publisher = node->create_publisher<sensor_msgs::PointCloud2>("/rtk/map_final", rclcpp::QoS(1).transient_local());
  RCLCPP_INFO(node->get_logger(),
      "LIO RTK geometry fusion: rear-axle lever [%.3f %.3f %.3f], nominal sigma %.3f/%.3f m, RGB map=%s, results %s",
      lever.x(), lever.y(), lever.z(), horizontal_sigma, vertical_sigma,
      color_enabled ? "camera" : "intensity display", directory.c_str());
  worker = std::thread([this] {
    try
    {
      double flushed_stamp = -1;
      while (true)
      {
        Frame frame;
        {
          std::unique_lock<std::mutex> lock(mutex);
          if (!changed.wait_for(lock, std::chrono::seconds(1),
                                [this] { return stopping || !queue.empty(); }))
          {
            lock.unlock();
            // Wall-clock stalls must never alter the factor graph. Inserting
            // a tail keyframe here bypasses the data-time keyframe interval,
            // making graph topology depend on CPU scheduling. During an idle
            // period publish only the newest already-solved state; finish()
            // still forces the true final tail into the graph before export.
            if (!recent_frames.empty() && recent_frames.back().stamp > flushed_stamp)
            {
              const auto &tail = recent_frames.back();
              if (!keyframes.empty())
                publish(std::min(tail.stamp, keyframes.back().stamp), true);
              flushed_stamp = tail.stamp;
            }
            continue;
          }
          if (queue.empty() && stopping) break;
          frame = std::move(queue.front());
          queue.pop_front();
        }
        changed.notify_all();
        process(std::move(frame));
      }
      if (!recent_frames.empty() && (keyframes.empty() ||
          recent_frames.back().stamp > keyframes.back().stamp + 1e-6))
        optimizeKeyframe(recent_frames.back());
      if (!keyframes.empty())
      {
        finalizeOptimization();
        publish(frames.back().stamp, true);
        exportMap();
      }
    }
    catch (...)
    {
      {
        std::lock_guard<std::mutex> lock(mutex);
        failure = std::current_exception();
        stopping = true;
      }
      changed.notify_all();
      RCLCPP_ERROR(node->get_logger(), "RTK backend failed; stopping mapping to prevent incomplete results");
      rclcpp::shutdown();
    }
  });
}

gtsam::ISAM2Params RtkFusion::solverParameters() const
{
  return fusionSolverParameters(relinearize_rotation, relinearize_translation, relinearize_misalignment);
}

void RtkFusion::accumulateSolverStats(const gtsam::ISAM2Result &result)
{
  last_relinearized += result.variablesRelinearized;
  last_reeliminated += result.variablesReeliminated;
  last_cliques = std::max<std::size_t>(last_cliques, result.cliques);
}

void RtkFusion::indexAnchor(std::size_t i)
{
  const V3D p = keyframes[i].optimized.translation();
  if (!p.allFinite()) throw std::runtime_error("Non-finite optimized pose");
  spatial_keys[{static_cast<std::int64_t>(std::floor(p.x() / revisit_radius)),
                static_cast<std::int64_t>(std::floor(p.y() / revisit_radius)),
                static_cast<std::int64_t>(std::floor(p.z() / revisit_radius))}].push_back(i);
  for (const auto &plane : keyframes[i].planes)
  {
    const V3D point = keyframes[i].optimized.transformFrom(plane.center) / (2 * plane_voxel);
    auto &keys = surface_keys[{static_cast<std::int64_t>(std::floor(point.x())),
        static_cast<std::int64_t>(std::floor(point.y())), static_cast<std::int64_t>(std::floor(point.z()))}];
    // Each scan owns at most one vote per occupied cell, irrespective of
    // how many extracted patches/returns it has in that cell.
    if (keys.empty() || keys.back() != i) keys.push_back(i);
  }
}

void RtkFusion::appendParameters(const std::vector<std::pair<std::string, std::string>> &items) const
{
  std::ofstream settings(directory / "parameters.txt", std::ios::app);
  settings.exceptions(std::ios::badbit | std::ios::failbit);
  for (const auto &item : items) settings << item.first << '=' << item.second << '\n';
}

RtkFusion::~RtkFusion()
{
  try { finish(); }
  catch (const std::exception &error)
  { RCLCPP_ERROR(node->get_logger(), "RTK finalization: %s", error.what()); }
}

void RtkFusion::finish()
{
  {
    std::lock_guard<std::mutex> lock(mutex);
    stopping = true;
  }
  changed.notify_all();
  if (worker.joinable()) worker.join();
  if (failure) std::rethrow_exception(failure);
}

gtsam::Vector6 RtkFusion::motionSigmas(double duration) const
{
  if (!std::isfinite(duration) || duration <= 0)
    throw std::runtime_error("Nonpositive relative-motion interval");
  gtsam::Vector6 sigmas;
  sigmas << motion_rotation_sigma, motion_translation_sigma;
  // Brownian increment model: changing knot spacing must not multiply
  // information about the same elapsed interval. These engineering sigmas
  // need real-map validation; H^-1 of a fixed frontend map is not covariance.
  sigmas *= std::sqrt(duration);
  return sigmas;
}

void RtkFusion::updatePoses()
{
  estimate = solver.calculateBestEstimate();
  for (std::size_t i = 0; i < keyframes.size(); ++i)
  {
    auto &key = keyframes[i];
    key.optimized = observations_enabled ? estimate.at<gtsam::Pose3>(gtsam::Symbol('x', i)) : key.raw;
    if (!key.optimized.matrix().allFinite())
      throw std::runtime_error("Non-finite optimized global pose");
  }
}

void RtkFusion::initializePose(const gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
{
  auto &current = keyframes.back();
  if (!current.has_position) return;
  const gtsam::Key key = gtsam::Symbol('x', keyframes.size() - 1);
  gtsam::NonlinearFactorGraph local;
  for (const auto &factor : graph)
    if (factor && std::find(factor->keys().begin(), factor->keys().end(), key) != factor->keys().end())
      local.push_back(factor);
  gtsam::Values values;
  values.insert(key, current.optimized);
  for (const auto &factor : local)
    for (gtsam::Key other : factor->keys())
    {
      if (values.exists(other)) continue;
      // Non-pose variables (installation misalignment) condition the local
      // solve at their current or freshly initialized value.
      if (initial.exists(other)) values.insert(other, initial.at(other));
      else values.insert(other, estimate.at(other));
    }
  double cost = local.error(values);
  // Choose an initial guess by the SAME motion/RTK objective. A failed native
  // increment must not seed associations metres outside their capture range.
  // This is not a pose overwrite: no new prior, attitude observation, or
  // altered RTK weight is introduced; all history remains variable in iSAM2.
  const auto native = current.optimized;
  values.update(key, gtsam::Pose3(native.rotation(),
      current.reference - native.rotation().rotate(lever)));
  if (local.error(values) < cost) cost = local.error(values);
  else values.update(key, native);
  // Only one six-dimensional block is solved here. Other poses are held at
  // the current estimate for initialization, not frozen in the actual graph.
  for (int iteration = 0; iteration < 8; ++iteration)
  {
    const auto linear = local.linearize(values);
    std::size_t rows = 0;
    for (const auto &factor : *linear)
    {
      const auto jacobian = std::dynamic_pointer_cast<gtsam::JacobianFactor>(factor);
      if (!jacobian) throw std::runtime_error("Expected Jacobian in conditional pose initialization");
      rows += jacobian->rows();
    }
    gtsam::Matrix a(rows, 6);
    gtsam::Vector b(rows);
    std::size_t offset = 0;
    for (const auto &factor : *linear)
    {
      const auto jacobian = std::dynamic_pointer_cast<gtsam::JacobianFactor>(factor);
      if (!jacobian) throw std::runtime_error("Expected Jacobian in conditional pose initialization");
      a.middleRows(offset, jacobian->rows()) = jacobian->getA(jacobian->find(key));
      b.segment(offset, jacobian->rows()) = jacobian->getb();
      offset += jacobian->rows();
    }
    const gtsam::Vector6 step = a.colPivHouseholderQr().solve(b);
    if (!step.allFinite()) throw std::runtime_error("Nonfinite conditional pose increment");
    if (step.head<3>().norm() < 1e-5 && step.tail<3>().norm() < 1e-4) break;
    const auto previous = values.at<gtsam::Pose3>(key);
    bool accepted = false;
    for (double scale = 1; scale >= 1. / 128; scale *= .5)
    {
      values.update(key, previous.retract(scale * step));
      const double trial = local.error(values);
      if (std::isfinite(trial) && trial < cost)
      { cost = trial; accepted = true; break; }
    }
    if (!accepted) { values.update(key, previous); break; }
  }
  current.optimized = observations_enabled ? values.at<gtsam::Pose3>(key) : current.raw;
  initial.update(key, current.optimized);
}


void RtkFusion::logOptimization(const char *phase, int iteration, double before, double after)
{
  const auto &graph = solver.getFactorsUnsafe();
  if (geometry_probes.empty() ||
      (keyframes.back().raw.translation() - keyframes[last_probe_key].raw.translation()).norm() >= 10)
    for (auto link = geometry_links.rbegin(); link != geometry_links.rend(); ++link)
    {
      if (link->b != keyframes.size() - 1) break;
      if (link->factor_index == std::numeric_limits<std::size_t>::max()) continue;
      const auto factor = std::dynamic_pointer_cast<const ScanPlaneBatchFactor>(graph.at(link->factor_index));
      if (!factor) continue;
      GeometryProbe probe{link->a, link->b, factor,
          keyframes[link->a].optimized, keyframes[link->b].optimized};
      if (geometry_probes.size() < 24) geometry_probes.push_back(std::move(probe));
      else
      {
        geometry_probes[next_probe_slot] = std::move(probe);
        next_probe_slot = 8 + (next_probe_slot - 8 + 1) % 16;
      }
      last_probe_key = link->b;
      break;
    }
  for (auto &probe : geometry_probes)
  {
    const auto &a = keyframes[probe.a];
    const auto &b = keyframes[probe.b];
    const auto relative = a.optimized.between(b.optimized);
    const auto previous = probe.previous_a.between(probe.previous_b);
    const auto native = a.raw.between(b.raw);
    double squared = 0, native_squared = 0, change_squared = 0, relative_motion = 0, world_motion = 0;
    std::vector<double> magnitudes;
    for (const auto &row : probe.factor->constraints())
    {
      const V3D point = relative.transformFrom(row.point), old_point = previous.transformFrom(row.point);
      const auto projected = row.physicalResidual(relative);
      squared += projected.squaredNorm();
      native_squared += row.physicalResidual(native).squaredNorm();
      change_squared += (projected - row.physicalResidual(previous)).squaredNorm();
      relative_motion = std::max(relative_motion, (point - old_point).norm());
      world_motion = std::max(world_motion,
          (a.optimized.transformFrom(row.center) - probe.previous_a.transformFrom(row.center)).norm());
      magnitudes.push_back(projected.norm());
    }
    const double rms = std::sqrt(squared / magnitudes.size());
    if (probe.initial_rms < 0) probe.initial_rms = rms;
    std::sort(magnitudes.begin(), magnitudes.end());
    const double rank = .95 * (magnitudes.size() - 1);
    const auto low = static_cast<std::size_t>(rank);
    const double p95 = magnitudes[low] + (rank - low) *
        (magnitudes[std::min(low + 1, magnitudes.size() - 1)] - magnitudes[low]);
    // Unsaturated metre residuals of EXACTLY the same local representatives
    // and normals, even when their original graph factor has been replaced.
    // A common world rigid move changes neither the residual nor relative
    // motion. These model probes complement, not replace actual PCD checks.
    geometry_history << std::setprecision(17) << keyframes.back().stamp << ',' << phase << ','
        << a.stamp << ',' << b.stamp << ',' << magnitudes.size() << ',' << rms << ',' << p95 << ','
        << magnitudes.back() << ',' << probe.initial_rms << ',' << std::sqrt(native_squared / magnitudes.size())
        << ',' << std::sqrt(change_squared / magnitudes.size()) << ',' << relative_motion << ',' << world_motion << '\n';
    probe.previous_a = a.optimized;
    probe.previous_b = b.optimized;
  }
  double costs[5] = {};
  for (const auto &factor : graph)
  {
    if (!factor) continue;
    const int group = dynamic_cast<const gtsam::GPSFactorArm *>(factor.get()) ? 0 :
        dynamic_cast<const NativeMotionFactor *>(factor.get()) ? 1 :
        dynamic_cast<const ScanPlaneBatchFactor *>(factor.get()) ? 2 :
        dynamic_cast<const InsAttitudeFactor *>(factor.get()) ? 4 : 3;
    costs[group] += factor->error(estimate);
  }
  double gradient_norm = std::numeric_limits<double>::quiet_NaN();
  // Full relinearization is an offline convergence diagnostic, not an extra
  // whole-graph linearization on every online LiDAR frame.
  if (std::string(phase) != "online")
  {
    const auto linear = graph.linearize(estimate);
    const auto gradient = linear->gradientAtZero();
    const auto diagonal = linear->hessianDiagonal();
    gradient_norm = 0;
    for (const auto &entry : gradient)
      gradient_norm = std::max(gradient_norm,
          (entry.second.array().abs() / diagonal.at(entry.first).array().max(1e-20).sqrt()).maxCoeff());
  }
  optimization_log << std::setprecision(17) << keyframes.back().stamp << ',' << phase << ',' << iteration
      << ',' << before << ',' << after;
  for (int group = 0; group < 4; ++group) optimization_log << ',' << costs[group];
  optimization_log << ',' << gradient_norm << ',' << costs[4] << '\n';
  // Fixed early anchors plus worst historical changes/residuals. Do not emit
  // an O(N^2) full-history log during long mine surveys.
  std::set<std::size_t> monitored{0, keyframes.size() - 1};
  for (double offset : {5.0, 10.0, 20.0, 30.0})
  {
    const auto found = std::lower_bound(keyframes.begin(), keyframes.end(), keyframes.front().stamp + offset,
        [](const Keyframe &key, double stamp) { return key.stamp < stamp; });
    if (found != keyframes.end()) monitored.insert(static_cast<std::size_t>(found - keyframes.begin()));
  }
  double worst_change = 0, worst_residual = 0;
  std::size_t changed_key = 0, residual_key = 0;
  for (std::size_t i = 0; i < keyframes.size(); ++i)
  {
    const auto &key = keyframes[i];
    if (i < last_logged_poses.size() && keyframes.back().stamp - key.stamp > 30)
    {
      const auto delta = last_logged_poses[i].between(key.optimized);
      const double change = delta.translation().norm() + 60 * gtsam::Rot3::Logmap(delta.rotation()).norm();
      if (change > worst_change) { worst_change = change; changed_key = i; }
    }
    if (key.has_position)
    {
      const double residual = (key.optimized.transformFrom(lever) - key.reference).norm();
      if (residual > worst_residual) { worst_residual = residual; residual_key = i; }
    }
  }
  monitored.insert(changed_key); monitored.insert(residual_key);
  for (std::size_t i : monitored)
  {
    const auto &key = keyframes[i];
    const auto change = i < last_logged_poses.size() ? last_logged_poses[i].between(key.optimized) : gtsam::Pose3();
    const double residual = key.has_position ? (key.optimized.transformFrom(lever) - key.reference).norm() :
        std::numeric_limits<double>::quiet_NaN();
    history_log << std::setprecision(17) << keyframes.back().stamp << ',' << key.stamp << ','
        << keyframes.back().stamp - key.stamp << ',' << change.translation().norm() << ','
        << gtsam::Rot3::Logmap(change.rotation()).norm() * 180 / M_PI << ',' << residual << '\n';
  }
  last_logged_poses.clear();
  for (const auto &key : keyframes) last_logged_poses.push_back(key.optimized);
}

void RtkFusion::finalizeOptimization()
{
  // Measurements do not change merely because their poses are optimized.
  // Re-cropping/reselecting patches after each 0.1 degree update changed the
  // objective and produced a two-cycle without any new sensor observations.
  // Keep verified correspondences and robust physical residuals; ALL poses
  // remain free. New scans supply new measurements through optimizeKeyframe.
  const auto base = solver.getFactorsUnsafe();
  const auto loops = collectLoopClosures();
  std::vector<bool> keep(loops.size(), true);
  const gtsam::Values initial = estimate;
  gtsam::NonlinearFactorGraph graph;
  double before = 0, after = 0;
  std::size_t iterations = 0, dropped = 0;
  gtsam::LevenbergMarquardtParams parameters;
  parameters.maxIterations = 30;
  parameters.relativeErrorTol = 1e-7;
  parameters.absoluteErrorTol = 1e-8;
  parameters.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
  std::ofstream bound_log;
  for (int pass = 0;; ++pass)
  {
    graph = base;
    for (std::size_t i = 0; i < loops.size(); ++i)
      if (keep[i]) graph.push_back(loops[i].factor);
    before = graph.error(initial);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
    estimate = optimizer.optimize();
    iterations = optimizer.iterations();
    after = graph.error(estimate);
    if (!std::isfinite(after) || after > before + 1e-7 * std::max(1.0, before))
      throw std::runtime_error("Final optimizer did not preserve/decrease the fixed-graph objective");
    if (loops.empty() || pass >= 3) break;
    // RTK bound: loops redistribute the RTK error between passes, they must not move the route out of the budget.
    std::vector<std::size_t> offending;
    double worst = 0;
    for (std::size_t k = 0; k < keyframes.size(); ++k)
    {
      if (!keyframes[k].has_position || !estimate.exists(gtsam::Symbol('x', k))) continue;
      const double r =
          (estimate.at<gtsam::Pose3>(gtsam::Symbol('x', k)).transformFrom(lever) - keyframes[k].reference).norm();
      worst = std::max(worst, r);
      if (r > loop_config.rtk_bound) offending.push_back(k);
    }
    if (offending.empty()) break;
    bool changed = false;
    for (std::size_t i = 0; i < loops.size(); ++i)
    {
      if (!keep[i]) continue;
      for (std::size_t k : offending)
      {
        const auto near = [&](std::size_t end) {
          return (end > k ? end - k : k - end) <= static_cast<std::size_t>(loop_config.rtk_bound_keys);
        };
        if (near(loops[i].a) || near(loops[i].b))
        {
          keep[i] = false;
          changed = true;
          ++dropped;
          if (!bound_log.is_open())
          {
            bound_log.open(directory / "loops_rtk_bound.csv");
            bound_log << "pass,a_stamp,b_stamp,offending_key_stamp,worst_residual_m\n";
          }
          bound_log << pass << ',' << std::setprecision(17) << keyframes[loops[i].a].stamp << ','
                    << keyframes[loops[i].b].stamp << ',' << keyframes[k].stamp << ',' << std::setprecision(6)
                    << worst << '\n';
          break;
        }
      }
    }
    RCLCPP_INFO(node->get_logger(),
        "Loop closure RTK bound pass %d: max keyframe residual %.3f m > %.3f m at %zu keyframe(s); %zu loop factor(s) "
        "dropped so far%s", pass, worst, loop_config.rtk_bound, offending.size(), dropped,
        changed ? ", re-solving" : " (no loop factor near them: residual not caused by loops, solution kept)");
    if (!changed) break;
  }
  for (std::size_t i = 0; i < loops.size(); ++i)
    if (!keep[i]) loop_display[loops[i].display].state = 2;
  finalized = true;
  if (!loops.empty())
    RCLCPP_INFO(node->get_logger(), "Loop closure: %zu of %zu loop factors kept in the final solve (RTK bound %.3f m)",
                loops.size() - dropped, loops.size(), loop_config.rtk_bound);
  solver = gtsam::ISAM2(solverParameters());
  const auto rebuilt = solver.update(graph, estimate);
  for (auto &link : geometry_links)
    link.factor_index = rebuilt.newFactorsIndices.at(link.factor_index);
  updatePoses();
  logOptimization("final_fixed_graph", iterations, before, after);
  logOptimization("final", 0, solver.getFactorsUnsafe().error(estimate), solver.getFactorsUnsafe().error(estimate));
  optimization_log.flush();
  history_log.flush();
  geometry_history.flush();
}

std::vector<RtkFusion::LoopFactorRecord> RtkFusion::collectLoopClosures()
{
  std::vector<LoopFactorRecord> out;
  if (!loop_config.enabled || !save_map || !observations_enabled || keyframes.size() < 20 || loop_config.max_pairs == 0)
    return out;
  const auto start = std::chrono::steady_clock::now();
  archive.flush();
  std::vector<LoopScanRef> scans(frames.size());
  std::vector<gtsam::Pose3> delivered(frames.size());
  double path = 0;
  for (std::size_t i = 0; i < frames.size(); ++i)
  {
    if (i) path += (frames[i].raw.translation() - frames[i - 1].raw.translation()).norm();
    scans[i] = {frames[i].offset, frames[i].count, frames[i].stamp, frames[i].raw, path};
    delivered[i] = poseAt(frames[i]);
  }
  std::vector<std::size_t> keyframe_scan(keyframes.size());
  std::vector<Eigen::Vector3d> positions(keyframes.size());
  std::vector<double> paths(keyframes.size()), stamps(keyframes.size());
  std::vector<bool> usable(keyframes.size());
  std::vector<gtsam::Pose3> optimized(keyframes.size());
  for (std::size_t k = 0; k < keyframes.size(); ++k)
  {
    const auto found = std::lower_bound(frames.begin(), frames.end(), keyframes[k].stamp - 1e-6,
        [](const Frame &frame, double stamp) { return frame.stamp < stamp; });
    usable[k] = found != frames.end() && std::abs(found->stamp - keyframes[k].stamp) < 1e-5;
    keyframe_scan[k] = usable[k] ? static_cast<std::size_t>(found - frames.begin()) : 0;
    optimized[k] = keyframes[k].optimized;
    positions[k] = keyframes[k].optimized.translation();
    paths[k] = usable[k] ? scans[keyframe_scan[k]].path : 0;
    stamps[k] = keyframes[k].stamp;
  }
  const auto pairs = detectLoopPairs(loop_config, positions, paths, stamps, usable);
  if (pairs.empty())
  {
    RCLCPP_INFO(node->get_logger(), "Loop closure: no revisited section (>= %.0f m driven, <= %.1f m apart)",
                loop_config.min_path, loop_config.radius);
    return out;
  }
  const auto results = registerLoopPairs(loop_config, (directory / "scans.bin").string(), lidar_rotation,
                                         lidar_translation, scans, delivered, keyframe_scan, optimized, pairs);
  std::ofstream log(directory / "loops.csv");
  log << "a_stamp,b_stamp,path_gap_m,distance_m,correspondences,overlap,rms_before_m,rms_after_m,correction_m,"
         "correction_deg,shift_x_m,shift_y_m,shift_z_m,min_eig_translation,min_eig_rotation,accepted,reason,"
         "rot_x_deg,rot_y_deg,rot_z_deg\n";
  std::size_t accepted = 0;
  std::vector<double> before, after, moves;
  for (const auto &pair : results)
  {
    log << std::setprecision(17) << keyframes[pair.a].stamp << ',' << keyframes[pair.b].stamp << ','
        << std::setprecision(6) << paths[pair.b] - paths[pair.a] << ','
        << (positions[pair.b] - positions[pair.a]).norm() << ',' << pair.correspondences << ',' << pair.overlap << ','
        << pair.rms_before << ',' << pair.rms_after << ',' << pair.correction << ',' << pair.correction_deg << ','
        << pair.shift.x() << ',' << pair.shift.y() << ',' << pair.shift.z() << ','
        << pair.min_eigen_translation << ',' << pair.min_eigen_rotation << ',' << pair.accepted << ','
        << pair.reason << ',' << pair.rotation_deg.x() << ',' << pair.rotation_deg.y() << ','
        << pair.rotation_deg.z() << '\n';
    loop_display.push_back({pair.a, pair.b, pair.accepted ? 0 : 1});
    if (!pair.accepted) continue;
    // Robust (Huber, 3 sigma) relative-pose factor with the ICP information: observable directions only.
    const auto noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(3.0),
        gtsam::noiseModel::Gaussian::Information(pair.information));
    out.push_back({pair.a, pair.b, std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                                       gtsam::Symbol('x', pair.a), gtsam::Symbol('x', pair.b), pair.measured, noise),
                   loop_display.size() - 1});
    ++accepted;
    before.push_back(pair.rms_before);
    after.push_back(pair.rms_after);
    moves.push_back(pair.correction);
  }
  const auto median = [](std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  };
  RCLCPP_INFO(node->get_logger(),
      "Loop closure: %zu revisit pairs, %zu accepted factors (point-to-plane rms median %.3f -> %.3f m, correction "
      "median %.3f m) in %.1f s; details in loops.csv",
      results.size(), accepted, median(before), median(after), median(moves),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  return out;
}

gtsam::Pose3 RtkFusion::correctionAt(double stamp) const
{
  if (keyframes.empty()) throw std::runtime_error("Global correction has no pose knots");
  // observations_enabled=false: the delivered/online trajectory IS the native
  // trajectory by definition; never apply interim gauge/knot corrections.
  if (!observations_enabled) return gtsam::Pose3();
  const auto right = std::lower_bound(keyframes.begin(), keyframes.end(), stamp,
      [](const Keyframe &key, double time) { return key.stamp < time; });
  const auto correction = [](const Keyframe &key) { return key.optimized * key.raw.inverse(); };
  if (right == keyframes.begin()) return correction(*right);
  if (right == keyframes.end()) return correction(keyframes.back());
  const auto &left = *std::prev(right);
  // Interpolate GLOBAL CORRECTION, not vehicle poses: preserve native
  // high-rate roll, pitch, steering and translations between sparse knots.
  // Before/after bounded history hold correction, never extrapolate it.
  return interpolateTransform(correction(left), correction(*right),
      (stamp - left.stamp) / (right->stamp - left.stamp));
}

void RtkFusion::observe(const nav_msgs::Odometry &message)
{
  Observation sample;
  sample.stamp = stampToSec(message.header.stamp) - time_offset;
  sample.position << message.pose.pose.position.x, message.pose.pose.position.y,
                     message.pose.pose.position.z;
  if (!std::isfinite(sample.stamp)) return;
  sample.position_valid = sample.position.allFinite();
  const auto &q = message.pose.pose.orientation;
  sample.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  sample.orientation_valid = sample.orientation.coeffs().allFinite() &&
      std::abs(sample.orientation.norm() - 1.0) < 0.01;
  if (sample.orientation_valid) sample.orientation.normalize();
  if (sample.orientation_valid && sample.position_valid)
    sample.imu_position = sample.position - sample.orientation * lever;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      sample.covariance(row, column) = message.pose.covariance[6 * row + column];
  const M3D fallback = V3D(horizontal_sigma * horizontal_sigma,
      horizontal_sigma * horizontal_sigma, vertical_sigma * vertical_sigma).asDiagonal();
  const bool unknown_covariance = sample.covariance.isZero(0);
  sample.covariance = (0.5 * (sample.covariance + sample.covariance.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<M3D> eig;
  if (sample.covariance.allFinite()) eig.compute(sample.covariance);
  sample.position_valid = sample.position_valid && sample.covariance.allFinite() && eig.info() == Eigen::Success &&
      eig.eigenvalues().minCoeff() >= -1e-12;
  // Unknown covariance uses a TOTAL nominal fallback. For reported covariance
  // the receiver scale and optional, explicitly configured model variance are
  // separate. Never add the missing-data fallback to a valid report.
  // Invalid reports retain their timestamps and break interpolation, rather
  // than being repaired into apparently precise fixed observations.
  sample.covariance_source = unknown_covariance ? 1U : 2U;
  if (unknown_covariance) sample.covariance = fallback;
  else
  {
    const M3D model = V3D(reported_model_sigma_xy * reported_model_sigma_xy,
        reported_model_sigma_xy * reported_model_sigma_xy,
        reported_model_sigma_z * reported_model_sigma_z).asDiagonal();
    sample.covariance = covariance_scale * sample.covariance + model;
  }
  if (sample.position_valid)
  {
    eig.compute(sample.covariance);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> horizontal(sample.covariance.topLeftCorner<2, 2>());
    sample.position_valid = eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0 &&
        horizontal.eigenvalues().maxCoeff() <= max_horizontal_sigma * max_horizontal_sigma &&
        sample.covariance(2, 2) <= max_vertical_sigma * max_vertical_sigma;
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!observations.empty() && sample.stamp <= observations.back().stamp) return;
    observations.push_back(sample);
    while (observations.size() > 2 && observations.front().stamp < sample.stamp - 60)
      observations.pop_front();
  }
  changed.notify_all();
}

void RtkFusion::observeStatus(const diagnostic_msgs::msg::DiagnosticArray &message)
{
  Quality quality{stampToSec(message.header.stamp) - time_offset, -1, -1};
  if (!std::isfinite(quality.stamp)) return;
  const auto parse = [](const std::string &text) {
    int value = -1;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc() && result.ptr == text.data() + text.size() ? value : -1;
  };
  {
    for (const auto &status : message.status)
      for (const auto &value : status.values)
      {
        if (value.key == "ins_pos_mode") quality.mode = parse(value.value);
        if (value.key == "gps_num_sats") quality.satellites = parse(value.value);
      }
  }
  std::lock_guard<std::mutex> lock(mutex);
  if (!qualities.empty() && quality.stamp <= qualities.back().stamp) return;
  if (quality.mode == 4 && quality.satellites >= minimum_satellites)
  {
    quality.fixed_since = quality.stamp;
    if (!qualities.empty() && qualities.back().fixed_since >= 0 &&
        quality.stamp - qualities.back().stamp <= status_age)
      quality.fixed_since = qualities.back().fixed_since;
  }
  if (qualities.empty() || quality.mode != qualities.back().mode)
    RCLCPP_INFO(node->get_logger(),
        "RTK quality transition at %.6f: mode %d, satellites %d; only stable mode 4 contributes observations",
        quality.stamp, quality.mode, quality.satellites);
  qualities.push_back(quality);
  while (qualities.size() > 2 && qualities.front().stamp < quality.stamp - 60)
    qualities.pop_front();
}

bool RtkFusion::referenceAt(double stamp, Observation &result, int *failure_reason)
{
  std::lock_guard<std::mutex> lock(mutex);
  const auto reject = [failure_reason](int reason) {
    if (failure_reason) *failure_reason = reason;
    return false;
  };
  const auto after = std::lower_bound(observations.begin(), observations.end(), stamp,
      [](const Observation &sample, double time) { return sample.stamp < time; });
  if (after == observations.end()) return reject(1);
  const auto before = (std::abs(after->stamp - stamp) < 1e-6 || after == observations.begin())
      ? after : std::prev(after);
  if (before->stamp > stamp + 1e-6) return reject(1);
  if (after->stamp - before->stamp > interpolation_gap) return reject(2);
  const auto validQuality = [this, &reject](double time) {
    auto quality = std::upper_bound(qualities.begin(), qualities.end(), time + 1e-6,
        [](double t, const Quality &sample) { return t < sample.stamp; });
    if (quality == qualities.begin()) return reject(3);
    --quality;
    if (time - quality->stamp > status_age) return reject(4);
    if (quality->mode != 4) return reject(5);
    if (quality->satellites < minimum_satellites) return reject(6);
    if (quality->fixed_since < 0 || time - quality->fixed_since < recovery_stable_interval)
      return reject(7);
    return true;
  };
  if (!validQuality(before->stamp) || !validQuality(after->stamp)) return false;
  if (!before->position_valid || !after->position_valid) return reject(8);
  const double interval = after->stamp - before->stamp;
  const double ratio = interval < 1e-6 ? 0.0 : (stamp - before->stamp) / interval;
  result.stamp = stamp;
  result.position = (1.0 - ratio) * before->position + ratio * after->position;
  // Match RViz: relocate each original INS sample before interpolation.
  result.imu_position = (1.0 - ratio) * before->imu_position + ratio * after->imu_position;
  // Consecutive INS outputs are correlated; interpolation does not earn the
  // variance reduction of two independent observations.
  result.covariance = (1.0 - ratio) * before->covariance + ratio * after->covariance;
  result.covariance_source = before->covariance_source | after->covariance_source;
  // Orientation may use its own latency: the sample stamped h is the attitude at
  // h - attitude_time_offset, i.e. the observation with stamp s + attitude_time_offset - time_offset.
  const double orientation_stamp = stamp + attitude_time_offset - time_offset;
  auto o_after = after;
  auto o_before = before;
  if (std::abs(orientation_stamp - stamp) > 1e-9)
  {
    o_after = std::lower_bound(observations.begin(), observations.end(), orientation_stamp,
        [](const Observation &sample, double time) { return sample.stamp < time; });
    if (o_after == observations.end()) { result.orientation_valid = false; if (failure_reason) *failure_reason = 0; return true; }
    o_before = (std::abs(o_after->stamp - orientation_stamp) < 1e-6 || o_after == observations.begin()) ? o_after : std::prev(o_after);
    if (o_before->stamp > orientation_stamp + 1e-6 || o_after->stamp - o_before->stamp > interpolation_gap)
    { result.orientation_valid = false; if (failure_reason) *failure_reason = 0; return true; }
  }
  const double o_interval = o_after->stamp - o_before->stamp;
  const double o_ratio = o_interval < 1e-6 ? 0.0 : (orientation_stamp - o_before->stamp) / o_interval;
  result.orientation_valid = o_before->orientation_valid && o_after->orientation_valid;
  if (result.orientation_valid)
    result.orientation = o_before->orientation.slerp(o_ratio, o_after->orientation);
  if (failure_reason) *failure_reason = 0;
  return true;
}

void RtkFusion::initializeGravity(double stamp, const V3D &gravity, const M3D &covariance)
{
  if (!std::isfinite(stamp) || !gravity.allFinite() || gravity.norm() < 9 || gravity.norm() > 11 ||
      !covariance.allFinite() || covariance.trace() <= 0 ||
      Eigen::SelfAdjointEigenSolver<M3D>(covariance).eigenvalues()(0) < -1e-10)
    throw std::runtime_error("Invalid stationary gravity initialization measurement");
  std::lock_guard<std::mutex> lock(mutex);
  if (initial_gravity_stamp >= 0 || !queue.empty())
    throw std::runtime_error("Gravity initialization must precede the first submitted scan, exactly once");
  initial_gravity_stamp = stamp;
  initial_gravity = gravity;
  initial_gravity_covariance = covariance;
}

void RtkFusion::submit(double stamp, double begin, const StatesGroup &state,
                       const PointCloudXYZI::Ptr &cloud, const gtsam::Matrix6 &lidar_information,
                       const PointCloudXYZRGB::Ptr &colored_world_cloud)
{
  // Interval bound: a LIVO update spans the time between two images; inside a fast-forwarded standstill the
  // player keeps only the camera key frames (GOP 2 s), so sanity-check at 5 s, not 2 s (which aborted the
  // first LIVO round with standstill skipping on truck29raw 20261013).
  if (!std::isfinite(stamp) || !std::isfinite(begin) || begin > stamp ||
      stamp - begin > 5 || !state.pos_end.allFinite() || !state.rot_end.allFinite())
    throw std::runtime_error("Invalid LIO pose or scan interval submitted to fusion");
  if (color_enabled && colored_world_cloud == nullptr)
    throw std::runtime_error("Camera-enabled RTK fusion requires an RGB cloud for every submitted frame");
  Frame frame;
  frame.stamp = stamp;
  frame.begin = begin;
  frame.raw = gtsam::Pose3(gtsam::Rot3(state.rot_end), state.pos_end);
  frame.cloud = std::make_shared<PointCloudXYZI>(*cloud);
  if (color_enabled)
    frame.color_cloud = std::make_shared<PointCloudXYZRGB>(*colored_world_cloud);
  frame.gravity = state.gravity;
  if (!lidar_information.allFinite()) throw std::runtime_error("Nonfinite native LiDAR information");
  frame.lidar_information = lidar_information;
  frame.frontend_speed = state.vel_end.norm();
  Observation reference;
  gtsam::Pose3 alignment;
  if (referenceAt(stamp, reference) && frontendAlignment(alignment))
    frame.frontend_residual = (alignment.transformFrom(V3D(state.pos_end + state.rot_end * lever)) - reference.position).norm();
  std::unique_lock<std::mutex> lock(mutex);
  if (queue.size() >= static_cast<std::size_t>(queue_capacity))
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "RTK processing queue full; applying backpressure, no mapping frames discarded");
  changed.wait(lock, [this] { return stopping || queue.size() < static_cast<std::size_t>(queue_capacity); });
  if (failure) std::rethrow_exception(failure);
  if (stopping) throw std::runtime_error("Cannot submit to stopped RTK backend");
  queue.push_back(std::move(frame));
  lock.unlock();
  changed.notify_all();
}

bool RtkFusion::fitSurface(const V3D &sum, const M3D &scatter,
                           std::size_t count, Plane &plane) const
{
  if (count < 8) return false;
  plane.center = sum / count;
  const M3D covariance = scatter / count - plane.center * plane.center.transpose();
  Eigen::SelfAdjointEigenSolver<M3D> eig(covariance);
  if (eig.info() != Eigen::Success) return false;
  // Collinearity creates a candidate, not an accepted constraint. Require
  // independent matching support and common rigid consistency in addGeometry.
  plane.linear = eig.eigenvalues()(1) < .01 && eig.eigenvalues()(2) >= .1 &&
      eig.eigenvalues()(1) <= 4 * plane_thickness * plane_thickness &&
      eig.eigenvalues()(1) <= .05 * eig.eigenvalues()(2);
  if (!plane.linear && (eig.eigenvalues()(1) < 0.01 ||
      eig.eigenvalues()(0) > plane_thickness * plane_thickness ||
      eig.eigenvalues()(0) > 0.12 * eig.eigenvalues()(1))) return false;
  plane.normal = eig.eigenvectors().col(plane.linear ? 2 : 0);
  plane.variance = std::max(0., eig.eigenvalues()(plane.linear ? 1 : 0));
  plane.sqrt_covariance = eig.eigenvectors() * eig.eigenvalues().cwiseMax(0.).cwiseSqrt().asDiagonal();
  plane.tangent = eig.eigenvectors().col(2);
  return true;
}

std::vector<RtkFusion::Plane> RtkFusion::extractPlanes(const Frame &frame) const
{
  struct Cluster
  {
    V3D sum = V3D::Zero();
    M3D scatter = M3D::Zero();
    std::vector<std::uint32_t> indices;
  };
  std::unordered_map<FusionVoxelKey, Cluster, FusionVoxelKeyHash> cells, fine_cells;
  cells.reserve(frame.cloud->size() / 4);
  fine_cells.reserve(frame.cloud->size() / 2);
  // Every fitted surface belongs to exactly this scan-end pose, just like
  // appendCloud(). A native-rigid multi-scan patch is NOT rigid after C(t)
  // changes: optimizing that frozen patch can increase actual map residuals.
  // Do not borrow previous scans unless their individual pose dependence is
  // represented in the factor. No such approximation is used here.
  for (std::size_t index = 0; index < frame.cloud->size(); ++index)
  {
    const auto &point = frame.cloud->points[index];
    const V3D p = lidar_rotation * V3D(point.x, point.y, point.z) + lidar_translation;
    if (!p.allFinite()) continue;
    const FusionVoxelKey key{static_cast<std::int64_t>(std::floor(p.x() / plane_voxel)),
        static_cast<std::int64_t>(std::floor(p.y() / plane_voxel)),
        static_cast<std::int64_t>(std::floor(p.z() / plane_voxel))};
    auto &cell = cells[key];
    cell.sum += p;
    cell.scatter.noalias() += p * p.transpose();
    cell.indices.push_back(static_cast<std::uint32_t>(index));
    const FusionVoxelKey fine_key{static_cast<std::int64_t>(std::floor(2 * p.x() / plane_voxel)),
        static_cast<std::int64_t>(std::floor(2 * p.y() / plane_voxel)),
        static_cast<std::int64_t>(std::floor(2 * p.z() / plane_voxel))};
    auto &fine = fine_cells[fine_key];
    fine.sum += p;
    fine.scatter.noalias() += p * p.transpose();
    fine.indices.push_back(static_cast<std::uint32_t>(index));
  }
  std::vector<std::pair<Plane, const Cluster *>> candidates;
  candidates.reserve(cells.size());
  const auto fit = [&](const Cluster &cell) {
    Plane plane;
    if (!fitSurface(cell.sum, cell.scatter, cell.indices.size(), plane)) return false;
    candidates.emplace_back(std::move(plane), &cell);
    return true;
  };
  for (const auto &entry : cells)
  {
    if (fit(entry.second)) continue;
    // Do not flatten a bend, step or a small structure into one large plane.
    // Use disjoint fine children only when their parent is nonplanar.
    for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) for (int z = 0; z < 2; ++z)
    {
      const auto found = fine_cells.find({2 * entry.first.x + x, 2 * entry.first.y + y, 2 * entry.first.z + z});
      if (found != fine_cells.end()) fit(found->second);
    }
  }
  // Spatially ordered stratified selection preserves near/far coverage and
  // makes results independent of unordered-map iteration order.
  std::sort(candidates.begin(), candidates.end(), [](const auto &left, const auto &right) {
    const auto &a = left.first;
    const auto &b = right.first;
    const double angle_a = std::atan2(a.center.y(), a.center.x());
    const double angle_b = std::atan2(b.center.y(), b.center.x());
    return angle_a == angle_b ? a.center.squaredNorm() < b.center.squaredNorm() : angle_a < angle_b;
  });
  std::vector<Plane> planes;
  const auto count = std::min(candidates.size(), static_cast<std::size_t>(max_planes));
  planes.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    auto &candidate = candidates[i * candidates.size() / count];
    auto &plane = candidate.first;
    plane.support.reserve(candidate.second->indices.size());
    for (auto index : candidate.second->indices)
    {
      const auto &point = frame.cloud->points[index];
      const V3D support = lidar_rotation * V3D(point.x, point.y, point.z) + lidar_translation;
      plane.support.push_back(support.cast<float>());
      plane.extendSupport(support);
    }
    planes.push_back(std::move(plane));
  }
  return planes;
}

std::size_t RtkFusion::addGeometry(std::size_t a, std::size_t b,
    gtsam::NonlinearFactorGraph &graph, GeometryKind kind)
{
  const auto &anchor = keyframes[a];
  const auto &current = keyframes[b];
  const gtsam::Pose3 relative = anchor.optimized.between(current.optimized);
  struct Match { const Plane *a; const Plane *b; };
  std::vector<Match> matches;
  // Repeated use of one patch is not independent evidence. Retain its
  // nearest compatible partner before applying the per-pair row budget.
  std::vector<Match> unique(anchor.planes.size(), {nullptr, nullptr});
  std::vector<double> nearest(anchor.planes.size(), match_distance * match_distance);
  for (const Plane &plane : current.planes)
  {
    const V3D point = relative.transformFrom(plane.center);
    const V3D normal = relative.rotation().rotate(plane.normal);
    const FusionVoxelKey cell{static_cast<std::int64_t>(std::floor(point.x() / plane_voxel)),
        static_cast<std::int64_t>(std::floor(point.y() / plane_voxel)),
        static_cast<std::int64_t>(std::floor(point.z() / plane_voxel))};
    const Plane *best = nullptr;
    double best_score = match_distance * match_distance;
    for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z)
    {
      const auto found = anchor.cells.find({cell.x + x, cell.y + y, cell.z + z});
      if (found == anchor.cells.end()) continue;
      for (std::size_t index : found->second)
      {
        const auto &candidate = anchor.planes[index];
        if (candidate.linear != plane.linear) continue;
        const V3D delta = point - candidate.center;
        const double score = delta.squaredNorm(), agreement = normal.dot(candidate.normal);
        if (score >= best_score || std::abs(agreement) <= .985) continue;
        const double consistency = 2.5 * std::sqrt(candidate.variance + plane.variance) + .5 * plane_thickness;
        if (candidate.linear)
        {
          const M3D projector = M3D::Identity() - normal * normal.transpose();
          if (((candidate.projector() - projector) * delta).norm() > consistency) continue;
        }
        else if (std::abs(candidate.normal.dot(delta) -
            std::copysign(1., agreement) * normal.dot(delta)) > consistency) continue;
        best = &candidate;
        best_score = score;
      }
    }
    if (best)
    {
      const auto index = static_cast<std::size_t>(best - anchor.planes.data());
      if (best_score < nearest[index])
      {
        nearest[index] = best_score;
        unique[index] = {best, &plane};
      }
    }
  }
  for (const auto &match : unique) if (match.a) matches.push_back(match);
  if (matches.size() < 12) return 0;
  const auto sigma = [&](const Match &m) {
    return std::sqrt(2 * plane_noise_floor * plane_noise_floor + m.a->variance + m.b->variance);
  };
  const auto measurement = [&](const Match &m) {
    return ScanPlaneBatchFactor::Constraint{m.a->normal, m.a->center, m.b->center,
        sigma(m), m.a->linear, m.b->sqrt_covariance};
  };
  const auto cost = [&](const gtsam::Pose3 &pose) {
    double sum = 0;
    for (const auto &m : matches)
    {
      const auto surface = measurement(m);
      const double value = surface.physicalResidual(pose).norm() / surface.sigma;
      sum += value <= 1.5 ? .5 * value * value : 1.5 * (value - .75);
    }
    return sum;
  };
  // A common registration validates associations; its fitted transform is
  // never substituted for the measurements or turned into a full-rank prior.
  const auto register_matches = [&](gtsam::Pose3 &registered) {
    gtsam::Vector6 scaling;
    scaling << 1. / 30, 1. / 30, 1. / 30, 1, 1, 1;
    for (int iteration = 0; iteration < 8; ++iteration)
    {
      gtsam::Matrix6 information = gtsam::Matrix6::Zero();
      gtsam::Vector6 gradient = gtsam::Vector6::Zero();
      for (const auto &m : matches)
      {
        const double stddev = sigma(m);
        const auto projector = m.a->projector();
        const Eigen::Matrix<double, 3, 4> residuals = measurement(m).physicalResidual(registered) / stddev;
        const double weight = residuals.norm() <= 1.5 ? 1 : 1.5 / residuals.norm();
        for (int part = 0; part < 4; ++part)
        {
          const V3D point = part ? V3D(registered.rotation().rotate(m.b->sqrt_covariance.col(part - 1))) :
              registered.transformFrom(m.b->center);
          Eigen::Matrix<double, 3, 6> jacobian;
          jacobian.leftCols<3>() = -projector * gtsam::skewSymmetric(point);
          jacobian.rightCols<3>() = part ? M3D::Zero().eval() : projector;
          jacobian = (jacobian * scaling.asDiagonal() / stddev).eval();
          information.noalias() += weight * jacobian.transpose() * jacobian;
          gradient.noalias() += weight * jacobian.transpose() * residuals.col(part);
        }
      }
      Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(information);
      if (eig.info() != Eigen::Success) return false;
      gtsam::Vector6 step = gtsam::Vector6::Zero();
      for (int i = 0; i < 6; ++i)
        if (eig.eigenvalues()(i) > std::max(1e-9, 1e-6 * eig.eigenvalues()(5)))
          step -= eig.eigenvectors().col(i) *
              (eig.eigenvectors().col(i).dot(gradient) / eig.eigenvalues()(i));
      // A road-only registration cannot invent yaw/horizontal information.
      step = step.cwiseProduct(scaling);
      step /= std::max({1., step.head<3>().norm() / .02, step.tail<3>().norm() / .3});
      const double before = cost(registered);
      bool accepted = false;
      for (int trial = 0; trial < 5; ++trial)
      {
        const auto proposal = gtsam::Pose3::Expmap(step) * registered;
        if (cost(proposal) <= before)
        { registered = proposal; accepted = true; break; }
        step *= .5;
      }
      if (!accepted || (step.head<3>().norm() < 1e-6 && step.tail<3>().norm() < 1e-4)) break;
    }
    return true;
  };
  gtsam::Pose3 registered = relative;
  if (!register_matches(registered)) return 0;
  // Independent voxel grids sample different parts of ruts, slopes and
  // occlusion boundaries. Refit BOTH surfaces on the actually shared
  // footprint. A nearest individual return is not a substitute for that
  // shared support and makes repeated, biased matches look independent.
  // Crop tangential extent only, retaining the normal error to be optimized.
  const auto coarse_count = matches.size();
  std::size_t source_count = 0, shared_count = 0;
  std::vector<std::pair<Plane, Plane>> shared;
  shared.reserve(coarse_count);
  const auto refit = [&](const Plane &source, const Plane &target,
                         const gtsam::Pose3 &source_to_target, Plane &fitted) {
    V3D sum = V3D::Zero();
    M3D scatter = M3D::Zero();
    std::size_t count = 0;
    source_count += source.support.size();
    for (const auto &stored : source.support)
    {
      const V3D point = stored.cast<double>();
      if (!target.covers(source_to_target.transformFrom(point))) continue;
      sum += point;
      scatter.noalias() += point * point.transpose();
      ++count;
    }
    shared_count += count;
    return fitSurface(sum, scatter, count, fitted) && fitted.linear == source.linear;
  };
  const auto inverse_registered = registered.inverse();
  for (const auto &m : matches)
  {
    Plane cropped_a, cropped_b;
    const bool valid_a = refit(*m.a, *m.b, inverse_registered, cropped_a);
    const bool valid_b = refit(*m.b, *m.a, registered, cropped_b);
    if (!valid_a || !valid_b || std::abs(cropped_a.normal.dot(
        registered.rotation().rotate(cropped_b.normal))) <= .985) continue;
    shared.emplace_back(std::move(cropped_a), std::move(cropped_b));
  }
  matches.clear();
  for (const auto &pair : shared) matches.push_back({&pair.first, &pair.second});
  const double shared_fraction = static_cast<double>(shared_count) / std::max<std::size_t>(1, source_count);
  if (matches.size() < 12)
  {
    association_log << std::setprecision(17) << anchor.stamp << ',' << current.stamp << ','
        << matches.size() << ",0,nan,nan,nan,nan,nan," << coarse_count << ',' << shared_fraction << ",nan,nan,0\n";
    return 0;
  }
  if (!register_matches(registered)) return 0;
  double before_squared = 0, consistent_squared = 0, rejected_squared = 0;
  std::vector<Match> consistent;
  consistent.reserve(matches.size());
  for (const auto &m : matches)
  {
    const double before = measurement(m).physicalResidual(relative).norm();
    const double after = measurement(m).physicalResidual(registered).norm();
    before_squared += before * before;
    if (std::abs(after) <= 2.5 * sigma(m))
    {
      consistent.push_back(m);
      consistent_squared += after * after;
    }
    else rejected_squared += after * after;
  }
  const auto change = relative.between(registered);
  const double translation_change = change.translation().norm();
  const double rotation_change = gtsam::Rot3::Logmap(change.rotation()).norm();
  const auto matched_count = matches.size(), consistent_count = consistent.size();
  double baseline_difference = std::numeric_limits<double>::quiet_NaN();
  double baseline_sigma = std::numeric_limits<double>::quiet_NaN();
  const auto record = [&](bool accepted) {
    association_log << std::setprecision(17) << anchor.stamp << ',' << current.stamp << ','
        << matched_count << ',' << consistent_count << ',' << std::sqrt(before_squared / matched_count) << ','
        << std::sqrt(consistent_squared / std::max<std::size_t>(1, consistent_count)) << ','
        << std::sqrt(rejected_squared / std::max<std::size_t>(1, matched_count - consistent_count)) << ','
        << translation_change << ',' << rotation_change * 180 / M_PI << ','
        << coarse_count << ',' << shared_fraction << ',' << baseline_difference << ',' << baseline_sigma << ',' << accepted << '\n';
  };
  if (consistent.size() < 12 || consistent.size() < .5 * matches.size() ||
      translation_change > match_distance || rotation_change > 3 * M_PI / 180) { record(false); return 0; }
  matches = std::move(consistent);
  // A few verified distant surfaces can observe tilt or yaw without
  // observing all six axes. Do not reject these solely for having fewer
  // than 40 patches or a rank-deficient normal scatter. Raw projected rows
  // retain their actual nullspace, unlike a fabricated full-pose constraint.
  const std::size_t count = std::min(matches.size(), static_cast<std::size_t>(max_pair_factors));
  std::array<std::vector<std::size_t>, 4> directions;
  for (std::size_t i = 0; i < matches.size(); ++i)
  {
    Eigen::Index axis;
    matches[i].a->normal.cwiseAbs().maxCoeff(&axis);
    if (matches[i].a->linear) axis = 3;
    directions[axis].push_back(i);
  }
  // Allocate the per-pair budget in proportion to the matched direction mix
  // instead of an equal round-robin. An edge dominated by street-parallel
  // walls and pole-like linears keeps its yaw evidence instead of donating
  // slots to a small ground stratum; every present class still retains at
  // least one stratum while its share allows. Repeated greedy largest-
  // deficit allocation is deterministic and bounded by the budget.
  std::array<std::size_t, 4> quotas{};
  const double direction_total = static_cast<double>(matches.size());
  for (std::size_t selected = 0; selected < count;)
  {
    int best = -1;
    double best_deficit = 0;
    for (int axis = 0; axis < 4; ++axis)
    {
      if (quotas[axis] >= directions[axis].size()) continue;
      const double deficit =
          static_cast<double>(directions[axis].size()) / direction_total * count - quotas[axis];
      if (deficit > best_deficit) { best_deficit = deficit; best = axis; }
    }
    if (best < 0) break;
    ++quotas[best];
    ++selected;
  }
  std::vector<ScanPlaneBatchFactor::Constraint> constraints;
  constraints.reserve(count);
  for (int axis = 0; axis < 4; ++axis) for (std::size_t i = 0; i < quotas[axis]; ++i)
  {
    // Select the centre of each direction stratum. With a single rare
    // vertical/line patch, selecting index zero biased the only cue.
    const auto &m = matches[directions[axis][(2 * i + 1) * directions[axis].size() /
        (2 * quotas[axis])]];
    auto constraint = measurement(m);
    // Adjacent scans reuse surfaces and are correlated with native LIO.
    // Share a single geometry information budget across these neighbours;
    // do not claim N independent measurements of the same local map.
    if (kind == GeometryKind::Local) constraint.sigma *= std::sqrt(local_neighbors);
    constraints.push_back(std::move(constraint));
  }
  auto factor = std::make_shared<ScanPlaneBatchFactor>(gtsam::Symbol('x', a), gtsam::Symbol('x', b),
                                                      std::move(constraints));
  if (anchor.has_position && current.has_position)
  {
    // Validate a rigid-registration invariant, not the graph's current yaw
    // or a native estimate that can drift. The rear-axle baseline length
    // must agree with TWO qualified fixes within measured uncertainty.
    // Weak/unobserved baseline directions receive infinite uncertainty,
    // never an artificial positive information floor or a hard RTK pose.
    const V3D baseline = registered.transformFrom(lever) - lever;
    const V3D reference = current.reference - anchor.reference;
    if (baseline.norm() > 1e-6 && reference.norm() > 1e-6)
    {
      gtsam::Vector6 units; units << 1. / 30, 1. / 30, 1. / 30, 1, 1, 1;
      gtsam::Matrix jacobian;
      factor->evaluateError(gtsam::Pose3(), registered, nullptr, &jacobian);
      jacobian = (jacobian * units.asDiagonal()).eval();
      const Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> spectrum(jacobian.transpose() * jacobian);
      if (spectrum.info() != Eigen::Success) { record(false); return 0; }
      Eigen::Matrix<double, 1, 6> derivative;
      derivative.head<3>() = -baseline.normalized().transpose() * registered.rotation().matrix() *
          gtsam::skewSymmetric(lever);
      derivative.tail<3>() = baseline.normalized().transpose() * registered.rotation().matrix();
      derivative = (derivative * units.asDiagonal()).eval();
      double variance = reference.normalized().dot(
          (anchor.reference_covariance + current.reference_covariance) * reference.normalized());
      for (int i = 0; i < 6; ++i)
      {
        const double response = derivative.dot(spectrum.eigenvectors().col(i));
        if (spectrum.eigenvalues()(i) > std::max(1e-9, 1e-6 * spectrum.eigenvalues()(5)))
          variance += response * response / spectrum.eigenvalues()(i);
        else if (std::abs(response) > 1e-6) variance = std::numeric_limits<double>::infinity();
      }
      baseline_difference = baseline.norm() - reference.norm();
      baseline_sigma = std::sqrt(variance);
      if (std::abs(baseline_difference) > 3 * baseline_sigma) { record(false); return 0; }
    }
  }
  record(true);
  new_links.push_back({a, b, graph.size(), kind});
  graph.push_back(factor);
  return count;
}

bool RtkFusion::frontendAlignment(gtsam::Pose3 &alignment)
{
  std::lock_guard<std::mutex> lock(mutex);
  alignment = frontend_to_enu;
  return frontend_alignment_known;
}

bool RtkFusion::frontendReferenceAt(double stamp, const StatesGroup &state, Observation &result)
{
  if (!referenceAt(stamp, result)) return false;
  std::lock_guard<std::mutex> lock(mutex);
  if (!frontend_alignment_known)
  {
    if (!result.orientation_valid) return false;
    // Preserve the gravity direction established before mapping. A noisy INS
    // roll/pitch at reacquisition must not tilt the entire historical gravity.
    const M3D level = Eigen::Quaterniond::FromTwoVectors(local_gravity, V3D(0, 0, -G_m_s2)).toRotationMatrix();
    const V3D predicted_forward = level * state.rot_end.col(1);
    const V3D measured_forward = result.orientation.toRotationMatrix().col(1);
    if (predicted_forward.head<2>().norm() < 0.1 || measured_forward.head<2>().norm() < 0.1) return false;
    const double yaw = std::atan2(measured_forward.y(), measured_forward.x()) -
        std::atan2(predicted_forward.y(), predicted_forward.x());
    const gtsam::Rot3 rotation(gtsam::Rot3::Rz(yaw).matrix() * level);
    frontend_to_enu = gtsam::Pose3(rotation,
        result.position - rotation.rotate(state.pos_end + state.rot_end * lever));
    frontend_alignment_known = true;
    RCLCPP_INFO(node->get_logger(), "First stable fixed RTK anchors local LIO without changing its voxel map");
  }
  const M3D rotation = frontend_to_enu.rotation().matrix().transpose();
  result.position = frontend_to_enu.transformTo(result.position);
  result.covariance = rotation * result.covariance * rotation.transpose();
  return true;
}

void RtkFusion::optimizeKeyframe(const Frame &frame)
{
  auto stage = std::chrono::steady_clock::now();
  const auto timing = [&stage](double &result) {
    const auto now = std::chrono::steady_clock::now();
    result = std::chrono::duration<double, std::milli>(now - stage).count();
    stage = now;
  };
  new_links.clear();
  if (keyframes.empty()) local_gravity = initial_gravity_stamp >= 0 ? initial_gravity : frame.gravity;
  Observation seed;
  const bool have_seed = observations_enabled && referenceAt(frame.stamp, seed);
  if (local_frontend && have_seed)
  {
    StatesGroup state;
    state.rot_end = frame.raw.rotation().matrix();
    state.pos_end = frame.raw.translation();
    Observation unused;
    frontendReferenceAt(frame.stamp, state, unused);
  }
  gtsam::Pose3 alignment;
  if (!graph_in_enu && frontendAlignment(alignment))
  {
    // All pre-fix factors are relative except the initial gauge. Transform
    // that gauge and all values once; keep every historical relative factor.
    if (!keyframes.empty())
    {
      auto graph = solver.getFactorsUnsafe();
      const auto prior = std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(graph.at(0));
      if (!prior) throw std::runtime_error("Missing initial gauge for delayed ENU alignment");
      graph.at(0) = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', 0), alignment * prior->prior(), prior->noiseModel());
      for (auto &pose : last_logged_poses) pose = alignment * pose;
      gtsam::Values values;
      for (std::size_t i = 0; i < keyframes.size(); ++i)
      {
        keyframes[i].optimized = alignment * keyframes[i].optimized;
        values.insert(gtsam::Symbol('x', i), keyframes[i].optimized);
      }
      solver = gtsam::ISAM2(solverParameters());
      const auto rebuilt = solver.update(graph, values);
      for (auto &link : geometry_links)
        if (link.factor_index != std::numeric_limits<std::size_t>::max())
          link.factor_index = rebuilt.newFactorsIndices.at(link.factor_index);
      updatePoses();
      spatial_keys.clear();
      surface_keys.clear();
    }
    graph_in_enu = true;
  }
  const std::size_t index = keyframes.size();
  Keyframe current;
  current.stamp = frame.stamp;
  current.raw = frame.raw;
  current.path = keyframes.empty() ? 0.0 :
      keyframes.back().path + (frame.raw.translation() - keyframes.back().raw.translation()).norm();
  current.angular_observability = nativeAngularObservability(frame.lidar_information);
  if (index == 0)
  {
    current.optimized = graph_in_enu ? alignment * frame.raw : frame.raw;
    if (have_seed && graph_in_enu)
    {
      // Finite gauge only: no continuous INS attitude prior. Subsequent
      // initial guesses inherit native relative motion, not RTK positions.
      const auto rotation = seed.orientation_valid ? gtsam::Rot3(seed.orientation) : current.optimized.rotation();
      current.optimized = gtsam::Pose3(rotation, seed.position - rotation.rotate(lever));
    }
    if (initial_gravity_stamp >= 0 && graph_in_enu)
    {
      // The native world need not be level even after INS-based alignment.
      // Transfer the stationary gravity measurement instead of importing the
      // receiver's uncertain roll/pitch or allowing an arbitrary world tilt.
      const auto level = gtsam::Rot3(Eigen::Quaterniond::FromTwoVectors(initial_gravity, V3D(0, 0, -1)));
      const auto rotation = (local_frontend ? alignment.rotation() : level) * frame.raw.rotation();
      current.optimized = gtsam::Pose3(rotation, have_seed ?
          V3D(seed.position - rotation.rotate(lever)) : current.optimized.translation());
    }
  }
  else
    current.optimized = keyframes.back().optimized * keyframes.back().raw.between(frame.raw);
  const bool geometry_anchor = geometry_anchors.empty() || frame.stamp -
      keyframes[geometry_anchors.back()].stamp >= geometry_anchor_interval - 1e-6;
  // Native deskew stays immutable in the physical sensor frame. Global
  // pose updates cannot silently change surfaces or repeat IMU integration.
  if (geometry_anchor)
  {
    current.planes = extractPlanes(frame);
    for (std::size_t i = 0; i < current.planes.size(); ++i)
    {
      const V3D &p = current.planes[i].center;
      current.cells[{static_cast<std::int64_t>(std::floor(p.x() / plane_voxel)),
                     static_cast<std::int64_t>(std::floor(p.y() / plane_voxel)),
                     static_cast<std::int64_t>(std::floor(p.z() / plane_voxel))}].push_back(
                         static_cast<std::uint32_t>(i));
    }
  }
  keyframes.push_back(std::move(current));
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  const gtsam::Symbol key('x', index);
  initial.insert(key, keyframes.back().optimized);
  if (index == 0)
  {
    gtsam::Vector6 sigmas;
    sigmas << 0.01, 0.01, 0.05, 10, 10, 10;
    gtsam::Matrix6 covariance = sigmas.cwiseProduct(sigmas).asDiagonal();
    if (initial_gravity_stamp >= 0)
    {
      const M3D rotation = frame.raw.rotation().matrix();
      const V3D gravity_body = rotation.transpose() * initial_gravity;
      M3D angular = gravityOrientationCovariance(gravity_body,
          rotation.transpose() * initial_gravity_covariance * rotation, .05 * .05);
      const M3D tangent = M3D::Identity() - gravity_body.normalized() * gravity_body.normalized().transpose();
      const M3D transfer_variance =
          (motion_rotation_sigma.cwiseProduct(motion_rotation_sigma) * std::max(0., frame.stamp - initial_gravity_stamp)).asDiagonal();
      angular += tangent * transfer_variance * tangent;
      covariance.topLeftCorner<3, 3>() = angular;
      RCLCPP_INFO(node->get_logger(), "Initial stationary-gravity prior: finite tilt covariance trace %.6g rad^2; no continuing attitude factors",
          (tangent * angular * tangent).trace());
    }
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        key, keyframes.back().optimized, gtsam::noiseModel::Gaussian::Covariance(covariance));
  }
  // Late quality/position callbacks can fill an earlier keyframe. Never add
  // the same sample/factor twice and never extrapolate beyond the INS bracket.
  const std::size_t first_pending = index > 100 ? index - 100 : 0;
  for (std::size_t i = first_pending; i <= index; ++i)
  {
    if (keyframes[i].has_position || !graph_in_enu || !observations_enabled) continue;
    Observation reference;
    if (!referenceAt(keyframes[i].stamp, reference, &keyframes[i].reference_failure))
    { ++missing_positions; continue; }
    // Temporal quadrature of the same position likelihood. A short interval
    // carries proportionally less information, not zero information. It
    // handles variable node rates and the final partial interval uniformly.
    // Cap a long interval at one actual observation; outages never accrue
    // fictitious GNSS information. Covariance and Huber threshold are scaled
    // together so the ENTIRE robust cost is multiplied by weight.
    const double weight = i == 0 ? 1 : std::min(1.,
        (keyframes[i].stamp - keyframes[i - 1].stamp) / position_factor_interval);
    reference.covariance /= weight;
    const auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_delta * std::sqrt(weight)),
        gtsam::noiseModel::Gaussian::Covariance(reference.covariance));
    graph.emplace_shared<gtsam::GPSFactorArm>(gtsam::Symbol('x', i),
        reference.position, lever, noise);
    keyframes[i].reference = reference.position;
    keyframes[i].reference_covariance = reference.covariance;
    keyframes[i].reference_covariance_source = reference.covariance_source;
    keyframes[i].has_position = true;
    keyframes[i].position_information_weight = weight;
    ++accepted_positions;
  }
  // Bounded absolute attitude from the qualified INS solution. Same quality
  // gate, timing and temporal weight as the rear-axle position; one factor per
  // keyframe. The installation misalignment is a single shared variable with
  // a weak prior, so a constant mounting offset never bends the trajectory.
  if (attitude_enabled && observations_enabled)
    for (std::size_t i = first_pending; i <= index; ++i)
    {
      if (keyframes[i].has_attitude || !graph_in_enu) continue;
      // Native speed over the preceding keyframe interval gates the INS
      // heading: stationary/slow heading is gyro-held, not GNSS-aligned.
      if (i == 0) continue;
      const double interval = keyframes[i].stamp - keyframes[i - 1].stamp;
      const double speed = interval > 0 ?
          (keyframes[i].raw.translation() - keyframes[i - 1].raw.translation()).norm() / interval : 0;
      if (speed >= 0.5 && motion_start_stamp < 0) motion_start_stamp = keyframes[i].stamp;
      if (speed < attitude_min_speed || motion_start_stamp < 0 ||
          keyframes[i].stamp - motion_start_stamp < attitude_motion_age) continue;
      Observation reference;
      if (!referenceAt(keyframes[i].stamp, reference) || !reference.orientation_valid) continue;
      const gtsam::Key misalignment_key = gtsam::Symbol('m', 0);
      if (!misalignment_initialized)
      {
        initial.insert(misalignment_key, gtsam::Rot3());
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(misalignment_key, gtsam::Rot3(),
            gtsam::noiseModel::Diagonal::Sigmas(misalignment_prior_sigma));
        misalignment_initialized = true;
      }
      const double weight = i == 0 ? 1 : std::min(1.,
          (keyframes[i].stamp - keyframes[i - 1].stamp) / position_factor_interval);
      const auto noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(attitude_huber * std::sqrt(weight)),
          gtsam::noiseModel::Diagonal::Sigmas(attitude_sigma / std::sqrt(weight)));
      keyframes[i].reference_rotation = gtsam::Rot3(reference.orientation);
      graph.emplace_shared<InsAttitudeFactor>(gtsam::Symbol('x', i), misalignment_key,
          keyframes[i].reference_rotation, noise);
      keyframes[i].has_attitude = true;
      ++accepted_attitudes;
    }
  if (index)
  {
    const auto &previous = keyframes[index - 1];
    const auto &current = keyframes[index];
    const double duration = current.stamp - previous.stamp;
    const auto measured = previous.raw.between(current.raw);
    const auto sigmas = motionSigmas(duration);

    graph.emplace_shared<NativeMotionFactor>(gtsam::Symbol('x', index - 1), key,
        measured, sigmas, motion_huber_rate * std::sqrt(duration));
    if (knot_parked && standstill_hold_rotation > 0 && standstill_hold_translation > 0)
    {
      gtsam::Vector6 hold;
      hold << gtsam::Vector3::Constant(standstill_hold_rotation), gtsam::Vector3::Constant(standstill_hold_translation);
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', index - 1), key, measured,
          gtsam::noiseModel::Diagonal::Sigmas(hold));
    }
  }
  initializePose(graph, initial);
  timing(key_timings[0]);
  // A robust native increment is a motion regularizer, not a guarantee of
  // local map consistency. Retain measured common-support geometry between
  // nearby scans, with a bounded shared information budget and its real
  // nullspace, so the global correction field stays locally consistent.
  if (geometry_anchor)
    for (std::size_t n = 0; n < std::min<std::size_t>(local_neighbors, geometry_anchors.size()); ++n)
      addGeometry(geometry_anchors[geometry_anchors.size() - 1 - n], index, graph, GeometryKind::Local);
  if (geometry_anchor)
  {
    const V3D position = keyframes.back().optimized.translation();
    const FusionVoxelKey cell{static_cast<std::int64_t>(std::floor(position.x() / revisit_radius)),
        static_cast<std::int64_t>(std::floor(position.y() / revisit_radius)),
        static_cast<std::int64_t>(std::floor(position.z() / revisit_radius))};
    std::vector<std::pair<double, std::size_t>> candidates;
    for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z)
    {
      const auto found = spatial_keys.find({cell.x + x, cell.y + y, cell.z + z});
      if (found == spatial_keys.end()) continue;
      for (std::size_t previous : found->second)
      {
        if (frame.stamp - keyframes[previous].stamp < revisit_age ||
            index - previous <= static_cast<std::size_t>(local_neighbors) ||
            keyframes.back().path - keyframes[previous].path < revisit_min_path) continue;
        const double distance = (keyframes[previous].optimized.translation() - position).norm();
        if (distance <= revisit_radius) candidates.emplace_back(distance, previous);
      }
    }
    std::sort(candidates.begin(), candidates.end());
    for (std::size_t i = 0; i < std::min<std::size_t>(2, candidates.size()); ++i)
      addGeometry(candidates[i].second, index, graph, GeometryKind::Revisit);

    // Select by ACTUAL observed surfaces, not prescribed vehicle distances.
    // Two scans 50-60 m apart may see the same object; the old 10/35 m
    // nearest-body rule could never connect them. Occupancy only proposes
    // candidates: independent surface fitting/common support/registration
    // remain mandatory, with exactly the original residual and noise.
    std::unordered_set<FusionVoxelKey, FusionVoxelKeyHash> occupied;
    for (const auto &plane : keyframes.back().planes)
    {
      const V3D p = keyframes.back().optimized.transformFrom(plane.center) / (2 * plane_voxel);
      occupied.insert({static_cast<std::int64_t>(std::floor(p.x())),
          static_cast<std::int64_t>(std::floor(p.y())), static_cast<std::int64_t>(std::floor(p.z()))});
    }
    std::vector<int> counts(index, 0), visited(index, -1);
    int cell_number = 0;
    for (const auto &cell : occupied)
    {
      for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z)
      {
        const auto found = surface_keys.find({cell.x + x, cell.y + y, cell.z + z});
        if (found == surface_keys.end()) continue;
        for (const auto i : found->second)
          if (visited[i] != cell_number) { ++counts[i]; visited[i] = cell_number; }
      }
      ++cell_number;
    }
    // Restore the last stable geometry policy: one best-overlap edge and one
    // long-baseline overlap edge. The later 20 m-only policy removed useful
    // observable constraints and was rejected by full-run map/trajectory data.
    for (bool long_baseline : {false, true})
    {
      std::vector<std::pair<double, std::size_t>> ranked;
      for (auto i : geometry_anchors)
      {
        if (!counts[i] || index - i <= static_cast<std::size_t>(local_neighbors)) continue;
        const double baseline = (keyframes[i].optimized.translation() - position).norm();
        if (baseline < key_translation || std::any_of(new_links.begin(), new_links.end(),
            [i](const GeometryLink &link) { return link.a == i; })) continue;
        ranked.emplace_back(counts[i] * (long_baseline ? baseline : 1.), i);
      }
      std::sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
        return a.first == b.first ? a.second < b.second : a.first > b.first;
      });
      for (std::size_t attempt = 0; attempt < std::min<std::size_t>(3, ranked.size()); ++attempt)
        if (addGeometry(ranked[attempt].second, index, graph, GeometryKind::Overlap)) break;
    }
    geometry_anchors.push_back(index);
  }


  timing(key_timings[1]);
  const auto result = solver.update(graph, initial);
  last_relinearized = last_reeliminated = last_cliques = 0;
  accumulateSolverStats(result);
  for (auto &link : new_links)
  {
    link.factor_index = result.newFactorsIndices.at(link.factor_index);
    geometry_links.push_back(link);
  }
  new_links.clear();
  accumulateSolverStats(solver.update());
  updatePoses();
  timing(key_timings[2]);
  // Refine the current fixed graph before publication, including history.
  // A stable newest pose does NOT imply that earlier poses have converged.
  // Keep bounded incremental work; no whole-graph batch LM on every scan.
  std::vector<gtsam::Pose3> previous(keyframes.size());
  for (int iteration = 0; iteration < 6; ++iteration)
  {
    for (std::size_t i = 0; i < keyframes.size(); ++i) previous[i] = keyframes[i].optimized;
    accumulateSolverStats(solver.update());
    updatePoses();
    refinement_translation = refinement_rotation = 0;
    for (std::size_t i = 0; i < keyframes.size(); ++i)
    {
      const auto change = previous[i].between(keyframes[i].optimized);
      refinement_translation = std::max(refinement_translation, change.translation().norm());
      refinement_rotation = std::max(refinement_rotation, gtsam::Rot3::Logmap(change.rotation()).norm());
    }
    refinement_steps = iteration + 1;
    if (refinement_translation < .002 && refinement_rotation < .0001) break;
  }
  timing(key_timings[3]);
  // The full-graph cost breakdown is O(factors): every online_log_interval keyframes only (the final
  // fixed-graph phases are always logged).
  if (online_log_interval <= 1 || index % static_cast<std::size_t>(online_log_interval) == 0)
    logOptimization("online", 0, std::numeric_limits<double>::quiet_NaN(), solver.getFactorsUnsafe().error(estimate));
  // Candidate indices only PROPOSE covisible/revisit partners (8 m / 4 m cells; every candidate is then
  // verified by registration). Re-indexing every anchor and plane at every knot is O(keys x planes) and
  // dominated long surveys; rebuild from the solved poses every index_rebuild_keys knots (or after a large
  // refinement step) and insert new anchors immediately.
  ++keys_since_index_rebuild;
  if (index_rebuild_keys <= 1 || keys_since_index_rebuild >= static_cast<std::size_t>(index_rebuild_keys) ||
      refinement_translation > 0.05 || refinement_rotation > 0.1 * M_PI / 180)
  {
    keys_since_index_rebuild = 0;
    spatial_keys.clear();
    // Reuse cell/vector storage during reindexing rather than allocating all
    // observed-surface postings at every knot. Empty old cells carry no votes.
    for (auto &cell : surface_keys) cell.second.clear();
    for (std::size_t i : geometry_anchors) indexAnchor(i);
  }
  else if (geometry_anchor)
    indexAnchor(index);
}

gtsam::Pose3 RtkFusion::poseAt(const Frame &frame) const
{
  return correctionAt(frame.stamp) * frame.raw;
}

void RtkFusion::appendCloud(const Frame &frame, pcl::PointCloud<pcl::PointXYZI> &output) const
{
  // Callers append up to local_map_frames scans: grow geometrically (an exact reserve per scan re-copied the
  // whole snapshot for every appended scan, ~0.5 GB of copies per RViz local-map publish).
  if (output.points.capacity() < output.size() + frame.cloud->size())
    output.reserve(std::max(output.size() + frame.cloud->size(), 2 * output.points.capacity()));
  const M3D native_rotation = frame.raw.rotation().matrix() * lidar_rotation;
  const V3D native_translation = frame.raw.transformFrom(lidar_translation);
  // FAST-LIVO has already deskewed each return to the scan-end body frame.
  // Transforming that point by the native scan-end pose recovers the static
  // native-world observation made at its acquisition time. Apply only the
  // interpolated LOW-FREQUENCY global correction at that point time. This is
  // not a second IMU deskew: native point geometry and the original timestamp
  // remain immutable. A scan-end-only correction is invalid after native LIO
  // velocity degenerates, because it preserves the resulting metre-scale
  // intra-scan world error in the final map.
  // Returns of one firing share their time offset, hence their correction (as in exportFrame): the per-return
  // correctionAt made the RViz local-map snapshot cost up to ~1 s of backend time per publish.
  std::unordered_map<float, gtsam::Pose3> corrections;
  corrections.reserve(4096);
  for (const PointType &point : *frame.cloud)
  {
    double point_stamp = frame.begin + point.curvature / 1000.0;
    if (!std::isfinite(point_stamp) || point_stamp < frame.begin - 1e-3 ||
        point_stamp > frame.stamp + 1e-3)
      throw std::runtime_error("Archived LiDAR point timestamp is outside its submitted frame interval");
    point_stamp = std::max(frame.begin, std::min(frame.stamp, point_stamp));
    const V3D native = native_rotation * V3D(point.x, point.y, point.z) + native_translation;
    auto found = corrections.find(point.curvature);
    if (found == corrections.end()) found = corrections.emplace(point.curvature, correctionAt(point_stamp)).first;
    const V3D p = found->second.transformFrom(native);
    pcl::PointXYZI result;
    result.x = p.x(); result.y = p.y(); result.z = p.z(); result.intensity = point.intensity;
    output.push_back(result);
  }
}

void RtkFusion::appendDisplayCloud(const Frame &frame, PointCloudXYZRGB &output) const
{
  if (!frame.color_cloud)
    throw std::runtime_error("RGB frame data missing from camera-enabled RTK cache");
  if (output.points.capacity() < output.size() + frame.color_cloud->size())
    output.reserve(std::max(output.size() + frame.color_cloud->size(), 2 * output.points.capacity()));
  // color_cloud is already in the native frontend world frame. Applying
  // C(t)=X_global*X_frontend^-1 preserves the exact camera association while
  // moving the colored surface with the same global correction as geometry.
  const auto correction = correctionAt(frame.stamp);
  for (const PointTypeRGB &point : *frame.color_cloud)
  {
    const V3D p = correction.transformFrom(V3D(point.x, point.y, point.z));
    PointTypeRGB result = point;
    result.x = p.x(); result.y = p.y(); result.z = p.z();
    output.push_back(result);
  }
}

void RtkFusion::process(Frame frame)
{
  const auto start = std::chrono::steady_clock::now();
  key_timings.fill(0);
  last_relinearized = last_reeliminated = last_cliques = 0;
  refinement_steps = 0;
  refinement_translation = refinement_rotation = 0;
  if (!frames.empty() && frame.stamp <= frames.back().stamp)
    throw std::runtime_error("Fusion frames must have strictly increasing timestamps");
  Observation display_reference;
  if (referenceAt(frame.stamp, display_reference)) frame.reference_imu = display_reference.imu_position;
  if (!frames.empty() && !frames.back().reference_imu.allFinite() &&
      referenceAt(frames.back().stamp, display_reference))
    frames.back().reference_imu = display_reference.imu_position;
  frame.count = frame.cloud->size();
  frame.color_count = frame.color_cloud ? frame.color_cloud->size() : 0;
  if (save_map)
  {
    frame.offset = static_cast<std::uint64_t>(archive.tellp());
    std::vector<float> packed;
    packed.reserve(static_cast<std::size_t>(frame.count) * 5);
    for (const PointType &point : *frame.cloud)
    {
      packed.insert(packed.end(), {point.x, point.y, point.z, point.intensity, point.curvature});
    }
    archive.write(reinterpret_cast<const char *>(packed.data()), packed.size() * sizeof(float));
    {
      // The merger stored each return's physical beam origin in the normal fields and the
      // deskew moved it with the return, so it stays within the intra-scan motion (<1 m) of
      // its mount while the mounts are metres apart: nearest mount = source LiDAR.
      std::vector<std::uint8_t> sources(frame.count, 255);
      if (!lidar_origins.empty())
      {
        std::size_t index = 0;
        for (const PointType &point : *frame.cloud)
        {
          const V3D origin(point.normal_x, point.normal_y, point.normal_z);
          std::size_t best = 0;
          double best_distance = std::numeric_limits<double>::infinity();
          for (std::size_t s = 0; s < lidar_origins.size(); ++s)
          {
            const double distance = (lidar_origins[s] - origin).squaredNorm();
            if (distance < best_distance) { best_distance = distance; best = s; }
          }
          sources[index++] = static_cast<std::uint8_t>(best);
        }
      }
      source_archive.write(reinterpret_cast<const char *>(sources.data()), sources.size());
    }
    if (color_enabled)
    {
      frame.color_offset = static_cast<std::uint64_t>(color_archive.tellp());
      std::vector<float> colors;
      colors.reserve(static_cast<std::size_t>(frame.color_count) * 6);
      for (const PointTypeRGB &point : *frame.color_cloud)
        colors.insert(colors.end(), {point.x, point.y, point.z,
            static_cast<float>(point.r), static_cast<float>(point.g),
            static_cast<float>(point.b)});
      color_archive.write(reinterpret_cast<const char *>(colors.data()),
                          colors.size() * sizeof(float));
    }
    const auto q = frame.raw.rotation().toQuaternion();
    const auto p = frame.raw.translation();
    frame_index << std::setprecision(17) << frame.stamp << ',' << frame.begin << ',' << frame.offset << ','
        << frame.count << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
        << frame.color_offset << ',' << frame.color_count << '\n';
  }
  recent_frames.push_back(frame);
  while (recent_frames.size() > static_cast<std::size_t>(local_map_frames)) recent_frames.pop_front();
  Frame metadata = frame;
  metadata.cloud.reset();
  metadata.color_cloud.reset();
  frames.push_back(std::move(metadata));
  if (frame.stamp - last_memory_log >= 30.0)
  {
    // Backend memory (keyframe planes with their float support returns dominate; every 30 s of data).
    last_memory_log = frame.stamp;
    std::size_t planes = 0, support = 0, cells = 0, postings = 0;
    for (const auto &key : keyframes)
    {
      planes += key.planes.size();
      for (const auto &plane : key.planes) support += plane.support.capacity();
      cells += key.cells.size();
      for (const auto &cell : key.cells) postings += cell.second.capacity();
    }
    const double megabytes = (static_cast<double>(planes) * sizeof(Plane) +
                              static_cast<double>(support) * sizeof(Eigen::Vector3f) + static_cast<double>(cells) * 64.0 +
                              static_cast<double>(postings) * 4.0 + static_cast<double>(frames.size()) * sizeof(Frame)) /
                             1048576.0;
    RCLCPP_INFO(node->get_logger(),
        "[memory] backend: %zu frames, %zu keyframes, %zu planes, %zu support returns, %zu cells (~%.0f MB)",
        frames.size(), keyframes.size(), planes, support, cells, megabytes);
  }
  // One scan of latency lets same-stamp INS/status callbacks arrive before
  // matching. A late RTK seed must not leave permanently frozen wrong matches.
  const Frame &candidate = recent_frames.size() > 1 ? recent_frames[recent_frames.size() - 2] : frame;
  // Detect a native discontinuity between the two newest frames: the front
  // end can settle/jump by several centimetres right after IMU
  // initialization (and after any relocalization) while the vehicle and the
  // INS reference barely move. Such a step between sparse knots would be
  // smeared across the whole keyframe interval by the interpolated global
  // correction, so bracket it with immediate knots on both sides instead.
  bool discontinuity_key = pending_discontinuity_key;
  pending_discontinuity_key = false;
  if (frames.size() >= 2)
  {
    const Frame &previous = frames[frames.size() - 2];
    const double step_dt = frame.stamp - previous.stamp;
    Observation previous_reference, current_reference;
    if (step_dt > 0 && step_dt < 0.5 &&
        referenceAt(previous.stamp, previous_reference) &&
        referenceAt(frame.stamp, current_reference) &&
        previous_reference.imu_position.allFinite() &&
        current_reference.imu_position.allFinite())
    {
      const V3D native_step = frame.raw.translation() - previous.raw.translation();
      const V3D reference_step = current_reference.imu_position - previous_reference.imu_position;
      const double mismatch = (native_step - reference_step).norm();
      if (mismatch > key_discontinuity)
      {
        discontinuity_key = true;
        pending_discontinuity_key = true;
        RCLCPP_INFO(node->get_logger(),
            "Native pose discontinuity %.3f m within %.3f s at %.3f; bracketing correction knots",
            mismatch, step_dt, frame.stamp);
      }
    }
  }
  bool keyframe = keyframes.empty();
  knot_parked = false;
  if (!keyframe)
  {
    const double dt = candidate.stamp - keyframes.back().stamp;
    const auto relative = keyframes.back().raw.between(candidate.raw);
    const double moved = relative.translation().norm();
    const double turned = gtsam::Rot3::Logmap(relative.rotation()).norm();
    // Standstill: knots carry no new geometry and the INS fix is time-correlated, so dense knots only
    // densify the graph (truck29raw 20261013: 720 of 1741 knots in two parkings). Space them by
    // key_stationary_interval; the first native motion above the thresholds restores the normal spacing.
    if (std::abs(candidate.frontend_speed) >= 0.5) motion_seen = true;
    const bool parked = motion_seen && moved < stationary_translation && turned < stationary_rotation &&
        std::abs(candidate.frontend_speed) < stationary_speed;
    const bool standstill = key_stationary_interval > 0 && parked;
    knot_parked = parked && !discontinuity_key;
    // The published (newest solved) scan must stay in the recent cache: at full scan rate the cache spans
    // local_map_frames scans (~3 s), so a standstill knot is also due when the last knot is about to leave it.
    const bool cache_due = recent_frames.size() >= static_cast<std::size_t>(local_map_frames) &&
        recent_frames.front().stamp >= keyframes.back().stamp - 1e-6;
    keyframe = dt >= key_min_interval && (standstill ? (dt >= key_stationary_interval || cache_due) :
        (dt >= key_interval || moved >= key_translation || turned >= key_rotation));
    if (discontinuity_key && dt > 1e-6) keyframe = true;
  }
  if (keyframe) optimizeKeyframe(candidate);
  // Do not label a failed native extrapolation beyond the newest correction
  // knot as an optimized current pose/cloud. All outputs share the newest
  // solved scan timestamp; the bounded latency is logged, and finish/idle
  // flushing still solves/publishes the real final scan.
  const auto boundary = std::upper_bound(recent_frames.begin(), recent_frames.end(),
      keyframes.back().stamp + 1e-6, [](double stamp, const Frame &item) { return stamp < item.stamp; });
  if (boundary == recent_frames.begin()) throw std::runtime_error("Solved scan missing from recent cache");
  const Frame &published = *std::prev(boundary);
  if (keyframe) publishCurrent(published);
  publish(published.stamp);
  double residual = std::numeric_limits<double>::quiet_NaN();
  if (!keyframes.empty() && keyframes.back().has_position)
    residual = (keyframes.back().optimized.transformFrom(lever) - keyframes.back().reference).norm();
  const double milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  Observation quality_checked;
  int reference_failure = 0;
  referenceAt(frame.stamp, quality_checked, &reference_failure);
  const double tail_residual = reference_failure == 0 ?
      (poseAt(frame).transformFrom(lever) - quality_checked.position).norm() :
      std::numeric_limits<double>::quiet_NaN();
  Observation published_reference;
  const double published_residual = referenceAt(published.stamp, published_reference) ?
      (poseAt(published).transformFrom(lever) - published_reference.position).norm() :
      std::numeric_limits<double>::quiet_NaN();
  diagnostics << std::setprecision(17) << frame.stamp << ',' << keyframes.size() << ','
      << accepted_positions << ',' << missing_positions << ',' << keyframes.back().planes.size() << ','
      << solver.getFactorsUnsafe().nrFactors() << ',' << published_residual << ',' << milliseconds << ','
      << frame.frontend_residual << ',' << frame.frontend_speed << ','
      << reference_failure << ',' << graph_in_enu;
  for (double duration : key_timings) diagnostics << ',' << duration;
  diagnostics << ',' << published.stamp << ',' << frame.stamp - published.stamp << ',' << residual << ',' << tail_residual;
  diagnostics << ',' << refinement_steps << ',' << refinement_translation << ',' << refinement_rotation * 180 / M_PI;
  diagnostics << ',' << (graph_in_enu && published_reference.imu_position.allFinite() ?
      (poseAt(published).translation() - published_reference.imu_position).norm() :
      std::numeric_limits<double>::quiet_NaN());
  diagnostics << ',' << last_relinearized << ',' << last_reeliminated << ',' << last_cliques;
  diagnostics << '\n';
  if (keyframe && (accepted_positions == 0 || (std::isfinite(residual) && residual > 0.10)))
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "RTK quality/consistency requires attention: %zu position factors, latest residual %.3f m",
        accepted_positions, residual);
}

void RtkFusion::publishCurrent(const Frame &frame)
{
  // No-RTK rounds: the backend only exports; the native displays (/path, frontend clouds)
  // carry the visualization. Publishing /rtk/* here would light the "optimized" RViz
  // displays and suggest a fused solution that does not exist.
  if (!observations_enabled) return;
  if (rclcpp::ok())
  {
    const auto fused = poseAt(frame);
    const auto p = fused.translation();
    const auto q = fused.rotation().toQuaternion();
    nav_msgs::Odometry message;
    message.header.frame_id = graph_in_enu ? "mine" : "lio_local";
    message.header.stamp = stampFromSec(frame.stamp);
    message.child_frame_id = "rtk_imu";
    message.pose.pose.position.x = p.x(); message.pose.pose.position.y = p.y(); message.pose.pose.position.z = p.z();
    message.pose.pose.orientation.x = q.x(); message.pose.pose.orientation.y = q.y();
    message.pose.pose.orientation.z = q.z(); message.pose.pose.orientation.w = q.w();
    // This correlated graph does not provide calibrated global covariance.
    // Never publish the default all-zero covariance as perfect certainty.
    // Downstream consumers must not use this display topic as a calibrated
    // independent measurement; trajectory and diagnostics are the outputs.
    for (int axis = 0; axis < 6; ++axis)
    {
      message.pose.covariance[7 * axis] = 1e6;
      message.twist.covariance[7 * axis] = 1e6;
    }
    odom_publisher->publish(message);
    if (current_publisher->get_subscription_count() > 0)
    {
      sensor_msgs::PointCloud2 current;
      if (color_enabled)
      {
        PointCloudXYZRGB cloud;
        appendDisplayCloud(frame, cloud);
        pcl::toROSMsg(cloud, current);
      }
      else
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        appendCloud(frame, cloud);
        pcl::toROSMsg(cloud, current);
      }
      current.header = message.header;
      current_publisher->publish(current);
    }
  }
}

void RtkFusion::publish(double stamp, bool force)
{
  if (!keyframes.empty()) stamp = std::min(stamp, keyframes.back().stamp);
  if (!force && last_publish >= 0 && stamp - last_publish < publish_interval) return;
  last_publish = stamp;
  diagnostics.flush();
  if (!rclcpp::ok()) return;
  // observations_enabled=false: still compute the online-vs-INS assessment and write
  // online_path.csv (studio live chart), but never publish fused topics (/rtk/*).
  if (observations_enabled && force && !recent_frames.empty()) publishCurrent(recent_frames.back());
  nav_msgs::Path path;
  path.header.frame_id = graph_in_enu ? "mine" : "lio_local";
  path.header.stamp = stampFromSec(stamp);
  path.poses.reserve(frames.size());
  std::size_t assessed = 0, over10 = 0, over20 = 0;
  double sum = 0, maximum = -1, worst_stamp = -1;
  double latest = std::numeric_limits<double>::quiet_NaN();
  for (const Frame &frame : frames)
  {
    if (frame.stamp > stamp + 1e-6) break;
    const auto pose = poseAt(frame);
    const auto p = pose.translation();
    const auto q = pose.rotation().toQuaternion();
    latest = std::numeric_limits<double>::quiet_NaN();
    if (graph_in_enu && frame.reference_imu.allFinite())
    {
      latest = (p - frame.reference_imu).norm();
      ++assessed; sum += latest; over10 += latest > .10; over20 += latest > .20;
      if (latest > maximum) { maximum = latest; worst_stamp = frame.stamp; }
    }
    geometry_msgs::PoseStamped message;
    message.header.frame_id = path.header.frame_id;
    message.header.stamp = stampFromSec(frame.stamp);
    message.pose.position.x = p.x(); message.pose.position.y = p.y(); message.pose.position.z = p.z();
    message.pose.orientation.x = q.x(); message.pose.orientation.y = q.y();
    message.pose.orientation.z = q.z(); message.pose.orientation.w = q.w();
    path.poses.push_back(message);
  }
  if (observations_enabled) path_publisher->publish(path);
  path_log << std::setprecision(17) << stamp << ',' << assessed << ',' << latest << ','
      << (assessed ? maximum : std::numeric_limits<double>::quiet_NaN()) << ',' << worst_stamp << ','
      << (assessed ? sum / assessed : std::numeric_limits<double>::quiet_NaN()) << ',' << over10 << ',' << over20 << '\n';
  path_log.flush();
  if (observations_enabled && map_publisher->get_subscription_count() > 0)
  {
    sensor_msgs::PointCloud2 message;
    if (color_enabled)
    {
      PointCloudXYZRGB cloud;
      for (const Frame &frame : recent_frames)
        if (frame.stamp <= stamp + 1e-6) appendDisplayCloud(frame, cloud);
      pcl::toROSMsg(cloud, message);
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      for (const Frame &frame : recent_frames)
        if (frame.stamp <= stamp + 1e-6) appendCloud(frame, cloud);
      pcl::toROSMsg(cloud, message);
    }
    message.header = path.header;
    map_publisher->publish(message);
  }
  if (observations_enabled) publishGraph(latest, assessed ? maximum : std::numeric_limits<double>::quiet_NaN());
}

void RtkFusion::publishGraph(double latest_error, double max_error)
{
  if (keyframes.empty() || !rclcpp::ok() || graph_publisher->get_subscription_count() == 0) return;
  using visualization_msgs::Marker;
  visualization_msgs::MarkerArray array;
  const auto stamp = stampFromSec(keyframes.back().stamp);
  const auto marker = [&](const char *ns, int type, double scale, std::array<float, 4> rgba) {
    Marker m;
    m.header.frame_id = graph_in_enu ? "mine" : "lio_local";
    m.header.stamp = stamp;
    m.ns = ns;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1;
    m.scale.x = m.scale.y = m.scale.z = scale;
    m.color.r = rgba[0]; m.color.g = rgba[1]; m.color.b = rgba[2]; m.color.a = rgba[3];
    return m;
  };
  const auto point = [](const V3D &p) {
    geometry_msgs::msg::Point q;
    q.x = p.x(); q.y = p.y(); q.z = p.z();
    return q;
  };
  // An empty point list is invalid for LINE/SPHERE lists: clear that namespace instead.
  const auto push = [&](Marker &m) {
    if (m.type != Marker::TEXT_VIEW_FACING && m.points.empty()) m.action = Marker::DELETE;
    array.markers.push_back(m);
  };
  // Keyframes (correction knots) coloured by their final/online RTK residual: green < 5 cm, yellow < 8 cm
  // (loop RTK bound), orange < 10 cm (acceptance), red above; grey = no RTK factor (quality/time rejected).
  auto keys = marker("keyframes", Marker::SPHERE_LIST, 0.6, {1, 1, 1, 1});
  std::size_t over8 = 0;
  for (const auto &key : keyframes)
  {
    keys.points.push_back(point(key.optimized.translation()));
    std_msgs::msg::ColorRGBA color;
    color.a = 1;
    if (!key.has_position) { color.r = color.g = color.b = .55f; }
    else
    {
      const double r = (key.optimized.transformFrom(lever) - key.reference).norm();
      over8 += r >= .08;
      if (r < .05) { color.r = .1f; color.g = .85f; color.b = .2f; }
      else if (r < .08) { color.r = 1; color.g = .85f; }
      else if (r < .10) { color.r = 1; color.g = .45f; }
      else { color.r = 1; color.g = .1f; color.b = .1f; }
    }
    keys.colors.push_back(color);
  }
  push(keys);
  // Online revisit links: scan-plane factors between keyframes that came back to a place (>= revisit_min_path).
  auto revisits = marker("revisit_links", Marker::LINE_LIST, 0.06, {.1f, .8f, 1, .85f});
  for (const auto &link : geometry_links)
    if (link.kind == GeometryKind::Revisit && link.a < keyframes.size() && link.b < keyframes.size())
    {
      revisits.points.push_back(point(keyframes[link.a].optimized.translation()));
      revisits.points.push_back(point(keyframes[link.b].optimized.translation()));
    }
  push(revisits);
  // Final submap loop closures (loops.csv): kept, rejected by registration, dropped by the RTK bound.
  auto kept = marker("loop_closures", Marker::LINE_LIST, 0.18, {.15f, 1, .3f, 1});
  auto rejected = marker("loops_rejected", Marker::LINE_LIST, 0.06, {1, .15f, .15f, .7f});
  auto dropped = marker("loops_rtk_dropped", Marker::LINE_LIST, 0.12, {1, .55f, 0, 1});
  for (const auto &loop : loop_display)
  {
    auto &target = loop.state == 0 ? kept : loop.state == 2 ? dropped : rejected;
    target.points.push_back(point(keyframes[loop.a].optimized.translation()));
    target.points.push_back(point(keyframes[loop.b].optimized.translation()));
  }
  push(kept); push(rejected); push(dropped);
  auto text = marker("status", Marker::TEXT_VIEW_FACING, 2.0, {1, 1, 1, 1});
  // Beside the vehicle (top-down views would put it on the vehicle marker and the current scan).
  text.pose.position = point(keyframes.back().optimized.translation() + V3D(0, 12, 6));
  char buffer[256];
  if (finalized)
    std::snprintf(buffer, sizeof(buffer), "FINAL  %zu keyframes, %zu >= 8 cm RTK residual\nloops kept %zu / rejected %zu / RTK-dropped %zu",
                  keyframes.size(), over8,
                  static_cast<std::size_t>(std::count_if(loop_display.begin(), loop_display.end(), [](const LoopDisplay &l) { return l.state == 0; })),
                  static_cast<std::size_t>(std::count_if(loop_display.begin(), loop_display.end(), [](const LoopDisplay &l) { return l.state == 1; })),
                  static_cast<std::size_t>(std::count_if(loop_display.begin(), loop_display.end(), [](const LoopDisplay &l) { return l.state == 2; })));
  else
    std::snprintf(buffer, sizeof(buffer), "RTK fusion  err %.1f cm (max %.1f)\n%zu keyframes, %zu revisit links%s",
                  latest_error * 100, max_error * 100, keyframes.size(), revisits.points.size() / 2,
                  knot_parked ? ", parked (hold)" : "");
  text.text = buffer;
  push(text);
  graph_publisher->publish(array);
}

std::size_t RtkFusion::exportFrame(const Frame &frame, std::ifstream &input, std::vector<float> &packed,
                                   pcl::PointCloud<pcl::PointXYZI> &output, const PriorityCells *priority,
                                   std::vector<float> *ranges, std::vector<std::uint8_t> *written,
                                   std::ifstream *sources, std::vector<Eigen::Vector3f> *origins) const
{
  packed.resize(static_cast<std::size_t>(frame.count) * 5);
  input.seekg(static_cast<std::streamoff>(frame.offset));
  input.read(reinterpret_cast<char *>(packed.data()), static_cast<std::streamsize>(packed.size() * sizeof(float)));
  if (!input) throw std::runtime_error("Cannot read the scan archive for the map export");
  // sources.bin (LIVOSRC1): 8-byte header, then one byte per scans.bin return = index of its LiDAR mount.
  std::vector<std::uint8_t> source;
  if (origins)
  {
    origins->clear();
    origins->reserve(frame.count);
    if (sources)
    {
      source.resize(frame.count);
      sources->seekg(static_cast<std::streamoff>(8 + (frame.offset - 8) / 20));
      sources->read(reinterpret_cast<char *>(source.data()), static_cast<std::streamsize>(source.size()));
      if (!*sources) throw std::runtime_error("Cannot read sources.bin for the map export");
    }
  }
  const double free_voxel = 0.5 * save_map_priority_voxel;
  const M3D native_rotation = frame.raw.rotation().matrix() * lidar_rotation;
  const V3D native_translation = frame.raw.transformFrom(lidar_translation);
  // Same arithmetic as appendCloud. Returns of one firing share their offset, hence their correction:
  // evaluate correctionAt once per distinct offset (it dominated the export at ~350 M returns).
  std::unordered_map<float, gtsam::Pose3> corrections;
  corrections.reserve(4096);
  output.clear();
  output.reserve(frame.count);
  if (ranges)
  {
    ranges->clear();
    ranges->reserve(frame.count);
  }
  if (written) written->assign((frame.count + 7) / 8, 0);
  std::size_t in_cap = 0;
  for (std::size_t i = 0; i < frame.count; ++i)
  {
    const float *v = &packed[5 * i];
    const float range = Eigen::Vector3f(v[0], v[1], v[2]).norm();
    if (save_map_max_range > 0.0 && range > save_map_max_range) continue;
    ++in_cap;
    double point_stamp = frame.begin + v[4] / 1000.0;
    if (!std::isfinite(point_stamp) || point_stamp < frame.begin - 1e-3 || point_stamp > frame.stamp + 1e-3)
      throw std::runtime_error("Archived LiDAR point timestamp is outside its submitted frame interval");
    point_stamp = std::max(frame.begin, std::min(frame.stamp, point_stamp));
    auto found = corrections.find(v[4]);
    if (found == corrections.end()) found = corrections.emplace(v[4], correctionAt(point_stamp)).first;
    const V3D native = native_rotation * V3D(v[0], v[1], v[2]) + native_translation;
    const V3D p = found->second.transformFrom(native);
    pcl::PointXYZI result;
    result.x = p.x(); result.y = p.y(); result.z = p.z(); result.intensity = v[3];
    if (priority)
    {
      const std::uint64_t key = priorityCell(result.x, result.y, result.z, save_map_priority_voxel);
      const auto &shard = priority->occupied[key % priority->occupied.size()];
      const auto cell = shard.find(key);
      if (cell != shard.end() &&
          range > std::max(save_map_priority_ratio * cell->second, cell->second + save_map_priority_margin))
        continue;
      if (!priority->free.empty())
      {
        const std::uint64_t free_key = priorityCell(result.x, result.y, result.z, free_voxel);
        const auto &free_shard = priority->free[free_key % priority->free.size()];
        const auto seen = free_shard.find(free_key);
        if (seen != free_shard.end() &&
            range > std::max(save_map_priority_ratio * seen->second, seen->second + save_map_priority_margin))
        {
          // not in the column's two lowest occupied cells (ground band, see save_map_priority_free_range)
          const auto ground = priority->ground.find(key >> 21);
          if (ground == priority->ground.end() || priorityCellZ(key) > ground->second + 1) continue;
        }
      }
    }
    output.push_back(result);
    if (ranges) ranges->push_back(range);
    if (origins)
    {
      const V3D mount = !source.empty() && source[i] < lidar_origins.size() ? lidar_origins[source[i]] : V3D::Zero();
      origins->push_back(found->second.transformFrom(V3D(native_rotation * mount + native_translation)).cast<float>());
    }
    if (written) (*written)[i / 8] |= static_cast<std::uint8_t>(1U << (i % 8));
  }
  return in_cap;
}

namespace
{
// 3 cm colour voxel: cell index and key (21 bits per axis, offset so that within +-31 km of the origin the keys
// ascend with x, then y, then z - ColorLookup::find walks the neighbour voxels in that order).
std::int64_t colourCell(double v) { return static_cast<std::int64_t>(std::floor(v / 0.03)); }
std::uint64_t colourKey(std::int64_t ix, std::int64_t iy, std::int64_t iz)
{
  const auto field = [](std::int64_t c) { return static_cast<std::uint64_t>(c + (1 << 20)) & 0x1FFFFF; };
  return (field(ix) << 42) | (field(iy) << 21) | field(iz);
}
}  // namespace

bool RtkFusion::ColorLookup::find(double x, double y, double z, std::uint32_t &value) const
{
  // The point's own 3 cm voxel, else the nearest kept observation within 6 cm. A ball: the former fallback (own
  // 6 cm voxel) missed observations 1 mm across a voxel border but took others 10 cm away (20261013, 4 areas:
  // 32-45 % -> 50-68 % of the geometry rows coloured; rows coloured from their own voxel keep exactly that colour).
  const std::uint64_t key = colourKey(colourCell(x), colourCell(y), colourCell(z));
  const auto own = std::lower_bound(keys.begin(), keys.end(), key);
  if (own != keys.end() && *own == key)
  {
    value = rgb[static_cast<std::size_t>(own - keys.begin())];
    return true;
  }
  constexpr double radius = 0.06;
  const Eigen::Vector3f point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  const std::int64_t y0 = colourCell(y - radius), y1 = colourCell(y + radius);
  const std::int64_t z0 = colourCell(z - radius), z1 = colourCell(z + radius);
  float best = static_cast<float>(radius * radius);
  std::size_t found = keys.size();
  for (std::int64_t ix = colourCell(x - radius); ix <= colourCell(x + radius); ++ix)
  {
    auto k = std::lower_bound(keys.begin(), keys.end(), colourKey(ix, y0, z0));
    const auto end = std::upper_bound(k, keys.end(), colourKey(ix, y1, z1));
    for (std::int64_t iy = y0; iy <= y1 && k != end; ++iy)
    {
      const std::uint64_t last = colourKey(ix, iy, z1);
      for (k = std::lower_bound(k, end, colourKey(ix, iy, z0)); k != end && *k <= last; ++k)
      {
        const auto i = static_cast<std::size_t>(k - keys.begin());
        const float squared = (position[i] - point).squaredNorm();
        if (squared <= best)
        {
          best = squared;
          found = i;
        }
      }
    }
  }
  if (found == keys.size()) return false;
  value = rgb[found];
  return true;
}

RtkFusion::ColorLookup RtkFusion::colorLookup(std::size_t batch) const
{
  // Every frame recolors the surfaces inside its frustum, so the archive holds many color instances of the same
  // physical spot. Keep, per voxel, only the instance with the lowest score: range (the pixel footprint grows
  // linearly with range, while the metric colour displacement from a residual camera time error, speed * dt, does
  // not shrink with it) + 20 m per rad/s of rotation (motion blur / time skew scale with omega * range).
  // The coloured returns are the camera-frustum returns of each scan: for roadside poles only the 16-60 m views
  // ahead (20261013: no pole return closer than 15.7 m). Exporting them as the RGB map gave poles the far-range
  // shell (P10-P90 18 cm vs 3-5 cm of map_N with range priority); they now only lend their colours to the
  // geometry rows, so rgb_map_N has the geometry of map_N.
  struct Scored { float score; std::uint32_t rgb; float x, y, z; };
  struct Candidate { std::uint64_t key; Scored value; };
  std::unordered_map<std::uint64_t, Scored> best;
  best.reserve(1 << 22);
  std::vector<std::vector<Candidate>> scored(batch);
  for (std::size_t first = 0; first < frames.size(); first += batch)
  {
    const std::size_t count = std::min(batch, frames.size() - first);
    std::exception_ptr error;
#pragma omp parallel
    {
      std::ifstream input(directory / "colors.bin", std::ios::binary);
      std::vector<float> packed;
#pragma omp for schedule(dynamic, 4)
      for (int n = 0; n < static_cast<int>(count); ++n)
      {
        const std::size_t fi = first + static_cast<std::size_t>(n);
        auto &out = scored[n];
        out.clear();
        try
        {
          const Frame &frame = frames[fi];
          if (frame.color_count == 0) continue;
          packed.resize(static_cast<std::size_t>(frame.color_count) * 6);
          input.seekg(static_cast<std::streamoff>(frame.color_offset));
          input.read(reinterpret_cast<char *>(packed.data()), static_cast<std::streamsize>(packed.size() * sizeof(float)));
          if (!input) throw std::runtime_error("Cannot read the colour archive for the map export");
          double omega = 0.0;
          if (fi + 1 < frames.size())
          {
            const double dt = std::max(frames[fi + 1].stamp - frames[fi].stamp, 1e-3);
            const auto delta = frames[fi].raw.between(frames[fi + 1].raw);
            omega = gtsam::Rot3::Logmap(delta.rotation()).norm() / dt;
          }
          const auto correction = correctionAt(frame.stamp);
          const V3D origin = frame.raw.transformFrom(lidar_translation);
          out.reserve(frame.color_count);
          for (std::size_t i = 0; i < frame.color_count; ++i)
          {
            const V3D native(packed[6 * i], packed[6 * i + 1], packed[6 * i + 2]);
            const double range = (native - origin).norm();
            if (save_rgb_max_range > 0.0 && range > save_rgb_max_range) continue;
            const V3D p = correction.transformFrom(native);
            const std::uint32_t rgb = (static_cast<std::uint32_t>(static_cast<std::uint8_t>(packed[6 * i + 3])) << 16) |
                                      (static_cast<std::uint32_t>(static_cast<std::uint8_t>(packed[6 * i + 4])) << 8) |
                                      static_cast<std::uint32_t>(static_cast<std::uint8_t>(packed[6 * i + 5]));
            out.push_back({colourKey(colourCell(p.x()), colourCell(p.y()), colourCell(p.z())),
                           {static_cast<float>(range + 20.0 * omega), rgb, static_cast<float>(p.x()),
                            static_cast<float>(p.y()), static_cast<float>(p.z())}});
          }
        }
        catch (...)
        {
#pragma omp critical(rgb_export_error)
          if (!error) error = std::current_exception();
        }
      }
    }
    if (error) std::rethrow_exception(error);
    for (std::size_t n = 0; n < count; ++n)
      for (const auto &candidate : scored[n])
      {
        const auto it = best.find(candidate.key);
        if (it == best.end()) best.emplace(candidate.key, candidate.value);
        else if (candidate.value.score < it->second.score) it->second = candidate.value;
      }
  }
  scored.clear();
  // Compact sorted arrays (binary search during the geometry export, 24 bytes per voxel).
  struct Item { std::uint64_t key; float x, y, z; std::uint32_t rgb; };
  std::vector<Item> items;
  items.reserve(best.size());
  for (const auto &[key, value] : best) items.push_back({key, value.x, value.y, value.z, value.rgb});
  std::unordered_map<std::uint64_t, Scored>().swap(best);
  std::sort(items.begin(), items.end(), [](const Item &a, const Item &b) { return a.key < b.key; });
  ColorLookup lookup;
  lookup.keys.reserve(items.size());
  lookup.rgb.reserve(items.size());
  lookup.position.reserve(items.size());
  for (const auto &item : items)
  {
    lookup.keys.push_back(item.key);
    lookup.rgb.push_back(item.rgb);
    lookup.position.emplace_back(item.x, item.y, item.z);
  }
  return lookup;
}

RtkFusion::PriorityCells RtkFusion::nearestObservation() const
{
  // Range priority, pass 1: per cell the nearest range of a scan that put >= 2 returns into it (one stray
  // return - dust, multipath - cannot claim a cell). Far returns of thin objects are displaced toward the
  // sensor and spread along the beam (truck29 20261013 lamp poles: shell P10-P90 12-20 cm with every return
  // within 45 m, ~2-4 cm from the near lane), so the copy seen from the far lane read as a 10-20 cm ghost.
  // Seen-free cells: part of that smear lands beside the pole, in cells no near scan put returns into, so the
  // occupied rule keeps it (straight-edged fringes along the cell borders, 3-10 % of a pole's rows). A near
  // return proves the space just in front of it empty: the half-voxel cells 0.6 and 1.2 voxels before it along
  // its physical beam get its range, and exportFrame drops the much farther returns found there.
  constexpr std::size_t shards = 32;
  PriorityCells priority;
  priority.occupied.resize(shards);
  const bool carve = save_map_priority_free_range > 0.0;
  if (carve) priority.free.resize(shards);
  const bool have_sources = std::filesystem::exists(directory / "sources.bin");
  const double free_voxel = 0.5 * save_map_priority_voxel;
  const std::array<float, 2> depths{static_cast<float>(0.6 * save_map_priority_voxel),
                                    static_cast<float>(1.2 * save_map_priority_voxel)};
  const std::size_t batch = 128;
  std::vector<std::vector<std::pair<std::uint64_t, float>>> cells(batch), free_cells(batch);
  for (std::size_t first = 0; first < frames.size(); first += batch)
  {
    const std::size_t count = std::min(batch, frames.size() - first);
    std::exception_ptr error;
#pragma omp parallel
    {
      std::ifstream input(directory / "scans.bin", std::ios::binary);
      std::ifstream sources;
      if (carve && have_sources) sources.open(directory / "sources.bin", std::ios::binary);
      std::vector<float> packed, ranges;
      std::vector<Eigen::Vector3f> origins;
      pcl::PointCloud<pcl::PointXYZI> cloud;
      std::unordered_map<std::uint64_t, std::pair<float, int>> local;
      std::unordered_map<std::uint64_t, float> local_free;
#pragma omp for schedule(dynamic, 4)
      for (int n = 0; n < static_cast<int>(count); ++n)
      {
        try
        {
          exportFrame(frames[first + static_cast<std::size_t>(n)], input, packed, cloud, nullptr, &ranges, nullptr,
                      sources.is_open() ? &sources : nullptr, carve ? &origins : nullptr);
          local.clear();
          local_free.clear();
          for (std::size_t i = 0; i < cloud.size(); ++i)
          {
            const auto &q = cloud.points[i];
            auto &entry = local.try_emplace(priorityCell(q.x, q.y, q.z, save_map_priority_voxel),
                                            std::make_pair(ranges[i], 0)).first->second;
            entry.first = std::min(entry.first, ranges[i]);
            ++entry.second;
            if (!carve) continue;
            const Eigen::Vector3f point(q.x, q.y, q.z);
            const Eigen::Vector3f beam = point - origins[i];
            const float length = beam.norm();
            if (!(length > depths[1]) || length >= save_map_priority_free_range) continue;
            for (const float depth : depths)
            {
              const Eigen::Vector3f sample = point - (depth / length) * beam;
              auto &seen = local_free.try_emplace(priorityCell(sample.x(), sample.y(), sample.z(), free_voxel),
                                                  ranges[i]).first->second;
              seen = std::min(seen, ranges[i]);
            }
          }
          auto &out = cells[static_cast<std::size_t>(n)];
          out.clear();
          for (const auto &entry : local)
            if (entry.second.second >= 2) out.emplace_back(entry.first, entry.second.first);
          free_cells[static_cast<std::size_t>(n)].assign(local_free.begin(), local_free.end());
        }
        catch (...)
        {
#pragma omp critical(map_priority_error)
          if (!error) error = std::current_exception();
        }
      }
    }
    if (error) std::rethrow_exception(error);
#pragma omp parallel for schedule(static, 1)
    for (int s = 0; s < static_cast<int>(shards); ++s)
      for (std::size_t n = 0; n < count; ++n)
      {
        for (const auto &cell : cells[n])
          if (cell.first % shards == static_cast<std::size_t>(s))
          {
            const auto it = priority.occupied[static_cast<std::size_t>(s)].try_emplace(cell.first, cell.second).first;
            it->second = std::min(it->second, cell.second);
          }
        if (carve)
          for (const auto &cell : free_cells[n])
            if (cell.first % shards == static_cast<std::size_t>(s))
            {
              const auto it = priority.free[static_cast<std::size_t>(s)].try_emplace(cell.first, cell.second).first;
              it->second = std::min(it->second, cell.second);
            }
      }
  }
  if (carve)
    for (const auto &shard : priority.occupied)
      for (const auto &cell : shard)
      {
        const auto it = priority.ground.try_emplace(cell.first >> 21, priorityCellZ(cell.first)).first;
        it->second = std::min(it->second, priorityCellZ(cell.first));
      }
  return priority;
}

void RtkFusion::exportMap()
{
  // Record the actual initial observation for exact later graph audits,
  // including a possible delayed ENU transform. CSV readers ignore comments.
  const auto prior = std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(solver.getFactorsUnsafe().at(0));
  if (!prior) throw std::runtime_error("Missing initial pose observation at export");
  const auto gaussian = std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(prior->noiseModel());
  if (!gaussian) throw std::runtime_error("Initial pose covariance is not Gaussian");
  const auto q = prior->prior().rotation().toQuaternion();
  optimization_log << std::setprecision(17) << "# initial_prior_pose " << keyframes.front().stamp << ' '
      << prior->prior().translation().transpose() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  optimization_log << "# initial_prior_covariance";
  const gtsam::Matrix covariance = gaussian->covariance();
  for (int row = 0; row < 6; ++row) for (int column = 0; column < 6; ++column)
    optimization_log << ' ' << covariance(row, column);
  optimization_log << '\n';
  optimization_log.flush();
  std::ofstream motion(directory / "motion_residuals.csv");
  motion.exceptions(std::ios::failbit | std::ios::badbit);
  motion << "a_stamp,b_stamp,rx_deg,ry_deg,rz_deg,tx_m,ty_m,tz_m,robust_cost,huber_scale\n";
  for (const auto &item : solver.getFactorsUnsafe())
  {
    const auto factor = std::dynamic_pointer_cast<NativeMotionFactor>(item);
    if (!factor) continue;
    const auto a = gtsam::Symbol(factor->keys()[0]).index(), b = gtsam::Symbol(factor->keys()[1]).index();
    auto error = factor->physicalError(keyframes[a].optimized, keyframes[b].optimized);
    error.head<3>() *= 180 / M_PI;
    motion << std::setprecision(17) << keyframes[a].stamp << ',' << keyframes[b].stamp;
    for (double value : error) motion << ',' << value;
    motion << ',' << factor->error(estimate) << ',' << factor->robustScale() << '\n';
  }
  {
    std::ofstream attitude(directory / "attitude_residuals.csv");
    attitude.exceptions(std::ios::failbit | std::ios::badbit);
    const gtsam::Rot3 misalignment = misalignment_initialized ?
        estimate.at<gtsam::Rot3>(gtsam::Symbol('m', 0)) : gtsam::Rot3();
    const V3D mis = gtsam::Rot3::Logmap(misalignment) * 180 / M_PI;
    final_misalignment_deg_ = mis;
    attitude << "# installation_misalignment_deg(body x,y,z) " << mis.transpose() << '\n';
    attitude << "stamp,rx_deg,ry_deg,rz_deg,robust_cost\n";
    for (const auto &item : solver.getFactorsUnsafe())
    {
      const auto factor = std::dynamic_pointer_cast<InsAttitudeFactor>(item);
      if (!factor) continue;
      const auto index = gtsam::Symbol(factor->keys()[0]).index();
      const V3D error = factor->physicalError(keyframes[index].optimized, misalignment) * 180 / M_PI;
      attitude << std::setprecision(17) << keyframes[index].stamp << ',' << error.x() << ',' << error.y()
               << ',' << error.z() << ',' << factor->error(estimate) << '\n';
    }
    RCLCPP_INFO(node->get_logger(),
        "INS attitude factors: %zu; estimated installation misalignment [%.4f %.4f %.4f] deg",
        accepted_attitudes, mis.x(), mis.y(), mis.z());
  }
  std::ofstream geometry(directory / "geometry.csv");
  geometry.exceptions(std::ios::failbit | std::ios::badbit);
  geometry << "a_stamp,b_stamp,baseline_m,matches,normal_x,normal_y,normal_z,offset_rms_m,offset_max_m,offset_p95_m,normalized_rms,native_offset_rms_m,line_matches\n";
  std::ofstream constraints(directory / "geometry_constraints.csv");
  constraints.exceptions(std::ios::failbit | std::ios::badbit);
  constraints << "a_stamp,b_stamp,ax,ay,az,bx,by,bz,nx,ny,nz,sigma_m,residual_m,native_residual_m,is_line";
  for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
    constraints << ",source_body_sqrt_" << row << col;
  constraints << ",centroid_residual_m\n";
  for (const auto &link : geometry_links)
  {
    if (link.factor_index == std::numeric_limits<std::size_t>::max()) continue;
    const auto factor = std::dynamic_pointer_cast<ScanPlaneBatchFactor>(solver.getFactorsUnsafe().at(link.factor_index));
    if (!factor) throw std::runtime_error("Invalid geometry factor index at export");
    const auto relative = keyframes[link.a].optimized.between(keyframes[link.b].optimized);
    const auto native_relative = keyframes[link.a].raw.between(keyframes[link.b].raw);
    std::array<int, 3> counts{};
    int line_count = 0;
    double squared = 0, maximum = 0, normalized_squared = 0, native_squared = 0;
    std::vector<double> distances;
    distances.reserve(factor->constraints().size());
    for (const auto &constraint : factor->constraints())
    {
      Eigen::Index axis;
      constraint.normal.cwiseAbs().maxCoeff(&axis);
      if (constraint.linear) ++line_count; else ++counts[axis];
      const V3D delta = relative.transformFrom(constraint.point) - constraint.center;
      const double signed_residual = constraint.linear ? constraint.project(delta).norm() : constraint.normal.dot(delta);
      const double residual = constraint.physicalResidual(relative).norm();
      const double native_residual = constraint.physicalResidual(native_relative).norm();
      squared += residual * residual;
      maximum = std::max(maximum, residual);
      normalized_squared += residual * residual / (constraint.sigma * constraint.sigma);
      native_squared += native_residual * native_residual;
      distances.push_back(residual);
      // Expose the actual physical constraints, not just Huber-suppressed
      // cost. Large residuals remain visible and traceable to map surfaces.
      const auto &target = keyframes[link.a];
      const auto &source = keyframes[link.b];
      constraints << std::setprecision(17) << target.stamp << ',' << source.stamp;
      for (const V3D vector : {target.optimized.transformFrom(constraint.center),
           source.optimized.transformFrom(constraint.point),
           target.optimized.rotation().rotate(constraint.normal)})
        for (int axis = 0; axis < 3; ++axis) constraints << ',' << vector(axis);
      constraints << ',' << constraint.sigma << ',' << residual << ',' << native_residual << ',' << constraint.linear;
      for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
        constraints << ',' << constraint.source_sqrt(row, col);
      constraints << ',' << signed_residual << '\n';
    }
    geometry << std::setprecision(17) << keyframes[link.a].stamp << ',' << keyframes[link.b].stamp << ','
        << relative.translation().norm() << ',' << factor->constraints().size();
    for (int count : counts) geometry << ',' << count;
    std::sort(distances.begin(), distances.end());
    const double rank = .95 * (distances.size() - 1);
    const auto low = static_cast<std::size_t>(rank);
    const double p95 = distances[low] + (rank - low) * (distances[std::min(low + 1, distances.size() - 1)] - distances[low]);
    geometry << ',' << std::sqrt(squared / distances.size()) << ',' << maximum << ',' << p95
             << ',' << std::sqrt(normalized_squared / distances.size()) << ',' << std::sqrt(native_squared / distances.size()) << ',' << line_count << '\n';
  }
  std::ofstream keys(directory / "keyframes.csv");
  keys.exceptions(std::ios::failbit | std::ios::badbit);
  keys << "stamp,has_position,reference_failure,reference_x,reference_y,reference_z,rtk_residual_m,covariance_source,cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,information_weight";
  for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
    keys << ",native_angular_observability_" << row << column;
  keys << ",has_attitude\n";
  double maximum_key_residual = 0;
  for (const auto &key : keyframes)
  {
    const double residual = key.has_position ?
        (key.optimized.transformFrom(lever) - key.reference).norm() :
        std::numeric_limits<double>::quiet_NaN();
    if (key.has_position) maximum_key_residual = std::max(maximum_key_residual, residual);
    keys << std::setprecision(17) << key.stamp << ',' << key.has_position << ',' << key.reference_failure << ','
         << key.reference.x() << ',' << key.reference.y() << ',' << key.reference.z() << ',' << residual << ','
         << key.reference_covariance_source;
    for (int row = 0; row < 3; ++row) for (int column = row; column < 3; ++column)
      keys << ',' << (key.has_position ? key.reference_covariance(row, column) :
                     std::numeric_limits<double>::quiet_NaN());
    keys << ',' << key.position_information_weight;
    for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
      keys << ',' << key.angular_observability(row, column);
    keys << ',' << key.has_attitude << '\n';
  }
  const auto rejected = std::count_if(keyframes.begin(), keyframes.end(),
      [](const Keyframe &key) { return !key.has_position; });
  if (rejected)
    RCLCPP_WARN(node->get_logger(),
        "RTK coverage: %zu factors, %zu quality/time-rejected keys; remaining valid keys use the configured RTK rate",
        accepted_positions, static_cast<std::size_t>(rejected));
  if (maximum_key_residual > 0.20)
    RCLCPP_WARN(node->get_logger(),
        "Final keyframe RTK residual %.4f m exceeds the 20 cm upper limit; position budget FAILED",
        maximum_key_residual);
  else
    RCLCPP_INFO(node->get_logger(),
        "Final keyframe max RTK residual %.4f m; all-frame route budget and cross-time geometry still require evaluation",
        maximum_key_residual);
  std::ofstream trajectory(directory / "trajectory_optimized.txt");
  std::ofstream original(directory / "trajectory_lio.txt");
  trajectory.exceptions(std::ios::failbit | std::ios::badbit);
  original.exceptions(std::ios::failbit | std::ios::badbit);
  trajectory << "# frame=" << (graph_in_enu ? "mine" : "lio_local") << "; physical IMU\n";
  original << "# frame=" << (graph_in_enu ? "mine" : "lio_local")
           << "; physical IMU; unassisted frontend (initial ENU alignment only)\n";
  std::ofstream corrections(directory / "corrections.txt");
  corrections.exceptions(std::ios::failbit | std::ios::badbit);
  corrections << "# native-LIO to " << (graph_in_enu ? "mine" : "lio_local")
              << "; SE3 correction knots, not physical vehicle poses\n";
  for (const auto &key : keyframes)
  {
    const auto pose = key.optimized * key.raw.inverse();
    const auto q = pose.rotation().toQuaternion();
    const auto p = pose.translation();
    corrections << std::fixed << std::setprecision(9) << key.stamp << ' ' << p.x() << ' ' << p.y() << ' ' << p.z()
        << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  for (const Frame &frame : frames)
  {
    for (bool optimized : {false, true})
    {
      auto &stream = optimized ? trajectory : original;
      const auto pose = optimized ? poseAt(frame) : frontend_to_enu * frame.raw;
      const auto p = pose.translation();
      const auto q = pose.rotation().toQuaternion();
      stream << std::fixed << std::setprecision(9) << frame.stamp << ' '
          << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
          << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
  }
  if (save_map)
  {
    archive.flush();
    source_archive.flush();
    frame_index.flush();
    std::size_t chunk_index = 0;
    // rgb_map_*.pcd = the geometry rows of map_*.pcd that a camera observation colours (colorLookup), written
    // chunk by chunk with the same index; both only when export_map_chunks.
    if (color_enabled) color_archive.flush();
    // Both products: scans are transformed in parallel batches and consumed in frame order, so every row,
    // tie-break and chunk boundary is the one of the former sequential loop (export 90 s -> ~15 s at 8.7k scans).
    const std::size_t export_batch = 128;
    if (export_map_chunks)
    {
      // RViz subscribed: also publish a 0.25 m voxel map of the delivered cloud (every 16th row) on /rtk/map_final.
      const bool display = rclcpp::ok() && final_map_publisher->get_subscription_count() > 0;
      std::unordered_map<std::uint64_t, std::array<float, 5>> display_cells;
      // Range priority (pass 1): nearest supported observation per cell; exportFrame drops the farther returns.
      const auto priority_start = std::chrono::steady_clock::now();
      const PriorityCells priority =
          save_map_priority_enabled && save_map_priority_ratio > 0.0 ? nearestObservation() : PriorityCells();
      const bool prioritized = !priority.occupied.empty();
      const double priority_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - priority_start).count();
      std::size_t priority_cells = 0, seen_free_cells = 0, returns_in_cap = 0, returns_written = 0;
      for (const auto &shard : priority.occupied) priority_cells += shard.size();
      for (const auto &shard : priority.free) seen_free_cells += shard.size();
      std::vector<std::size_t> in_cap(export_batch, 0);
      // With range priority the rows of a scan are a subset of its in-cap returns: map_rows.bin records which
      // (one bit per archived return, frames.csv order) so the evaluation still pairs every row with its return.
      std::vector<std::vector<std::uint8_t>> written(export_batch);
      std::ofstream rows_file;
      if (prioritized)
      {
        rows_file.exceptions(std::ios::badbit | std::ios::failbit);
        rows_file.open(directory / "map_rows.bin", std::ios::binary | std::ios::trunc);
        rows_file.write("LIVOMSK1", 8);
      }
      const auto colour_start = std::chrono::steady_clock::now();
      const ColorLookup colours = color_enabled ? colorLookup(export_batch) : ColorLookup();
      const double colour_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - colour_start).count();
      pcl::PointCloud<pcl::PointXYZI> chunk;
      PointCloudXYZRGB rgb_chunk;
      std::size_t rgb_written = 0;
      std::future<void> writing;  // the previous chunk is written while the next batch is transformed
      const auto write = [&](pcl::PointCloud<pcl::PointXYZI> &&cloud, PointCloudXYZRGB &&rgb) {
        if (writing.valid()) writing.get();
        const std::size_t index = chunk_index++;
        writing = std::async(std::launch::async, [this, index, cloud = std::move(cloud), rgb = std::move(rgb)]() {
          const auto file = directory / ("map_" + std::to_string(index) + ".pcd");
          if (pcl::io::savePCDFileBinary(file.string(), cloud) != 0)
            throw std::runtime_error("Cannot write optimized point cloud " + file.string());
          if (rgb.empty()) return;  // rgb_map_<index>.pcd = coloured rows of map_<index>.pcd
          const auto rgb_file = directory / ("rgb_map_" + std::to_string(index) + ".pcd");
          if (pcl::io::savePCDFileBinary(rgb_file.string(), rgb) != 0)
            throw std::runtime_error("Cannot write optimized RGB point cloud " + rgb_file.string());
        });
      };
      std::vector<pcl::PointCloud<pcl::PointXYZI>> outputs(export_batch);
      std::vector<PointCloudXYZRGB> rgb_outputs(export_batch);
      for (std::size_t first = 0; first < frames.size(); first += export_batch)
      {
        const std::size_t count = std::min(export_batch, frames.size() - first);
        std::exception_ptr error;
#pragma omp parallel
        {
          std::ifstream input(directory / "scans.bin", std::ios::binary);
          std::vector<float> packed;
#pragma omp for schedule(dynamic, 4)
          for (int n = 0; n < static_cast<int>(count); ++n)
          {
            try
            {
              in_cap[static_cast<std::size_t>(n)] = exportFrame(
                  frames[first + static_cast<std::size_t>(n)], input, packed, outputs[n],
                  prioritized ? &priority : nullptr, nullptr, prioritized ? &written[n] : nullptr);
              auto &rgb = rgb_outputs[n];
              rgb.clear();
              if (!colours.empty())
              {
                rgb.reserve(outputs[n].size());
                for (const auto &q : outputs[n])
                {
                  std::uint32_t value = 0;
                  if (!colours.find(q.x, q.y, q.z, value)) continue;
                  PointTypeRGB point;
                  point.x = q.x; point.y = q.y; point.z = q.z;
                  point.r = static_cast<std::uint8_t>((value >> 16) & 0xFF);
                  point.g = static_cast<std::uint8_t>((value >> 8) & 0xFF);
                  point.b = static_cast<std::uint8_t>(value & 0xFF);
                  rgb.push_back(point);
                }
              }
            }
            catch (...)
            {
#pragma omp critical(map_export_error)
              if (!error) error = std::current_exception();
            }
          }
        }
        if (error) std::rethrow_exception(error);
        for (std::size_t n = 0; n < count; ++n)
        {
          returns_in_cap += in_cap[n];
          returns_written += outputs[n].size();
          if (rows_file.is_open())
            rows_file.write(reinterpret_cast<const char *>(written[n].data()),
                            static_cast<std::streamsize>(written[n].size()));
          if (display)
            for (std::size_t i = 0; i < outputs[n].size(); i += 16)
            {
              const auto &q = outputs[n].points[i];
              const auto cell = [](float v) {
                return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v / 0.25f)) & 0x1FFFFF);
              };
              auto &sum = display_cells[(cell(q.x) << 42) | (cell(q.y) << 21) | cell(q.z)];
              sum[0] += q.x; sum[1] += q.y; sum[2] += q.z; sum[3] += q.intensity; sum[4] += 1;
            }
          chunk.insert(chunk.end(), outputs[n].begin(), outputs[n].end());
          rgb_chunk.insert(rgb_chunk.end(), rgb_outputs[n].begin(), rgb_outputs[n].end());
          rgb_written += rgb_outputs[n].size();
          if (chunk.size() >= static_cast<std::size_t>(export_chunk_points))
          {
            write(std::move(chunk), std::move(rgb_chunk));
            chunk = pcl::PointCloud<pcl::PointXYZI>();
            rgb_chunk = PointCloudXYZRGB();
          }
        }
      }
      if (!chunk.empty()) write(std::move(chunk), std::move(rgb_chunk));
      if (writing.valid()) writing.get();
      if (!colours.empty())
        RCLCPP_INFO(node->get_logger(),
            "Saved RGB map: %zu of %zu geometry rows coloured (%.1f %%; %zu colour voxels of 3 cm from coloured "
            "returns within %.1f m, own voxel else nearest within 6 cm, %.1f s): rgb_map_N.pcd = coloured rows of "
            "map_N.pcd",
            rgb_written, returns_written,
            returns_written ? 100.0 * static_cast<double>(rgb_written) / static_cast<double>(returns_written) : 0.0,
            colours.keys.size(), save_rgb_max_range, colour_sec);
      RCLCPP_INFO(node->get_logger(),
          "Saved geometry map: %zu of %zu returns within the %.1f m cap written (%.1f %%; range priority %s: ratio %.2f, "
          "margin %.1f m, %zu cells of %.2f m, %zu seen-free cells of %.2f m from returns within %.1f m, pass 1 %.1f s)",
          returns_written, returns_in_cap, save_map_max_range,
          returns_in_cap ? 100.0 * static_cast<double>(returns_written) / static_cast<double>(returns_in_cap) : 0.0,
          prioritized ? "on" : "off", save_map_priority_ratio, save_map_priority_margin, priority_cells,
          save_map_priority_voxel, seen_free_cells,
          0.5 * save_map_priority_voxel, save_map_priority_free_range, priority_sec);
      if (display && !display_cells.empty())
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.reserve(display_cells.size());
        for (const auto &entry : display_cells)
        {
          const auto &sum = entry.second;
          pcl::PointXYZI q;
          q.x = sum[0] / sum[4]; q.y = sum[1] / sum[4]; q.z = sum[2] / sum[4]; q.intensity = sum[3] / sum[4];
          cloud.push_back(q);
        }
        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(cloud, message);
        message.header.frame_id = graph_in_enu ? "mine" : "lio_local";
        message.header.stamp = stampFromSec(frames.back().stamp);
        final_map_publisher->publish(message);
        RCLCPP_INFO(node->get_logger(), "RViz: final map published on /rtk/map_final (%zu cells of 0.25 m)", cloud.size());
      }
    }
  }
  RCLCPP_INFO(node->get_logger(), "RTK finalized %zu frames, %zu keys, %zu position factors (saved-map range cap %.1f m, rgb %.1f m, 0 = none): %s",
      frames.size(), keyframes.size(), accepted_positions, save_map_max_range, save_rgb_max_range, directory.c_str());
}
