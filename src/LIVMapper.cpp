/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include "frame_profile.h"
#include "rtk_fusion.h"

#include <json/json.h>

#include <fstream>
#include <malloc.h>
#include <set>
#include <unordered_set>

namespace
{
struct DownsampleVoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const DownsampleVoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct DownsampleVoxelKeyHash
{
  std::size_t operator()(const DownsampleVoxelKey &key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) +
            0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) +
            0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct DownsampleMetadata
{
  V3D beam_origin_sum = V3D::Zero();
  double intensity_sum = 0.0;
  std::size_t count = 0;
};

}  // namespace


LIVMapper::LIVMapper(const rclcpp::Node::SharedPtr &node)
    : extT(0, 0, 0),
      node_(node),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(node_, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = string(ROOT_DIR) + "results";
  Ros2ParameterReader output_parameters(node_);
  output_parameters.param<string>("output/directory", root_dir, string(ROOT_DIR) + "results");
  root_dir = projectResultDirectory(root_dir).string() + '/';
  initializeFiles();
  initializeComponents();
  if (rtk_enabled && (slam_mode_ == ONLY_LIO || slam_mode_ == LIVO))
  {
    if (!imu_standard_rfu || !rear_axle_to_imu_enabled)
      throw std::runtime_error("RTK fusion requires standard_rfu IMU and physical-IMU reference_frame_conversion.enabled");
    rtk_fusion_ = std::make_unique<RtkFusion>(node_, extR, extT, imu_to_rear_axle, root_dir,
                                              rtk_reference_time_shift_,
                                              calib_rtk_position_offset_,
                                              calib_ins_attitude_offset_);
    RCLCPP_INFO(node_->get_logger(), "RTK backend enabled for %s frontend",
                slam_mode_ == LIVO ? "LIVO" : "LIO");
    if (multi_lidar_enabled)
    {
      // readParameters() already loaded the LiDAR sources; tag archived returns per LiDAR.
      std::vector<V3D> origins;
      for (const auto &source : lidar_sources) origins.push_back(source->rear_from_lidar_translation);
      if (!rtk_observations_enabled)
        RCLCPP_WARN(node_->get_logger(),
            "rtk.observations_enabled=false: no INS/RTK observations will be fused; the backend runs as a "
            "pure-odometry smoother/exporter and all products stay in the INS-anchored mine frame");
      rtk_fusion_->setLidarOrigins(origins);
    }
    {
      std::ostringstream lever_text;
      lever_text << std::setprecision(17) << imu_to_rear_axle.x() << ' ' << imu_to_rear_axle.y() << ' ' << imu_to_rear_axle.z();
      const auto num = [](double v) { std::ostringstream s; s << std::setprecision(17) << v; return s.str(); };
      rtk_fusion_->appendParameters({
          {"effective.lidar_time_offset_sec", num(lidar_time_offset)},
          {"effective.img_time_offset_sec", num(img_time_offset)},
          {"effective.rtk_reference_time_shift_sec", num(rtk_reference_time_shift_)},
          {"effective.imu_to_rear_axle", lever_text.str()},
          {"effective.camera_mode", img_en ? (vio_ekf_update_en ? "livo" : "colour_only") : "off"},
          {"effective.camera_frame_divider", std::to_string(camera_frame_divider)},
          {"effective.vio_lidar_parity_ratio", num(vio_lidar_parity_ratio)},
          {"calibration.file", calibration_file_},
          {"calibration.id", calibration_identity_}});
    }
  }
  else if (rtk_enabled)
    RCLCPP_WARN(node_->get_logger(), "RTK backend requires an IMU-enabled LIO/LIVO frontend");
  path.header.stamp = stampFromSec(node_->now().seconds());
  path.header.frame_id = lio_world_frame;
  imu_reference_path.header.frame_id = "mine";
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  alignment_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
  printCalibrationSummary("startup");
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters()
{
  Ros2ParameterReader nh(node_);
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<string>("common/ins_odom_topic", ins_odom_topic, "/imu_data/odometry");
  nh.param<bool>("rtk/enabled", rtk_enabled, false);
  nh.param<bool>("rtk/observations_enabled", rtk_observations_enabled, true);
  nh.param<bool>("rtk/frontend_attitude_aiding", frontend_attitude_aiding, false);
  nh.param<double>("rtk/frontend_attitude_sigma_deg", frontend_attitude_sigma_deg, 0.3);
  nh.param<double>("rtk/frontend_attitude_huber_deg", frontend_attitude_huber_deg, 1.0);
  nh.param<double>("rtk/frontend_attitude_pull_fraction", frontend_attitude_pull_fraction, 0.05);
  nh.param("rtk/frontend_attitude_axis_mask", frontend_attitude_axis_mask, std::vector<double>{1.0, 1.0, 0.0});
  nh.param<double>("rtk/frontend_attitude_weak_geometry_info", frontend_attitude_weak_geometry_info, 0.0);
  nh.param<double>("rtk/frontend_attitude_weak_release_factor", frontend_attitude_weak_release_factor, 1.5);
  nh.param<double>("rtk/frontend_attitude_weak_tilt_deg", frontend_attitude_weak_tilt_deg, 0.0);
  if (!std::isfinite(frontend_attitude_weak_geometry_info) || frontend_attitude_weak_geometry_info < 0.0 ||
      !std::isfinite(frontend_attitude_weak_release_factor) || frontend_attitude_weak_release_factor < 1.0)
    throw std::runtime_error("rtk.frontend_attitude_weak_geometry_info must be >= 0 and weak_release_factor >= 1");
  if (frontend_attitude_axis_mask.size() != 3 || std::any_of(frontend_attitude_axis_mask.begin(),
      frontend_attitude_axis_mask.end(), [](double v) { return !(v == 0.0 || v == 1.0); }))
    throw std::runtime_error("rtk.frontend_attitude_axis_mask must be three 0/1 values (body x,y,z)");
  if (!std::isfinite(frontend_attitude_sigma_deg) || frontend_attitude_sigma_deg <= 0.0 ||
      !std::isfinite(frontend_attitude_huber_deg) || frontend_attitude_huber_deg <= 0.0 ||
      !std::isfinite(frontend_attitude_pull_fraction) || frontend_attitude_pull_fraction < 0.0 ||
      frontend_attitude_pull_fraction >= 1.0)
    throw std::runtime_error("rtk.frontend_attitude_sigma_deg/huber_deg must be positive and pull_fraction in [0,1)");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<bool>("common/img_en", img_en, true);
  nh.param<bool>("common/lidar_en", lidar_en, true);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");

  nh.param<bool>("multi_lidar/enabled", multi_lidar_enabled, false);
  nh.param<string>("multi_lidar/calibration_file",
                   multi_lidar_calibration_file, "");
  nh.param<vector<string>>(
      "multi_lidar/topics", multi_lidar_topics,
      vector<string>{"/front_lidar"});
  nh.param<vector<string>>(
      "multi_lidar/transform_keys", multi_lidar_transform_keys,
      vector<string>{"transform_matrix_lidar_6"});
  nh.param<double>("multi_lidar/synchronization_tolerance_sec",
                   multi_lidar_sync_tolerance, 0.005);
  int multi_lidar_queue_size_parameter = 5;
  nh.param<int>("multi_lidar/queue_size",
                multi_lidar_queue_size_parameter, 5);
  if (multi_lidar_enabled)
  {
    if (multi_lidar_topics.empty() ||
        multi_lidar_topics.size() != multi_lidar_transform_keys.size())
      throw std::runtime_error(
          "multi_lidar.topics and transform_keys must be non-empty and "
          "have the same length.");
    if (multi_lidar_calibration_file.empty())
      throw std::runtime_error(
          "multi_lidar.calibration_file must not be empty.");
    if (!std::isfinite(multi_lidar_sync_tolerance) ||
        multi_lidar_sync_tolerance <= 0.0)
      throw std::runtime_error(
          "multi_lidar.synchronization_tolerance_sec must be positive.");
    if (multi_lidar_queue_size_parameter <= 0)
      throw std::runtime_error("multi_lidar.queue_size must be positive.");
    multi_lidar_queue_size =
        static_cast<std::size_t>(multi_lidar_queue_size_parameter);

    const std::set<string> unique_topics(
        multi_lidar_topics.begin(), multi_lidar_topics.end());
    const std::set<string> unique_keys(
        multi_lidar_transform_keys.begin(),
        multi_lidar_transform_keys.end());
    if (unique_topics.size() != multi_lidar_topics.size())
      throw std::runtime_error("multi_lidar.topics contains duplicates.");
    if (unique_keys.size() != multi_lidar_transform_keys.size())
      throw std::runtime_error(
          "multi_lidar.transform_keys contains duplicates.");

    nh.param<double>("multi_lidar/body_filter/minimum_z",
                     multi_lidar_body_exclusion_min_z, -3.0);
    if (!std::isfinite(multi_lidar_body_exclusion_min_z))
      throw std::runtime_error(
          "multi_lidar.body_filter.minimum_z must be finite.");

    multi_lidar_body_exclusion_rectangles.clear();
    multi_lidar_body_exclusion_rectangles.resize(
        multi_lidar_topics.size());
    std::set<string> body_filter_parameter_names;
    for (std::size_t source_index = 0;
         source_index < multi_lidar_topics.size(); ++source_index)
    {
      string topic_parameter_name = multi_lidar_topics[source_index];
      while (!topic_parameter_name.empty() &&
             topic_parameter_name.front() == '/')
        topic_parameter_name.erase(topic_parameter_name.begin());
      std::replace(topic_parameter_name.begin(), topic_parameter_name.end(),
                   '/', '_');
      if (topic_parameter_name.empty() ||
          !body_filter_parameter_names.insert(topic_parameter_name).second)
        throw std::runtime_error(
            "Multi-LiDAR topics do not produce unique body-filter parameter "
            "names.");

      vector<double> rectangle_coordinates;
      nh.param<vector<double>>(
          "multi_lidar/body_filter/" + topic_parameter_name,
          rectangle_coordinates, vector<double>{});
      if (rectangle_coordinates.size() % 4 != 0)
        throw std::runtime_error(
            "multi_lidar.body_filter." + topic_parameter_name +
            " must contain x1,y1,x2,y2 for each rectangle.");

      auto &rectangles =
          multi_lidar_body_exclusion_rectangles[source_index];
      rectangles.reserve(rectangle_coordinates.size() / 4);
      for (std::size_t coordinate_index = 0;
           coordinate_index < rectangle_coordinates.size();
           coordinate_index += 4)
      {
        const double x1 = rectangle_coordinates[coordinate_index];
        const double y1 = rectangle_coordinates[coordinate_index + 1];
        const double x2 = rectangle_coordinates[coordinate_index + 2];
        const double y2 = rectangle_coordinates[coordinate_index + 3];
        if (!std::isfinite(x1) || !std::isfinite(y1) ||
            !std::isfinite(x2) || !std::isfinite(y2))
          throw std::runtime_error(
              "multi_lidar.body_filter." + topic_parameter_name +
              " contains a non-finite coordinate.");
        rectangles.push_back({std::min(x1, x2), std::max(x1, x2),
                              std::min(y1, y2), std::max(y1, y2)});
      }
    }

    // Static-start extrinsic self-check: ground overlap with the reference
    // LiDAR observes roll/pitch/height of every other LiDAR; small deviations
    // are corrected in place, larger ones are only reported.
    auto &precheck = extrinsic_precheck_config_;
    nh.param<bool>("multi_lidar/precheck/enabled", precheck.enabled, false);
    nh.param<bool>("multi_lidar/precheck/apply", precheck.apply, true);
    nh.param<int>("multi_lidar/precheck/min_frames", precheck.min_frames, 5);
    nh.param<int>("multi_lidar/precheck/max_frames", precheck.max_frames, 12);
    nh.param<double>("multi_lidar/precheck/max_rotation_deg",
                     precheck.max_rotation_deg, 1.0);
    nh.param<double>("multi_lidar/precheck/max_height_m",
                     precheck.max_height_m, 0.15);
    nh.param<double>("multi_lidar/precheck/min_apply_rotation_deg",
                     precheck.min_apply_rotation_deg, 0.2);
    nh.param<double>("multi_lidar/precheck/min_apply_height_m",
                     precheck.min_apply_height_m, 0.04);
    nh.param<double>("multi_lidar/precheck/static_gyro_deg_s",
                     precheck.static_gyro_deg_s, 0.5);
    nh.param<double>("multi_lidar/precheck/static_acc_std_mps2",
                     precheck.static_acc_std_mps2, 0.4);
    nh.param<double>("multi_lidar/precheck/cell_size_m",
                     precheck.cell_size_m, 1.5);
    nh.param<bool>("multi_lidar/precheck/allow_extrapolated_ground",
                   precheck.allow_extrapolated_ground, false);
    nh.param<string>("multi_lidar/precheck/reference_topic",
                     extrinsic_precheck_reference_topic_, "/front_lidar");
    vector<double> precheck_body_box;
    nh.param<vector<double>>("multi_lidar/precheck/body_box",
                             precheck_body_box,
                             vector<double>{-2.8, 2.8, -4.5, 9.5});
    if (precheck_body_box.size() != 4)
      throw std::runtime_error(
          "multi_lidar.precheck.body_box must contain x_min,x_max,y_min,y_max.");
    for (int i = 0; i < 4; ++i) precheck.body_box_m[i] = precheck_body_box[i];
    if (precheck.min_frames < 1 || precheck.max_frames < precheck.min_frames ||
        precheck.max_rotation_deg <= 0.0 || precheck.max_height_m <= 0.0 ||
        precheck.cell_size_m <= 0.0)
      throw std::runtime_error(
          "multi_lidar.precheck frame counts, gates and cell size must be positive.");
  }

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  // > 0: bound the visual information per frame to this multiple of the LiDAR information
  // (see VIOManager::parityCovarianceScale); 0 = original fixed-weight LIVO update.
  nh.param<double>("vio/lidar_parity_ratio", vio_lidar_parity_ratio, 0.0);
  // > 0: half size (m) of the visual sparse map cube, independent of the LiDAR map cube (0 = same cube).
  // Full-HD LIVO at 10 fps otherwise retains ~40 MB of reference images per second of driving.
  nh.param<double>("vio/visual_half_map_m", vio_visual_half_map_m, 0.0);
  nh.param<double>("vio/visual_behind_m", vio_visual_behind_m, 0.0);
  nh.param<double>("vio/new_points_min_move_m", vio_new_points_min_move_m, 0.1);
  nh.param<double>("vio/new_points_min_turn_deg", vio_new_points_min_turn_deg, 0.5);
  if (!(vio_new_points_min_move_m >= 0.0) || !(vio_new_points_min_turn_deg >= 0.0))
    throw std::runtime_error("vio/new_points_min_move_m and vio/new_points_min_turn_deg must be nonnegative");
  if (!std::isfinite(vio_visual_behind_m) || vio_visual_behind_m < 0.0)
    throw std::runtime_error("vio/visual_behind_m must be finite and nonnegative (0 = cube only)");
  if (!std::isfinite(vio_visual_half_map_m) || vio_visual_half_map_m < 0.0)
    throw std::runtime_error("vio/visual_half_map_m must be finite and nonnegative (0 = LiDAR map cube)");
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);
  // false: camera colours the LiDAR map only; the estimator is pure LIO.
  nh.param<bool>("vio/ekf_update_en", vio_ekf_update_en, true);
  nh.param<bool>("debug/print_frame_timing", print_frame_timing, false);
  nh.param<double>("debug/status_interval_sec", status_interval_sec, 10.0);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<double>("imu/gyr_bias_rw_sigma", gyr_bias_rw_sigma, 0.001);
  nh.param<double>("imu/acc_bias_rw_sigma", acc_bias_rw_sigma, 0.001);
  nh.param<double>("imu/initial_gyr_bias_sigma", initial_gyr_bias_sigma, std::sqrt(1e-5));
  for (double noise : {gyr_cov, acc_cov, gyr_bias_rw_sigma, acc_bias_rw_sigma, initial_gyr_bias_sigma})
    if (!std::isfinite(noise) || noise <= 0)
      throw std::runtime_error("IMU noise parameters must be finite and positive");
  nh.param<double>("imu/maximum_time_gap", imu_max_time_gap, 0.2);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);
  if (!std::isfinite(imu_max_time_gap) || imu_max_time_gap <= 0.0)
    throw std::runtime_error("imu.maximum_time_gap must be finite and positive.");

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  if (!std::isfinite(filter_size_surf_min) || filter_size_surf_min <= 0.0)
    throw std::runtime_error(
        "preprocess.filter_size_surf must be finite and positive.");
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  if (p_pre->N_SCANS <= 0 || p_pre->N_SCANS > 128)
    throw std::runtime_error("preprocess.scan_line must be in [1, 128].");
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);
  nh.param<double>("preprocess/maximum_point_offset_sec",
                   lidar_max_point_offset, 0.2);
  if (!std::isfinite(lidar_max_point_offset) ||
      lidar_max_point_offset <= 0.0)
    throw std::runtime_error(
        "preprocess.maximum_point_offset_sec must be positive.");
  p_pre->maximum_point_offset_sec = lidar_max_point_offset;

  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/type", pcd_save_type, 0);
  nh.param<bool>("image_save/img_save_en", img_save_en, false);
  nh.param<int>("image_save/interval", img_save_interval, 1);

  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  vector<double> rear_axle_lidar_t;
  vector<double> rear_axle_lidar_r;
  nh.param<vector<double>>(
      "extrin_calib/front_lidar_to_rear_axle_T",
      rear_axle_lidar_t, vector<double>());
  nh.param<vector<double>>(
      "extrin_calib/front_lidar_to_rear_axle_R",
      rear_axle_lidar_r, vector<double>());
  if (!rear_axle_lidar_t.empty() || !rear_axle_lidar_r.empty())
  {
    if (rear_axle_lidar_t.size() != 3 || rear_axle_lidar_r.size() != 9)
      throw std::runtime_error(
          "front_lidar_to_rear_axle_T/R must contain 3 and 9 values.");
    extrinT = rear_axle_lidar_t;
    extrinR = rear_axle_lidar_r;
  }
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
  nh.param<vector<double>>("camera/intrinsics", camera_intrinsics,
                          vector<double>{1642.570556640625, 1855.53173828125,
                                         971.0083025529602, 534.2602337509306});
  nh.param<vector<double>>("camera/distortion", camera_distortion,
                          vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  nh.param<int>("camera/width", camera_width, 1920);
  nh.param<int>("camera/height", camera_height, 1080);
  nh.param<int>("camera/frame_divider", camera_frame_divider, 1);
  if (camera_frame_divider < 1)
    throw std::runtime_error("camera.frame_divider must be >= 1.");
  if (multi_lidar_enabled) loadMultiLidarCalibration();
  string imu_message_format;
  nh.param<string>("mine/imu_message_format", imu_message_format, "packed_mine_pose");
  if (imu_message_format == "standard_rfu")
  {
    imu_standard_rfu = true;
  }
  else if (imu_message_format != "packed_mine_pose")
  {
    throw std::runtime_error(
        "mine.imu_message_format must be standard_rfu or packed_mine_pose.");
  }
  nh.param<bool>("mine/imu_gyro_in_degrees", imu_gyro_in_degrees, true);
  nh.param<bool>("mine/imu_acceleration_gravity_compensated", imu_acceleration_gravity_compensated, true);
  nh.param<double>("mine/imu_acceleration_scale", imu_acceleration_scale, 1.0);
  nh.param<bool>("reference_frame_conversion/enabled", rear_axle_to_imu_enabled, false);
  vector<double> antenna_to_rear_axle_values;
  vector<double> imu_to_antenna_values;
  nh.param<vector<double>>(
      "reference_frame_conversion/antenna_to_rear_axle_m",
      antenna_to_rear_axle_values, vector<double>{0.0, 0.0, 0.0});
  nh.param<vector<double>>(
      "reference_frame_conversion/imu_to_antenna_m",
      imu_to_antenna_values, vector<double>{0.0, 0.0, 0.0});
  if (antenna_to_rear_axle_values.size() != 3 || imu_to_antenna_values.size() != 3)
    throw std::runtime_error("reference-frame lever arms must each contain 3 values.");
  imu_to_rear_axle <<
      imu_to_antenna_values[0] + antenna_to_rear_axle_values[0],
      imu_to_antenna_values[1] + antenna_to_rear_axle_values[1],
      imu_to_antenna_values[2] + antenna_to_rear_axle_values[2];
  vector<double> imu_acceleration_transform_values;
  nh.param<vector<double>>(
      "mine/imu_acceleration_transform", imu_acceleration_transform_values,
      vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  if (imu_acceleration_transform_values.size() != 9)
    throw std::runtime_error("mine.imu_acceleration_transform must contain 9 row-major values.");
  imu_acceleration_transform <<
      imu_acceleration_transform_values[0], imu_acceleration_transform_values[1], imu_acceleration_transform_values[2],
      imu_acceleration_transform_values[3], imu_acceleration_transform_values[4], imu_acceleration_transform_values[5],
      imu_acceleration_transform_values[6], imu_acceleration_transform_values[7], imu_acceleration_transform_values[8];
  nh.param<double>("debug/plot_time", plot_time, -10);
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<bool>("publish/rgb_flat_field_en", rgb_flat_field_en, true);
  RCLCPP_INFO(node_->get_logger(),
              "RGB colouring: paired synchronized image and scan only (no cached camera views)");
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::loadMultiLidarCalibration()
{
  const std::filesystem::path requested_path(multi_lidar_calibration_file);
  if (!std::filesystem::exists(requested_path))
    throw std::runtime_error(
        "Multi-LiDAR calibration file does not exist: " +
        requested_path.string());
  const std::filesystem::path calibration_path =
      std::filesystem::canonical(requested_path);
  multi_lidar_calibration_file = calibration_path.string();

  std::ifstream stream(calibration_path);
  if (!stream.is_open())
    throw std::runtime_error(
        "Cannot open Multi-LiDAR calibration: " +
        calibration_path.string());
  Json::CharReaderBuilder builder;
  Json::Value calibration;
  string parse_errors;
  if (!Json::parseFromStream(
          builder, stream, &calibration, &parse_errors))
    throw std::runtime_error(
        "Cannot parse Multi-LiDAR calibration " +
        calibration_path.string() + ": " + parse_errors);
  if (calibration["lidar_transform_semantics"].asString() !=
      "lidar_to_body_rear_axle")
    throw std::runtime_error(
        "Calibration lidar_transform_semantics must be "
        "lidar_to_body_rear_axle.");

  const auto parse_rigid_transform = [](
      const Json::Value &values, const string &name) {
    if (!values.isArray() || values.size() != 16)
      throw std::runtime_error(
          name + " must contain 16 row-major numeric values.");
    Eigen::Matrix4d transform;
    for (Json::ArrayIndex index = 0; index < 16; ++index)
    {
      if (!values[index].isNumeric())
        throw std::runtime_error(name + " contains a non-numeric value.");
      const double value = values[index].asDouble();
      if (!std::isfinite(value))
        throw std::runtime_error(name + " contains a non-finite value.");
      transform(static_cast<int>(index / 4),
                static_cast<int>(index % 4)) = value;
    }
    const Eigen::Vector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
    if (!transform.row(3).transpose().isApprox(expected_last_row, 1.0e-12))
      throw std::runtime_error(name + " is not a homogeneous transform.");
    const M3D rotation = transform.block<3, 3>(0, 0);
    if (!(rotation.transpose() * rotation).isApprox(M3D::Identity(), 2.0e-3) ||
        std::abs(rotation.determinant() - 1.0) > 2.0e-3)
      throw std::runtime_error(name + " rotation is not a proper rotation.");
    return transform;
  };

  const Json::Value &lidar_entries = calibration["lidar"];
  if (!lidar_entries.isArray() || lidar_entries.empty())
    throw std::runtime_error(
        "Calibration must contain a non-empty lidar array.");

  lidar_sources.clear();
  lidar_sources.reserve(multi_lidar_topics.size());
  for (std::size_t source_index = 0;
       source_index < multi_lidar_topics.size(); ++source_index)
  {
    const string &transform_key =
        multi_lidar_transform_keys[source_index];
    const Json::Value *matrix_values = nullptr;
    for (const auto &entry : lidar_entries)
    {
      if (entry.isObject() && entry.isMember(transform_key))
      {
        matrix_values = &entry[transform_key];
        break;
      }
    }
    if (matrix_values == nullptr)
      throw std::runtime_error(
          "Calibration is missing " + transform_key + ".");

    const Eigen::Matrix4d rear_from_lidar =
        parse_rigid_transform(*matrix_values, transform_key);
    auto source = std::make_unique<LidarSource>();
    source->topic = multi_lidar_topics[source_index];
    source->transform_key = transform_key;
    source->rear_from_lidar_rotation =
        rear_from_lidar.block<3, 3>(0, 0);
    source->rear_from_lidar_translation =
        rear_from_lidar.block<3, 1>(0, 3);
    source->body_exclusion_rectangles =
        multi_lidar_body_exclusion_rectangles[source_index];
    lidar_sources.push_back(std::move(source));
  }

  // The synchronized cloud is represented in the rear-axle frame.  It is a
  // virtual LiDAR input whose LiDAR->rear transform is therefore identity.
  // initializeComponents() optionally composes rear->IMU afterwards.
  extrinT = {0.0, 0.0, 0.0};
  extrinR = {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0};

  // VIO consumes virtual-lidar(rear axle)->camera.  The same external file
  // stores camera->rear axle, so invert that rigid transform here.
  const Json::Value &rear_from_camera_values =
      calibration["transforms"]["midrange_camera_to_rear_axle"]
                 ["transform_matrix"];
  const Eigen::Matrix4d rear_from_camera = parse_rigid_transform(
      rear_from_camera_values, "midrange_camera_to_rear_axle");
  rear_from_camera_ = rear_from_camera;
  calibration_file_ = calibration_path.string();
  if (calibration.isMember("identity") && calibration["identity"].isObject() &&
      calibration["identity"].isMember("id"))
    calibration_identity_ = calibration["identity"]["id"].asString();
  if (calibration.isMember("calibration_type"))
    calibration_type_ = calibration["calibration_type"].asString();
  const M3D camera_from_rear_rotation =
      rear_from_camera.block<3, 3>(0, 0).transpose();
  const V3D camera_from_rear_translation =
      -camera_from_rear_rotation * rear_from_camera.block<3, 1>(0, 3);
  // Eigen stores matrices column-major; the downstream MAT_FROM_ARRAY macro
  // expects row-major parameter order.
  cameraextrinR = {
      camera_from_rear_rotation(0, 0), camera_from_rear_rotation(0, 1),
      camera_from_rear_rotation(0, 2), camera_from_rear_rotation(1, 0),
      camera_from_rear_rotation(1, 1), camera_from_rear_rotation(1, 2),
      camera_from_rear_rotation(2, 0), camera_from_rear_rotation(2, 1),
      camera_from_rear_rotation(2, 2)};
  cameraextrinT = {
      camera_from_rear_translation.x(),
      camera_from_rear_translation.y(),
      camera_from_rear_translation.z()};

  // Optional sensor sections of the unified calibration file (schema >= 6).
  // Camera intrinsics and the per-truck time offsets are calibration data,
  // so they travel with the extrinsics; when present they override the
  // algorithm yaml and the source is logged.
  const auto read_numbers = [](const Json::Value &values, std::size_t count,
                               const string &name) {
    if (!values.isArray() || values.size() != count)
      throw std::runtime_error(name + " must contain " + std::to_string(count) + " numbers.");
    vector<double> out;
    for (const auto &value : values)
    {
      if (!value.isNumeric() || !std::isfinite(value.asDouble()))
        throw std::runtime_error(name + " contains a non-numeric value.");
      out.push_back(value.asDouble());
    }
    return out;
  };
  if (calibration.isMember("camera") && calibration["camera"].isObject())
  {
    const Json::Value &camera = calibration["camera"];
    if (camera.isMember("intrinsics"))
      camera_intrinsics = read_numbers(camera["intrinsics"], 4, "camera.intrinsics");
    if (camera.isMember("distortion"))
      camera_distortion = read_numbers(camera["distortion"], 5, "camera.distortion");
    if (camera.isMember("width")) camera_width = camera["width"].asInt();
    if (camera.isMember("height")) camera_height = camera["height"].asInt();
    RCLCPP_INFO(node_->get_logger(),
                "Camera model from calibration file: %dx%d fx %.2f fy %.2f cx %.2f cy %.2f "
                "distortion [%.4f %.4f %.4f %.4f %.4f]",
                camera_width, camera_height, camera_intrinsics[0], camera_intrinsics[1],
                camera_intrinsics[2], camera_intrinsics[3], camera_distortion[0],
                camera_distortion[1], camera_distortion[2], camera_distortion[3],
                camera_distortion[4]);
    if (camera.isMember("flat_field") && camera["flat_field"].isObject())
    {
      const Json::Value &flat = camera["flat_field"];
      if (flat.get("model", "").asString() != "poly8")
        throw std::runtime_error("camera.flat_field.model must be poly8");
      flat_field_coefficients_ = read_numbers(flat["coefficients"], 8, "camera.flat_field.coefficients");
      const vector<double> range = read_numbers(flat["y_range"], 2, "camera.flat_field.y_range");
      flat_field_y_lo_ = range[0];
      flat_field_y_hi_ = range[1];
      RCLCPP_INFO(node_->get_logger(), "Camera flat field from calibration file: poly8, y range [%.2f, %.2f]",
                  flat_field_y_lo_, flat_field_y_hi_);
    }
  }
  if (calibration.isMember("time_offsets") && calibration["time_offsets"].isObject())
  {
    const Json::Value &offsets = calibration["time_offsets"];
    if (offsets.isMember("lidar") && offsets["lidar"].isNumeric())
      lidar_time_offset = offsets["lidar"].asDouble();
    if (offsets.isMember("camera") && offsets["camera"].isNumeric())
      img_time_offset = offsets["camera"].asDouble();
    calib_lidar_ins_offset_ = lidar_time_offset;
    // INS association offsets travel with the calibration as well (the axle
    // stage solves time_offsets.lidar under exactly these two values), so the
    // mapper must not silently use different yaml values.
    if (offsets.isMember("rtk_position") && offsets["rtk_position"].isNumeric())
      calib_rtk_position_offset_ = offsets["rtk_position"].asDouble();
    if (offsets.isMember("ins_attitude") && offsets["ins_attitude"].isNumeric())
      calib_ins_attitude_offset_ = offsets["ins_attitude"].asDouble();
    // Calibration determines BOTH time semantics; the mapper stays a generic tool:
    //   lidar_frontend  scan<->IMU hardware offset -> front-end sync/deskew/image pairing
    //   lidar           scan<->INS alignment (axle stage) -> fusion reference association
    // With lidar_frontend present the front-end runs on the hardware base and the
    // INS-side offsets are shifted by (lidar - lidar_frontend), so the fusion keeps the
    // calibrated association. parameters.txt records the effective values and the
    // evaluation follows automatically.
    if (offsets.isMember("lidar_frontend") && offsets["lidar_frontend"].isNumeric())
    {
      const double frontend = offsets["lidar_frontend"].asDouble();
      const double ins_aligned = lidar_time_offset;
      lidar_time_offset = frontend;
      calib_lidar_frontend_offset_ = frontend;
      const double shift = (offsets.isMember("lidar") && offsets["lidar"].isNumeric())
                               ? (ins_aligned - frontend) : 0.0;
      rtk_reference_time_shift_ = shift;
      RCLCPP_INFO(node_->get_logger(),
                  "Front-end lidar offset %+.4f s (hardware IMU base); INS alignment %+.4f s kept via reference shift %+.4f s",
                  frontend, ins_aligned, shift);
    }
    RCLCPP_INFO(node_->get_logger(),
                "Time offsets from calibration file: lidar %+.4f s, camera %+.4f s",
                lidar_time_offset, img_time_offset);
  }

  string source_description;
  vector<string> precheck_topics;
  vector<Eigen::Matrix4d> precheck_transforms;
  std::size_t precheck_reference = lidar_sources.size();
  for (std::size_t index = 0; index < lidar_sources.size(); ++index)
  {
    if (index != 0) source_description += ", ";
    source_description += lidar_sources[index]->topic + "->" +
                          lidar_sources[index]->transform_key;
    Eigen::Matrix4d rear_from_lidar = Eigen::Matrix4d::Identity();
    rear_from_lidar.block<3, 3>(0, 0) = lidar_sources[index]->rear_from_lidar_rotation;
    rear_from_lidar.block<3, 1>(0, 3) = lidar_sources[index]->rear_from_lidar_translation;
    precheck_topics.push_back(lidar_sources[index]->topic);
    precheck_transforms.push_back(rear_from_lidar);
    if (lidar_sources[index]->topic == extrinsic_precheck_reference_topic_)
      precheck_reference = index;
  }
  RCLCPP_INFO(
      node_->get_logger(),
      "Loaded %zu LiDAR source(s) from %s: %s",
      lidar_sources.size(), multi_lidar_calibration_file.c_str(),
      source_description.c_str());

  if (extrinsic_precheck_config_.enabled && lidar_sources.size() >= 2)
  {
    if (precheck_reference >= lidar_sources.size())
    {
      precheck_reference = lidar_sources.size() - 1;
      RCLCPP_WARN(node_->get_logger(),
                  "Extrinsic precheck reference topic %s is not configured; using %s",
                  extrinsic_precheck_reference_topic_.c_str(),
                  lidar_sources[precheck_reference]->topic.c_str());
    }
    extrinsic_precheck_.configure(extrinsic_precheck_config_, precheck_topics,
                                  precheck_transforms, precheck_reference);
    RCLCPP_INFO(node_->get_logger(),
                "Extrinsic precheck armed: reference %s, %d-%d static frames, gates "
                "%.2f deg / %.3f m, %s",
                lidar_sources[precheck_reference]->topic.c_str(),
                extrinsic_precheck_config_.min_frames, extrinsic_precheck_config_.max_frames,
                extrinsic_precheck_config_.max_rotation_deg,
                extrinsic_precheck_config_.max_height_m,
                extrinsic_precheck_config_.apply ? "corrections inside the gates are applied"
                                                 : "check only");
  }
  else
    extrinsic_precheck_.configure(livo::ExtrinsicPrecheckConfig{}, precheck_topics,
                                  precheck_transforms, 0);
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  if (extrinT.size() != 3 || extrinR.size() != 9)
    throw std::runtime_error("LiDAR body extrinsic T/R must contain 3 and 9 values.");
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  // The truck calibration is T_rear_axle_lidar. When enabled, relocate the
  // estimator body to the physical IMU centre without changing the LIO/IEKF.
  // All involved vehicle frames use the same RFU axes, so only translation
  // changes: T_imu_lidar = T_imu_rear_axle * T_rear_axle_lidar.
  if (rear_axle_to_imu_enabled)
  {
    extT += imu_to_rear_axle;
  }

  voxelmap_manager->extT_ = extT;
  voxelmap_manager->extR_ = extR;

  RCLCPP_INFO(
      node_->get_logger(),
      "Wuhu LiDAR time contract: header=scan-start, point timestamp=relative-seconds; "
      "rear_axle_to_imu=%s; active lidar->body T=[%.3f, %.3f, %.3f] m; "
      "IMU format=%s, gyro=%s, acceleration=%s",
      rear_axle_to_imu_enabled ? "true" : "false",
      extT.x(), extT.y(), extT.z(),
      imu_standard_rfu ? "standard_rfu" : "packed_mine_pose",
      imu_gyro_in_degrees ? "deg/s->rad/s" : "rad/s",
      imu_acceleration_gravity_compensated
          ? "gravity-free; restoring gravity"
          : "specific force with gravity");

  if (camera_intrinsics.size() != 4 || camera_distortion.size() != 5)
    throw std::runtime_error("Camera intrinsics must be [fx, fy, cx, cy] and distortion must contain five values.");
  cv::Mat distortion(1, 5, CV_64F, camera_distortion.data());
  camera_model = std::make_unique<vk::PinholeCamera>(
      camera_width, camera_height, camera_intrinsics[0], camera_intrinsics[1],
      camera_intrinsics[2], camera_intrinsics[3], distortion);
  vio_manager->cam = camera_model.get();

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->lidar_parity_ratio = vio_lidar_parity_ratio;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->ekf_update_en = vio_ekf_update_en;
  vio_manager->print_frame_timing = print_frame_timing;
  vio_manager->new_points_min_move_m = vio_new_points_min_move_m;
  vio_manager->new_points_min_turn_deg = vio_new_points_min_turn_deg;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  // Bias random walks are continuous spectral densities; propagation uses
  // dt, unlike the discrete sample gyro/acceleration noises using dt^2.
  p_imu->set_gyr_bias_cov(V3D::Constant(gyr_bias_rw_sigma * gyr_bias_rw_sigma));
  p_imu->set_acc_bias_cov(V3D::Constant(acc_bias_rw_sigma * acc_bias_rw_sigma));
  p_imu->initial_gyr_bias_sigma = initial_gyr_bias_sigma;
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  std::filesystem::create_directories(root_dir + "Log/result");
  std::filesystem::create_directories(root_dir + "Log/pcd");
  std::filesystem::create_directories(root_dir + "Log/image");
  std::filesystem::create_directories(root_dir + "Log/Colmap/sparse/0");
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(root_dir + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(root_dir + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(root_dir + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(root_dir + "Log/mat_pre.txt", std::ios::out);
  fout_out.open(root_dir + "Log/mat_out.txt", std::ios::out);
  p_imu->fout_imu.open(root_dir + "Log/imu.txt", std::ios::out);
  // RTK uses the native LiDAR Hessian as an observability diagnostic in both
  // LIO and LIVO.  Keep collecting it whenever the backend is enabled; the
  // CSV itself remains a LIO-only diagnostic to avoid changing file output.
  if (rtk_enabled) voxelmap_manager->collect_geometry_diagnostics_ = true;
  if (vio_lidar_parity_ratio > 0) voxelmap_manager->collect_geometry_diagnostics_ = true;
  if (!img_en && lidar_en && imu_en)
  {
    voxelmap_manager->collect_geometry_diagnostics_ = true;
    lio_geometry_log.exceptions(std::ios::badbit | std::ios::failbit);
    lio_geometry_log.open(root_dir + "Log/lio_geometry.csv");
    attitude_gate_log.open(root_dir + "Log/attitude_gate.csv");
    attitude_gate_log << "stamp,weakest_translation_information,roll_pitch_disagreement_deg,weak_geometry,aiding\n";
    lio_geometry_log << "stamp,effective_points,point_plane_rms_m,rotation_radius_m";
    for (int i = 0; i < 6; ++i) lio_geometry_log << ",lambda" << i;
    lio_geometry_log << ",lambda_min_max_ratio";
    for (int row = 0; row < 6; ++row) for (int column = row; column < 6; ++column)
      lio_geometry_log << ",body_h" << row << column;
    lio_geometry_log << ",gravity_x,gravity_y,gravity_z\n";
  }
}

void LIVMapper::initializeSubscribersAndPublishers()
{
  bool reliable_input_qos = false;
  Ros2ParameterReader nh(node_);
  nh.param<bool>("mine/reliable_input_qos", reliable_input_qos, false);
  // rosbag2 may publish a whole MCAP chunk in a short burst.  Keep enough
  // history for offline replay. Reliability remains configurable so live
  // best-effort drivers are not made incompatible by the ROS 2 adapter.
  rclcpp::QoS lidar_qos(rclcpp::KeepLast(200));
  rclcpp::QoS imu_qos(rclcpp::KeepLast(5000));
  rclcpp::QoS image_qos(rclcpp::KeepLast(100));
  if (reliable_input_qos)
  {
    lidar_qos.reliable();
    imu_qos.reliable();
    image_qos.reliable();
  }
  else
  {
    lidar_qos.best_effort();
    imu_qos.best_effort();
    image_qos.best_effort();
  }
  if (multi_lidar_enabled)
  {
    sub_pcls.reserve(lidar_sources.size());
    for (std::size_t source_index = 0;
         source_index < lidar_sources.size(); ++source_index)
    {
      sub_pcls.push_back(
          node_->create_subscription<sensor_msgs::PointCloud2>(
              lidar_sources[source_index]->topic, lidar_qos,
              [this, source_index](
                  const sensor_msgs::PointCloud2::ConstSharedPtr msg) {
                multi_lidar_pcl_cbk(msg, source_index);
              }));
    }
  }
  else
  {
    sub_pcl = node_->create_subscription<sensor_msgs::PointCloud2>(
        lid_topic, lidar_qos,
        std::bind(&LIVMapper::standard_pcl_cbk, this,
                  std::placeholders::_1));
  }
  sub_imu = node_->create_subscription<sensor_msgs::Imu>(
      imu_topic, imu_qos, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  if (imu_standard_rfu)
  {
    sub_ins_odom = node_->create_subscription<nav_msgs::Odometry>(
        ins_odom_topic, imu_qos,
        std::bind(&LIVMapper::ins_odom_cbk, this, std::placeholders::_1));
  }
  if (rtk_fusion_)
  {
    string status_topic;
    nh.param<string>("rtk/status_topic", status_topic, "/imu_data/ins_status");
    sub_ins_status = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        status_topic, imu_qos,
        std::bind(&LIVMapper::ins_status_cbk, this, std::placeholders::_1));
  }
  sub_img = node_->create_subscription<sensor_msgs::Image>(
      img_topic, image_qos, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));

  pubLaserCloudFullRes = node_->create_publisher<sensor_msgs::PointCloud2>("/cloud_registered", 10);
  pubCurrentFrame = node_->create_publisher<sensor_msgs::PointCloud2>("/cloud_current_frame", 10);
  pubNormal = node_->create_publisher<visualization_msgs::MarkerArray>("visualization_marker", 10);
  pubSubVisualMap = node_->create_publisher<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 10);
  pubLaserCloudEffect = node_->create_publisher<sensor_msgs::PointCloud2>("/cloud_effected", 10);
  pubLaserCloudMap = node_->create_publisher<sensor_msgs::PointCloud2>("/Laser_map", 10);
  pubOdomAftMapped = node_->create_publisher<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = node_->create_publisher<nav_msgs::Path>("/path", 10);
  plane_pub = node_->create_publisher<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = node_->create_publisher<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = node_->create_publisher<sensor_msgs::PointCloud2>("/dyn_obj", 10);
  pubLaserCloudDynRmed = node_->create_publisher<sensor_msgs::PointCloud2>("/dyn_obj_removed", 10);
  pubLaserCloudDynDbg = node_->create_publisher<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 10);
  mavros_pose_publisher = node_->create_publisher<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = image_transport::create_publisher(node_.get(), "/rgb_img");
  pubImuPropOdom = node_->create_publisher<nav_msgs::Odometry>("/LIVO2/imu_propagate", 100);
  pubImuReferencePath = node_->create_publisher<nav_msgs::Path>("/imu_reference_path", 10);
  pubImuReferenceOdom = node_->create_publisher<nav_msgs::Odometry>("/imu_reference_odom", 10);
  imu_prop_timer = node_->create_wall_timer(std::chrono::milliseconds(4), std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_ = node_->create_publisher<visualization_msgs::MarkerArray>("/planes", 10);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  const bool was_initializing = p_imu->imu_need_init;
  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();
  if (was_initializing && !p_imu->imu_need_init && !mine_frame_initialized)
  {
    // Use the exact measurement-group endpoint that Process2 used to finish
    // IMU initialization. lidar_frame_end_time is populated for complete
    // ONLY_LIO scans, but remains zero in LIVO where LiDAR data is cut at
    // image timestamps; using it there rejects every valid INS sample.
    const MeasureGroup &initialization_measure =
        LidarMeasures.measures.back();
    const double initialization_time =
        LidarMeasures.lio_vio_flg == LIO
            ? initialization_measure.lio_time
            : initialization_measure.vio_time;
    initializeMineFrame(initialization_time);
    if (rtk_fusion_)
      rtk_fusion_->initializeGravity(initialization_time, _state.gravity,
          _state.rot_end * p_imu->initial_gravity_covariance * _state.rot_end.transpose());
  }

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::initializeMineFrame(double initialization_time)
{
  V3D mean_position = V3D::Zero();
  Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
  Eigen::Quaterniond reference = Eigen::Quaterniond::Identity();
  std::size_t sample_count = 0;

  for (const MinePose &sample : mine_pose_samples)
  {
    if (sample.stamp > initialization_time) break;
    if (rtk_fusion_ && rtk_observations_enabled)
    {
      RtkFusion::Observation quality_checked;
      const double time = sample.stamp + imu_time_offset - rtk_fusion_->time_offset;
      if (!rtk_fusion_->referenceAt(time, quality_checked)) continue;
    }
    if (sample_count == 0) reference = sample.orientation;
    Eigen::Quaterniond aligned = sample.orientation;
    if (reference.coeffs().dot(aligned.coeffs()) < 0.0) aligned.coeffs() *= -1.0;
    quaternion_sum += aligned.coeffs();
    mean_position += sample.position;
    ++sample_count;
  }

  if (sample_count == 0)
  {
    if (rtk_fusion_)
    {
      rtk_fusion_->local_frontend = true;
      rtk_fusion_->frontend_alignment_known = false;
      rtk_fusion_->graph_in_enu = false;
      lio_world_frame = "lio_local";
      voxelmap_manager->world_frame_ = lio_world_frame;
      mine_frame_initialized = true;
      mine_initialization_time = initialization_time;
      RCLCPP_WARN(node_->get_logger(), "No stable mode-4 RTK at startup: continuing local LIO; global ENU alignment is pending");
      return;
    }
    throw std::runtime_error(
        "No INS odometry pose was available at IMU initialization time.");
  }

  mean_position /= static_cast<double>(sample_count);
  quaternion_sum.normalize();
  Eigen::Quaterniond mean_orientation(
      quaternion_sum.w(), quaternion_sum.x(),
      quaternion_sum.y(), quaternion_sum.z());
  mean_orientation.normalize();
  const M3D mine_rotation = mean_orientation.toRotationMatrix();

  // Initial world-frame change precedes mapping. Subsequent optional RTK
  // aiding joins the measurement update, never overwrites a mapped pose.
  _state.pos_end = mine_rotation * _state.pos_end + mean_position;
  _state.rot_end = mine_rotation * _state.rot_end;
  _state.vel_end = mine_rotation * _state.vel_end;
  _state.gravity = mine_rotation * _state.gravity;

  // Right-multiplicative attitude errors stay in the body tangent space under
  // this left world-frame change. Position, velocity and gravity errors are
  // world vectors, so rotate their complete covariance rows/columns as well.
  MD(DIM_STATE, DIM_STATE) transform_jacobian =
      MD(DIM_STATE, DIM_STATE)::Identity();
  transform_jacobian.block<3, 3>(3, 3) = mine_rotation;
  transform_jacobian.block<3, 3>(7, 7) = mine_rotation;
  transform_jacobian.block<3, 3>(16, 16) = mine_rotation;
  _state.cov =
      transform_jacobian * _state.cov * transform_jacobian.transpose();
  state_propagat = _state;
  voxelmap_manager->state_ = _state;

  mine_initialization_time = initialization_time;
  mine_frame_initialized = true;
  mine_pose_publish_index = 0;
  while (mine_pose_publish_index < mine_pose_samples.size() &&
         mine_pose_samples[mine_pose_publish_index].stamp <= initialization_time)
  {
    ++mine_pose_publish_index;
  }

  geometry_msgs::PoseStamped initial_pose;
  initial_pose.header.frame_id = "mine";
  initial_pose.header.stamp = stampFromSec(initialization_time);
  initial_pose.pose.position.x = mean_position.x();
  initial_pose.pose.position.y = mean_position.y();
  initial_pose.pose.position.z = mean_position.z();
  initial_pose.pose.orientation.x = mean_orientation.x();
  initial_pose.pose.orientation.y = mean_orientation.y();
  initial_pose.pose.orientation.z = mean_orientation.z();
  initial_pose.pose.orientation.w = mean_orientation.w();
  imu_reference_path.header.frame_id = "mine";
  imu_reference_path.poses.clear();
  imu_reference_path.poses.push_back(initial_pose);

  RCLCPP_INFO(
      node_->get_logger(),
      "Mine frame initialized from %zu stationary INS samples at [%.3f, %.3f, %.3f]",
      sample_count, mean_position.x(), mean_position.y(), mean_position.z());
}

void LIVMapper::publishReferenceTrajectory(double current_time)
{
  if (!mine_frame_initialized) return;

  const MinePose *latest = nullptr;
  while (mine_pose_publish_index < mine_pose_samples.size() &&
         mine_pose_samples[mine_pose_publish_index].stamp <= current_time)
  {
    const MinePose &sample = mine_pose_samples[mine_pose_publish_index++];
    if (sample.stamp <= mine_initialization_time) continue;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "mine";
    pose.header.stamp = stampFromSec(sample.stamp);
    pose.pose.position.x = sample.position.x();
    pose.pose.position.y = sample.position.y();
    pose.pose.position.z = sample.position.z();
    pose.pose.orientation.x = sample.orientation.x();
    pose.pose.orientation.y = sample.orientation.y();
    pose.pose.orientation.z = sample.orientation.z();
    pose.pose.orientation.w = sample.orientation.w();
    imu_reference_path.poses.push_back(pose);
    latest = &sample;
  }

  imu_reference_path.header.stamp = stampFromSec(current_time);
  pubImuReferencePath->publish(imu_reference_path);
  if (latest == nullptr || imu_reference_path.poses.empty()) return;

  const auto &pose = imu_reference_path.poses.back();
  imu_reference_odom.header = pose.header;
  imu_reference_odom.child_frame_id = "imu_reference";
  imu_reference_odom.pose.pose = pose.pose;
  pubImuReferenceOdom->publish(imu_reference_odom);
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  frame_profile::Stopwatch fp;
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  if (print_frame_timing)
    std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  // LiDAR information of the slice just registered by handleLIO: the parity bound of the
  // photometric update is evaluated against this frame's own LiDAR conditioning.
  vio_manager->lidar_information = voxelmap_manager->lidar_information_body_;
  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);
  fp.lap("vio.process_frame");

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  fp.lap("vio.publish_colour_world");
  if (rtk_fusion_ && slam_mode_ == LIVO && pending_rtk_livo_frame_.valid)
  {
    // scan_colours_ follow pcl_w_wait_pub, i.e. the pending cloud in the world frame (dense_map_en)
    if (scan_colours_.size() != pending_rtk_livo_frame_.cloud->size())
      throw std::runtime_error("LIVO colours are not aligned with the archived scan (RTK fusion needs publish/dense_map_en: true)");
    rtk_fusion_->submit(
        pending_rtk_livo_frame_.stamp, pending_rtk_livo_frame_.begin,
        pending_rtk_livo_frame_.state, pending_rtk_livo_frame_.cloud,
        pending_rtk_livo_frame_.lidar_information, &scan_colours_);
    scan_colours_.clear();
    pending_rtk_livo_frame_.valid = false;
    pending_rtk_livo_frame_.cloud.reset();
  }
  fp.lap("vio.rtk_submit");
  publish_img_rgb(pubImage, vio_manager);
  fp.lap("vio.publish_image");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
  fp.lap("vio.logging");
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  frame_profile::Stopwatch fp;
  double t0 = omp_get_wtime();

  std::unordered_map<DownsampleVoxelKey, DownsampleMetadata,
                     DownsampleVoxelKeyHash> downsample_metadata;
  if (multi_lidar_enabled)
  {
    downsample_metadata.reserve(feats_undistort->size());
    const double inverse_leaf_size = 1.0 / filter_size_surf_min;
    for (const PointType &point : feats_undistort->points)
    {
      const DownsampleVoxelKey key{
          static_cast<std::int64_t>(
              std::floor(point.x * inverse_leaf_size)),
          static_cast<std::int64_t>(
              std::floor(point.y * inverse_leaf_size)),
          static_cast<std::int64_t>(
              std::floor(point.z * inverse_leaf_size))};
      DownsampleMetadata &metadata = downsample_metadata[key];
      metadata.beam_origin_sum +=
          V3D(point.normal_x, point.normal_y, point.normal_z);
      metadata.intensity_sum += point.intensity;
      ++metadata.count;
    }
    // PCL treats normal_* as a unit surface normal and normalizes it.  The
    // Multi-LiDAR input uses these fields for the physical beam origin, so
    // downsample XYZ only and restore averaged metadata explicitly below.
    downSizeFilterSurf.setDownsampleAllData(false);
  }
  else
  {
    downSizeFilterSurf.setDownsampleAllData(true);
  }
  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);

  if (multi_lidar_enabled)
  {
    const double inverse_leaf_size = 1.0 / filter_size_surf_min;
    for (PointType &point : feats_down_body->points)
    {
      const DownsampleVoxelKey key{
          static_cast<std::int64_t>(
              std::floor(point.x * inverse_leaf_size)),
          static_cast<std::int64_t>(
              std::floor(point.y * inverse_leaf_size)),
          static_cast<std::int64_t>(
              std::floor(point.z * inverse_leaf_size))};
      const auto metadata = downsample_metadata.find(key);
      if (metadata == downsample_metadata.end() ||
          metadata->second.count == 0)
        throw std::runtime_error(
            "Cannot recover Multi-LiDAR metadata after voxel filtering.");
      const double inverse_count =
          1.0 / static_cast<double>(metadata->second.count);
      const V3D beam_origin =
          metadata->second.beam_origin_sum * inverse_count;
      point.normal_x = static_cast<float>(beam_origin.x());
      point.normal_y = static_cast<float>(beam_origin.y());
      point.normal_z = static_cast<float>(beam_origin.z());
      point.intensity = static_cast<float>(
          metadata->second.intensity_sum * inverse_count);
    }
  }
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  // Qualified INS attitude (same mode-4 gate, freshness and timing contract as
  // the backend position factors) as a bounded absolute attitude measurement.
  // It never touches position/velocity; without it, scan-to-map registration
  // on this multi-LiDAR rig drifted native pitch ~0.01 deg per metre.
  voxelmap_manager->attitude_aiding_ = false;
  // Always-on aiding imprints INS roll/pitch biases into well-constrained scans (truck29 startup
  // pole ghost), so by default the aid is applied only while the LiDAR geometry is weak: on an
  // open, flat quarry floor (truck28 t=920-1000 s) the translation is unobserved, a 1-2 deg tilt
  // error leaks gravity into the velocity and pure LIO ran away to 44 m/s.
  const bool weak_gate = frontend_attitude_weak_geometry_info > 0.0;
  attitude_gate_tilt_deg_ = -1.0;
  if (rtk_fusion_ && (frontend_attitude_aiding || weak_gate) && rtk_observations_enabled)
  {
    RtkFusion::Observation reference;
    gtsam::Pose3 alignment;
    if (rtk_fusion_->referenceAt(LidarMeasures.last_lio_update_time, reference) &&
        reference.orientation_valid && rtk_fusion_->frontendAlignment(alignment))
    {
      const M3D measurement = alignment.rotation().matrix().transpose() * reference.orientation.toRotationMatrix();
      const V3D mask(frontend_attitude_axis_mask[0], frontend_attitude_axis_mask[1], frontend_attitude_axis_mask[2]);
      attitude_gate_tilt_deg_ = Log(M3D(measurement.transpose() * state_propagat.rot_end)).cwiseProduct(mask).norm() * 180.0 / M_PI;
      if (weak_gate && weak_geometry_active_ && attitude_gate_tilt_deg_ > frontend_attitude_weak_tilt_deg && !weak_tilt_latched_)
      {
        weak_tilt_latched_ = true;
        RCLCPP_INFO(node_->get_logger(), "Weak LiDAR geometry with %.2f deg roll/pitch disagreement at %.3f: INS roll/pitch aiding on",
                    attitude_gate_tilt_deg_, LidarMeasures.last_lio_update_time);
      }
      if (attitude_gate_log.is_open())
        attitude_gate_log << std::setprecision(17) << LidarMeasures.last_lio_update_time << ',' << weakest_translation_information_
                          << ',' << attitude_gate_tilt_deg_ << ',' << int(weak_geometry_active_) << ',' << int(weak_tilt_latched_) << '\n';
    }
    // attitude_gate_tilt_deg_ >= 0: a qualified reference and the frontend alignment exist.
    if (attitude_gate_tilt_deg_ >= 0.0 &&
        (frontend_attitude_aiding || (weak_gate && weak_geometry_active_ && weak_tilt_latched_)))
    {
      voxelmap_manager->attitude_measurement_ =
          alignment.rotation().matrix().transpose() * reference.orientation.toRotationMatrix();
      const double sigma = frontend_attitude_sigma_deg * M_PI / 180.0;
      voxelmap_manager->attitude_information_ = 1.0 / (sigma * sigma);
      voxelmap_manager->attitude_huber_rad_ = frontend_attitude_huber_deg * M_PI / 180.0;
      voxelmap_manager->attitude_pull_fraction_ = frontend_attitude_pull_fraction;
      voxelmap_manager->attitude_axis_mask_ = V3D(frontend_attitude_axis_mask[0],
          frontend_attitude_axis_mask[1], frontend_attitude_axis_mask[2]);
      voxelmap_manager->attitude_aiding_ = true;
    }
  }
  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;
  if (frontend_attitude_weak_geometry_info > 0.0 && voxelmap_manager->collect_geometry_diagnostics_)
  {
    // LiDAR-only information (attitude rows are added after it is captured), state order [rot, pos].
    const Eigen::Matrix<double, 6, 6> &information = voxelmap_manager->lidar_information_body_;
    const Eigen::Matrix3d rotation_block = information.topLeftCorner<3, 3>();
    Eigen::Matrix3d translation = information.bottomRightCorner<3, 3>();
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(rotation_block);
    if (ldlt.info() == Eigen::Success && rotation_block.trace() > 0.0)
      translation -= information.block<3, 3>(3, 0) * ldlt.solve(information.block<3, 3>(0, 3));
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(0.5 * (translation + translation.transpose()));
    weakest_translation_information_ = eig.info() == Eigen::Success ? std::max(0.0, eig.eigenvalues()(0)) : 0.0;
    const bool was_active = weak_geometry_active_;
    if (weakest_translation_information_ > frontend_attitude_weak_release_factor * frontend_attitude_weak_geometry_info)
      weak_geometry_armed_ = true;
    if (weak_geometry_armed_ && !weak_geometry_active_ &&
        weakest_translation_information_ < frontend_attitude_weak_geometry_info)
      weak_geometry_active_ = true;
    else if (weak_geometry_active_ &&
             weakest_translation_information_ > frontend_attitude_weak_release_factor * frontend_attitude_weak_geometry_info)
      weak_geometry_active_ = false;
    if (!weak_geometry_active_) weak_tilt_latched_ = false;
    if (weak_geometry_active_ && weak_tilt_latched_) ++weak_geometry_frames_;
    if (weak_geometry_active_ != was_active)
    {
      if (weak_geometry_active_) ++weak_geometry_episodes_;
      RCLCPP_INFO(node_->get_logger(), "Weak LiDAR geometry %s at %.3f: translational information %.3g (gate %.3g)%s",
                  weak_geometry_active_ ? "entered" : "left", LidarMeasures.last_lio_update_time,
                  weakest_translation_information_, frontend_attitude_weak_geometry_info,
                  weak_geometry_active_ ? "; INS roll/pitch aiding armed (roll/pitch disagreement gate)" : "; aiding off");
    }
  }
  if (lio_geometry_log.is_open())
  {
    // A fixed metric radius makes spectra comparable across frames; it is a
    // unit convention, not a degeneracy threshold or extra measurement.
    constexpr double radius = 30.0;
    const auto information = voxelmap_manager->normalizedLidarInformation(radius);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(information);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite())
      throw std::runtime_error("Non-finite LiDAR-only geometry information");
    lio_geometry_log << std::setprecision(17) << LidarMeasures.last_lio_update_time << ','
        << voxelmap_manager->effct_feat_num_ << ',' << voxelmap_manager->lidar_residual_rms_ << ',' << radius;
    for (double value : eig.eigenvalues()) lio_geometry_log << ',' << value;
    lio_geometry_log << ',' << (eig.eigenvalues()(5) > 0 ? eig.eigenvalues()(0) / eig.eigenvalues()(5) : 0);
    for (int row = 0; row < 6; ++row) for (int column = row; column < 6; ++column)
      lio_geometry_log << ',' << voxelmap_manager->lidar_information_body_(row, column);
    // Read-only: the estimated world gravity is not necessarily ENU -Z.
    // Retain it to distinguish native tilt from a later global correction;
    // neither a fixed -Z assumption nor an INS attitude is a substitute.
    for (double value : _state.gravity) lio_geometry_log << ',' << value;
    lio_geometry_log << '\n';
  }

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(root_dir + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(root_dir + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = quaternionFromRpy(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    voxelmap_manager->pv_list_[i].var = voxelmap_manager->pointCovarianceWorld(
        voxelmap_manager->cross_mat_list_[i], voxelmap_manager->body_cov_list_[i]);
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  if (print_frame_timing) std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();
  fp.lap("lio.front_icp_voxel_map");

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    const V3D slide_before = voxelmap_manager->last_slide_position;
    voxelmap_manager->mapSliding();
    // The visual sparse map slides with the LiDAR map (same cube), see VIOManager::slideVisualMap.
    if (vio_manager && (voxelmap_manager->last_slide_position - slide_before).squaredNorm() > 0.0)
    {
      const double lidar_half_size = voxelmap_manager->config_setting_.half_map_size *
                                     voxelmap_manager->config_setting_.max_voxel_size_;
      // Never keep visual points the LiDAR map already dropped; optionally keep far fewer.
      const double half_size = vio_visual_half_map_m > 0.0 ? std::min(vio_visual_half_map_m, lidar_half_size)
                                                           : lidar_half_size;
      // heading of the camera side: body forward axis (RFU y) projected to the horizontal
      V3D forward = _state.rot_end * V3D(0.0, 1.0, 0.0);
      forward.z() = 0.0;
      if (forward.norm() > 1e-6) forward.normalize();
      const std::size_t released = vio_manager->slideVisualMap(voxelmap_manager->last_slide_position, half_size,
                                                               forward, vio_visual_behind_m);
      if (released > 0)
        RCLCPP_INFO(node_->get_logger(), "Visual map sliding: released %zu points outside +-%.0f m (%zu voxels kept)",
                    released, half_size, vio_manager->feat_map.size());
    }
  }
  
  fp.lap("lio.map_sliding");
  // World-frame copies of the scan are display products in LIO mode (the RTK backend archives the
  // body-frame deskewed scan). LIVO always needs pcl_w_wait_pub: the next VIO update colours it for the
  // RGB map. Without a subscriber (no RViz) the two ~100k-point transforms + PointCloud2 serializations
  // per frame (~10 MB) were pure overhead.
  const bool world_cloud_needed = slam_mode_ == LIVO || pcd_save_en ||
                                  pubLaserCloudFullRes->get_subscription_count() > 0;
  if (world_cloud_needed)
  {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
      RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }
    *pcl_w_wait_pub = *laserCloudWorld;

    fp.lap("lio.world_transform_copy");
    publish_frame_world(pubLaserCloudFullRes, vio_manager);
    fp.lap("lio.publish_world_cloud");
  }
  if (pubCurrentFrame->get_subscription_count() > 0)
  {
    sensor_msgs::PointCloud2 current_frame_msg;
    PointCloudXYZI::Ptr current_frame_world(new PointCloudXYZI(feats_undistort->size(), 1));
    for (size_t i = 0; i < feats_undistort->size(); i++)
    {
      RGBpointBodyToWorld(&feats_undistort->points[i], &current_frame_world->points[i]);
    }
    // Keep physical reflectivity, including for native LIO without RTK. Camera
    // mode displays /cloud_registered instead, which retains actual RGB.
    pcl::toROSMsg(*current_frame_world, current_frame_msg);
    current_frame_msg.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
    current_frame_msg.header.frame_id = lio_world_frame;
    pubCurrentFrame->publish(current_frame_msg);
    fp.lap("lio.publish_current_frame");
  }
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);
  publishReferenceTrajectory(LidarMeasures.last_lio_update_time);
  fp.lap("lio.publish_effect_path_misc");
  if (rtk_fusion_)
  {
    // point_time_reference is the absolute time represented by curvature==0.
    // For ONLY_LIO it is the physical scan start; for LIVO it is the previous
    // image-rate estimator update after the raw scan has been split/rebased.
    // feats_undistort is rigidly deskewed to last_lio_update_time in either
    // case, so this is the exact native interval consumed by the RTK backend.
    const double fusion_begin = LidarMeasures.measures.back().point_time_reference;
    if (slam_mode_ == LIVO)
    {
      if (pending_rtk_livo_frame_.valid)
        throw std::runtime_error("Previous LIVO frame reached the next LiDAR update before camera coloring");
      // Defer only the backend handoff until the immediately following VIO
      // update colors these SAME world points. Preserve the pre-VIO native
      // state/cloud/information exactly, so RGB never changes optimization.
      pending_rtk_livo_frame_.valid = true;
      pending_rtk_livo_frame_.stamp = LidarMeasures.last_lio_update_time;
      pending_rtk_livo_frame_.begin = fusion_begin;
      pending_rtk_livo_frame_.state = _state;
      pending_rtk_livo_frame_.cloud =
          std::make_shared<PointCloudXYZI>(*feats_undistort);
      pending_rtk_livo_frame_.lidar_information =
          voxelmap_manager->lidar_information_body_;
    }
    else
    {
      rtk_fusion_->submit(LidarMeasures.last_lio_update_time, fusion_begin,
          _state, feats_undistort, voxelmap_manager->lidar_information_body_);
    }
  }

  fp.lap("lio.rtk_submit");
  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  if (status_interval_sec > 0.0 &&
      LidarMeasures.last_lio_update_time - last_status_print_time_ >= status_interval_sec)
  {
    last_status_print_time_ = LidarMeasures.last_lio_update_time;
    RCLCPP_INFO(node_->get_logger(),
                "[status] t=%+.1f s frame %d: LIO %.1f ms (avg %.1f), points %zu, VIO avg %.1f ms%s",
                LidarMeasures.last_lio_update_time - _first_lidar_time, frame_num,
                1000.0 * (t4 - t0), 1000.0 * aver_time_consu, feats_down_body->points.size(),
                1000.0 * vio_manager->ave_total,
                slam_mode_ == LIVO ? (vio_ekf_update_en ? " [LIVO]" : " [LIVO colour-only]") : " [LIO]");
    logMemoryStatus();
  }
  if (print_frame_timing)
  {
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  }

  if (print_frame_timing && multi_lidar_enabled && !current_lidar_ts_info.sources.empty())
  {
    const auto &ts = current_lidar_ts_info;
    std::size_t max_topic_len = 12;
    for (const auto &src : ts.sources)
      max_topic_len = std::max(max_topic_len, src.topic.size());
    const std::size_t col1 = max_topic_len + 2;
    const std::size_t total_w = col1 + 3 + 14 + 3 + 10 + 3 + 12 + 2;
    const std::string hline(total_w, '-');
    printf("\033[1;34m+%s+\033[0m\n", hline.c_str());
    const std::size_t title_pad = total_w > 28 ? (total_w - 28) / 2 : 0;
    printf("\033[1;34m|%*sLiDAR Timestamp Sync%*s|\033[0m\n",
           static_cast<int>(title_pad), "",
           static_cast<int>(total_w - 28 - title_pad), "");
    printf("\033[1;34m+%s+\033[0m\n", hline.c_str());
    printf("\033[1;34m| %-*s | %-14s | %-10s | %-12s |\033[0m\n",
           static_cast<int>(col1), "Topic",
           "Header Time(s)", "Delta(ms)", "Points");
    printf("\033[1;34m+%s+\033[0m\n", hline.c_str());
    for (const auto &src : ts.sources)
    {
      const double delta_ms = (src.header_time - ts.merged_header_time) * 1000.0;
      printf("\033[1;36m| %-*s | %-14.6f | %-10.3f | %-12zu |\033[0m\n",
             static_cast<int>(col1), src.topic.c_str(),
             src.header_time, delta_ms, src.point_count);
    }
    printf("\033[1;34m+%s+\033[0m\n", hline.c_str());
    printf("\033[1;36m| %-*s | %-14.6f | %-10s | %-12s |\033[0m\n",
           static_cast<int>(col1), "Merged",
           ts.merged_header_time, "", "");
    printf("\033[1;36m| %-*s | %-14s | %-10.3f | %-12s |\033[0m\n",
           static_cast<int>(col1), "Scan-Start Span",
           "", ts.start_span_ms, "");
    printf("\033[1;34m+%s+\033[0m\n", hline.c_str());
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
  fp.lap("lio.logging_tables");
  frame_profile::Table::instance().endFrame(print_frame_timing);
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = root_dir + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = root_dir + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run(const volatile std::sig_atomic_t &stop_requested)
{
  rclcpp::Rate rate(1000.0);
  // rclcpp::spin_some() takes at most one message per subscription per call.
  // With one call per processed frame (~80 ms in LIVO) the 100 Hz IMU backlog
  // grew to ~0.9 s inside DDS, and long playbacks then lost or reordered IMU
  // samples (fatal gap / backwards stamp). Drain every ready message into the
  // mapper's own buffers instead; playback lead is bounded by the player.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  frame_profile::Stopwatch run_sw;
  while (rclcpp::ok())
  {
    // Once stopped, drain already accepted complete LiDAR/IMU groups, but
    // stop admitting an unbounded live stream into the finalization queue.
    if (!stop_requested) executor.spin_all(std::chrono::milliseconds(50));
    run_sw.lap("run.spin_all_callbacks");
    if (!sync_packages(LidarMeasures)) 
    {
      run_sw.lap("run.sync_wait");
      if (stop_requested) break;
      rate.sleep();
      continue;
    }
    run_sw.lap("run.sync_packages");
    handleFirstFrame();

    processImu();
    run_sw.lap("run.imu_process");

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
    run_sw.restart();  // frame sections are timed inside handleLIO/handleVIO
  }
  // The backend may spend seconds finalizing its map. Persist frontend
  // diagnostics first, so its completion cannot precede buffered LIO rows.
  if (lio_geometry_log.is_open()) lio_geometry_log.flush();
  if (fout_lidar_pos.is_open()) fout_lidar_pos.flush();
  releaseFrontendMaps();
  if (rtk_fusion_) rtk_fusion_->finish();
  savePCD();
  if (frontend_attitude_weak_geometry_info > 0.0)
    RCLCPP_INFO(node_->get_logger(), "Weak-geometry attitude aiding: %ld aided scans, %ld weak episodes (gate %.3g)",
                weak_geometry_frames_, weak_geometry_episodes_, frontend_attitude_weak_geometry_info);
  printCalibrationSummary("final");
}

namespace
{
// ZYX (yaw-pitch-roll) Euler angles in degrees, same convention as
// tools/calib/livo_calib.py rpy_deg (scipy "xyz" extrinsic).
Eigen::Vector3d rpyDegrees(const Eigen::Matrix3d &R)
{
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw) * (180.0 / M_PI);
}
}  // namespace

void LIVMapper::printCalibrationSummary(const char *stage) const
{
  // One block that a person can read on the console: which calibration is in
  // effect, every extrinsic, and every time offset with its meaning. Printed at
  // startup and again after the backend finished, so it stays visible at the
  // end of a run.
  std::ostringstream out;
  out << std::fixed;
  const char *line = "------------------------------------------------------------------------------------------";
  out << "\n" << line << "\n"
      << "  CALIBRATION IN EFFECT (" << stage << ")"
      << (calibration_identity_.empty() ? "" : "  " + calibration_identity_) << "\n"
      << line << "\n";
  if (!calibration_file_.empty()) out << "  file : " << calibration_file_ << "\n";
  if (!calibration_type_.empty()) out << "  type : " << calibration_type_ << "\n";
  out << "  LiDAR -> rear axle (RFU)                 x[m]      y[m]      z[m]   roll[deg] pitch[deg]  yaw[deg]\n";
  for (const auto &source : lidar_sources)
  {
    const Eigen::Vector3d rpy = rpyDegrees(source->rear_from_lidar_rotation);
    const Eigen::Vector3d &t = source->rear_from_lidar_translation;
    out << "   " << std::left << std::setw(40) << (source->topic + " (" + source->transform_key + ")") << std::right
        << std::setprecision(3) << std::setw(9) << t.x() << std::setw(10) << t.y() << std::setw(10) << t.z()
        << std::setw(10) << rpy.x() << std::setw(11) << rpy.y() << std::setw(10) << rpy.z() << "\n";
  }
  if (img_en)
  {
    const Eigen::Vector3d rpy = rpyDegrees(rear_from_camera_.block<3, 3>(0, 0));
    const Eigen::Vector3d t = rear_from_camera_.block<3, 1>(0, 3);
    out << "   " << std::left << std::setw(40) << "camera -> rear axle" << std::right
        << std::setprecision(3) << std::setw(9) << t.x() << std::setw(10) << t.y() << std::setw(10) << t.z()
        << std::setw(10) << rpy.x() << std::setw(11) << rpy.y() << std::setw(10) << rpy.z() << "\n";
    out << "   camera intrinsics fx " << std::setprecision(2) << camera_intrinsics[0] << " fy " << camera_intrinsics[1]
        << " cx " << camera_intrinsics[2] << " cy " << camera_intrinsics[3] << " (" << camera_width << "x" << camera_height
        << "), distortion [" << std::setprecision(4) << camera_distortion[0] << " " << camera_distortion[1] << " "
        << camera_distortion[2] << " " << camera_distortion[3] << " " << camera_distortion[4] << "]\n";
  }
  out << "  rear axle -> physical IMU lever [m]      " << std::setprecision(3) << std::setw(9) << imu_to_rear_axle.x()
      << std::setw(10) << imu_to_rear_axle.y() << std::setw(10) << imu_to_rear_axle.z()
      << (rear_axle_to_imu_enabled ? "  (estimator body = IMU)" : "  (not applied)") << "\n";
  out << std::setprecision(4);
  out << "  time offsets [s] (added to the sensor header stamps)\n";
  out << "   scan  <-> IMU  (front-end deskew/propagation)  " << std::showpos << lidar_time_offset << std::noshowpos
      << (std::isfinite(calib_lidar_frontend_offset_) ? "  [calibration time_offsets.lidar_frontend]"
                                                       : (std::isfinite(calib_lidar_ins_offset_) ? "  [calibration time_offsets.lidar]" : "  [yaml lidar_time_offset]"))
      << "\n";
  if (std::isfinite(calib_lidar_frontend_offset_))
    out << "   scan  <-> INS  (axle-stage alignment)          " << std::showpos << calib_lidar_ins_offset_ << std::noshowpos
        << "  -> reference shift " << std::showpos << rtk_reference_time_shift_ << std::noshowpos << " applied to the INS association\n";
  if (img_en)
    out << "   image <-> IMU  (camera)                        " << std::showpos << img_time_offset << std::noshowpos << "\n";
  if (rtk_fusion_)
  {
    const double pos = rtk_fusion_->positionTimeOffset();
    const double att = rtk_fusion_->attitudeTimeOffset();
    out << "   INS association (effective): position " << std::showpos << pos << ", attitude " << att << std::noshowpos
        << (std::isfinite(calib_rtk_position_offset_) ? "  [calibration rtk_position/ins_attitude + shift]" : "  [yaml rtk.time_offset_sec + shift]") << "\n"
        << "   => scan stamped t (after the scan<->IMU offset) uses INS position at t" << std::showpos << pos
        << " and INS attitude at t" << att << std::noshowpos << "\n";
    const Eigen::Vector3d mis = rtk_fusion_->installationMisalignmentDeg();
    if (mis.allFinite() && mis.norm() > 0.0)
      out << "   INS installation misalignment estimated by the backend [deg]: " << std::setprecision(4)
          << mis.x() << " " << mis.y() << " " << mis.z() << "\n";
  }
  out << "  camera mode: " << (img_en ? (vio_ekf_update_en ? "LIVO (photometric ESIKF update on)" : "colour-only (vio.ekf_update_en=false, estimator = LIO)")
                                      : "off (LIO)")
      << (img_en && camera_frame_divider > 1 ? ", frame divider " + std::to_string(camera_frame_divider) : "")
      << (img_en && vio_ekf_update_en ? (vio_lidar_parity_ratio > 0 ? ", LiDAR-parity bound " + std::to_string(vio_lidar_parity_ratio) : std::string(", fixed visual weight")) : std::string()) << "\n";
  out << line;
  RCLCPP_INFO(node_->get_logger(), "%s", out.str().c_str());
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stampToSec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stampToSec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stampToSec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::multi_lidar_pcl_cbk(
    const sensor_msgs::PointCloud2::ConstSharedPtr &msg,
    std::size_t source_index)
{
  frame_profile::Scoped cb_timer("cb.lidar");
  if (!lidar_en) return;
  std::unique_lock<std::mutex> lock(mtx_buffer);
  try
  {
    if (source_index >= lidar_sources.size())
      throw std::runtime_error("Invalid Multi-LiDAR source index.");
    LidarSource &source = *lidar_sources[source_index];
    const double header_time = stampToSec(msg->header.stamp);
    if (!std::isfinite(header_time))
      throw std::runtime_error(
          source.topic + " has a non-finite header timestamp.");
    if (header_time <= source.last_header_time)
      throw std::runtime_error(
          source.topic + " timestamp is not strictly increasing.");
    source.last_header_time = header_time;

    PointCloudXYZI::Ptr points(new PointCloudXYZI());
    // Preprocessing, including ring validation, decimation, point-time
    // validation and the blind-zone filter, occurs in the physical sensor
    // frame before applying the sensor extrinsic.
    p_pre->process(msg, points);
    if (!points || points->size() <= 1)
      throw std::runtime_error(
          source.topic + " has fewer than two valid time-tagged points.");

    // These bounds are expressed in the physical source-LiDAR frame. Apply
    // them before the source cloud is transformed into the common rear-axle
    // frame, otherwise the configured rectangles would have the wrong axes.
    if (!source.body_exclusion_rectangles.empty())
    {
      const auto first_retained = std::remove_if(
          points->points.begin(), points->points.end(),
          [&source, this](const PointType &point) {
            if (point.z <= multi_lidar_body_exclusion_min_z) return false;
            for (const auto &rectangle :
                 source.body_exclusion_rectangles)
            {
              if (point.x >= rectangle.min_x &&
                  point.x <= rectangle.max_x &&
                  point.y >= rectangle.min_y &&
                  point.y <= rectangle.max_y)
                return true;
            }
            return false;
          });
      points->points.erase(first_retained, points->points.end());
      points->width = static_cast<std::uint32_t>(points->points.size());
      points->height = 1;
      if (points->size() <= 1)
        throw std::runtime_error(
            source.topic +
            " has fewer than two points after vehicle-body filtering.");
    }

    for (PointType &point : points->points)
    {
      const V3D lidar_point(point.x, point.y, point.z);
      const V3D rear_point =
          source.rear_from_lidar_rotation * lidar_point +
          source.rear_from_lidar_translation;
      point.x = static_cast<float>(rear_point.x());
      point.y = static_cast<float>(rear_point.y());
      point.z = static_cast<float>(rear_point.z());

      // Preserve the physical beam origin in the otherwise unused normal
      // fields.  Deskew transforms it together with the return point, and the
      // measurement covariance is then computed from the true beam vector
      // instead of incorrectly treating the rear axle as every LiDAR origin.
      point.normal_x =
          static_cast<float>(source.rear_from_lidar_translation.x());
      point.normal_y =
          static_cast<float>(source.rear_from_lidar_translation.y());
      point.normal_z =
          static_cast<float>(source.rear_from_lidar_translation.z());
    }

    source.pending_frames.push_back({header_time, points});
    if (extrinsic_precheck_.enabled()) feedExtrinsicPrecheck(source_index);
    if (source.pending_frames.size() > multi_lidar_queue_size)
    {
      const double dropped_time = source.pending_frames.front().header_time;
      source.pending_frames.pop_front();
      RCLCPP_WARN(
          node_->get_logger(),
          "Dropping queued %s frame %.9f because the other configured "
          "LiDAR sources did not arrive in time.",
          source.topic.c_str(), dropped_time);
    }
    synchronizeMultiLidarFrames();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Multi-LiDAR input failed: %s", error.what());
    lock.unlock();
    sig_buffer.notify_all();
    rclcpp::shutdown();
    return;
  }
  lock.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::synchronizeMultiLidarFrames()
{
  while (!lidar_sources.empty())
  {
    bool every_source_ready = true;
    for (const auto &source : lidar_sources)
      every_source_ready &= !source->pending_frames.empty();
    if (!every_source_ready) return;

    std::size_t earliest_source = 0;
    double earliest_time =
        lidar_sources.front()->pending_frames.front().header_time;
    double latest_time = earliest_time;
    for (std::size_t index = 1; index < lidar_sources.size(); ++index)
    {
      const double time =
          lidar_sources[index]->pending_frames.front().header_time;
      if (time < earliest_time)
      {
        earliest_time = time;
        earliest_source = index;
      }
      latest_time = std::max(latest_time, time);
    }

    const double start_span = latest_time - earliest_time;
    if (start_span > multi_lidar_sync_tolerance)
    {
      LidarSource &source = *lidar_sources[earliest_source];
      source.pending_frames.pop_front();
      RCLCPP_WARN(
          node_->get_logger(),
          "Dropping unmatched %s frame: configured LiDAR scan-start span "
          "%.3f ms exceeds %.3f ms.",
          source.topic.c_str(), start_span * 1000.0,
          multi_lidar_sync_tolerance * 1000.0);
      continue;
    }

    PointCloudXYZI::Ptr merged(new PointCloudXYZI());
    std::size_t total_points = 0;
    for (const auto &source : lidar_sources)
      total_points += source->pending_frames.front().points->size();
    merged->reserve(total_points);

    MergedLidarTimestampInfo ts_info;
    ts_info.merged_header_time = 0.0;
    ts_info.start_span_ms = start_span * 1000.0;
    for (auto &source : lidar_sources)
    {
      PendingLidarFrame frame = std::move(source->pending_frames.front());
      source->pending_frames.pop_front();
      const float header_delta_ms = static_cast<float>(
          (frame.header_time - earliest_time) * 1000.0);
      for (PointType point : frame.points->points)
      {
        point.curvature += header_delta_ms;
        merged->push_back(point);
      }
      MergedLidarTimestampInfo::SourceInfo src_info;
      src_info.topic = source->topic;
      src_info.header_time = frame.header_time;
      src_info.point_count = frame.points->size();
      ts_info.sources.push_back(src_info);
    }
    if (merged->size() <= 1)
      throw std::runtime_error(
          "Merged Multi-LiDAR frame has fewer than two points.");
    std::stable_sort(
        merged->points.begin(), merged->points.end(),
        [](const PointType &left, const PointType &right) {
          return left.curvature < right.curvature;
        });

    const double merged_header_time = earliest_time + lidar_time_offset;
    if (merged_header_time <= last_timestamp_lidar)
      throw std::runtime_error(
          "Merged Multi-LiDAR timestamp is not strictly increasing.");
    const double point_span_seconds =
        merged->points.back().curvature / 1000.0;
    if (!std::isfinite(point_span_seconds) ||
        point_span_seconds < 0.0 ||
        point_span_seconds >
            lidar_max_point_offset + multi_lidar_sync_tolerance + 1.0e-6)
      throw std::runtime_error(
          "Merged Multi-LiDAR point times exceed the configured envelope.");
    ts_info.merged_header_time = merged_header_time;

    lid_raw_data_buffer.push_back(merged);
    lid_header_time_buffer.push_back(merged_header_time);
    lid_timestamp_info_buffer.push_back(ts_info);
    last_timestamp_lidar = merged_header_time;
    ++multi_lidar_frame_count;
    if (multi_lidar_frame_count == 1 ||
        multi_lidar_frame_count % 100 == 0)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "Merged Multi-LiDAR frame %zu: sources=%zu, points=%zu, "
          "scan-start span=%.3f ms, point span=%.3f ms.",
          multi_lidar_frame_count, lidar_sources.size(), merged->size(),
          start_span * 1000.0, point_span_seconds * 1000.0);
    }
  }
}

void LIVMapper::feedExtrinsicPrecheck(std::size_t source_index)
{
  // Called with mtx_buffer held, right after the newest frame of this source
  // was queued. Stillness is judged on the newest 0.2 s of raw IMU samples.
  const std::size_t window = std::min<std::size_t>(imu_buffer.size(), 20);
  if (window < 10) return; // no IMU yet: cannot judge motion, keep waiting
  std::vector<Eigen::Vector3d> gyro, acc;
  gyro.reserve(window);
  acc.reserve(window);
  for (std::size_t i = imu_buffer.size() - window; i < imu_buffer.size(); ++i)
  {
    const auto &sample = *imu_buffer[i];
    gyro.emplace_back(sample.angular_velocity.x, sample.angular_velocity.y,
                      sample.angular_velocity.z);
    acc.emplace_back(sample.linear_acceleration.x, sample.linear_acceleration.y,
                     sample.linear_acceleration.z);
  }
  if (extrinsic_precheck_.isStatic(gyro, acc))
  {
    const PointCloudXYZI &frame = *lidar_sources[source_index]->pending_frames.back().points;
    std::vector<Eigen::Vector3f> points;
    points.reserve(frame.size());
    for (const auto &point : frame.points) points.emplace_back(point.x, point.y, point.z);
    extrinsic_precheck_.addFrame(source_index, points);
    if (extrinsic_precheck_.saturated()) finishExtrinsicPrecheck("static window complete");
  }
  else if (extrinsic_precheck_.sufficient())
    finishExtrinsicPrecheck("vehicle started moving");
  else
    finishExtrinsicPrecheck("vehicle moving before enough static frames were collected");
}

void LIVMapper::finishExtrinsicPrecheck(const string &reason)
{
  const auto results = extrinsic_precheck_.solve();
  bool any_applied = false, any_solved = false, any_gated = false;
  for (std::size_t s = 0; s < results.size(); ++s)
  {
    const auto &r = results[s];
    if (r.reference)
    {
      RCLCPP_INFO(node_->get_logger(), "Extrinsic precheck (%s): %s is the %s",
                  reason.c_str(), r.topic.c_str(), r.note.c_str());
      continue;
    }
    if (!r.solved)
    {
      RCLCPP_WARN(node_->get_logger(), "Extrinsic precheck: %s not checked: %s",
                  r.topic.c_str(), r.note.c_str());
      continue;
    }
    any_solved = true;
    any_gated |= !r.within_gates;
    const char *verdict = r.applied ? "applied"
                          : (!r.within_gates ? r.note.c_str()
                             : (!r.significant ? r.note.c_str() : "within gates, not applied (check only)"));
    if (r.applied || r.within_gates)
      RCLCPP_INFO(node_->get_logger(),
                  "Extrinsic precheck: %s ground residual median %.2f -> %.2f cm (n=%d), "
                  "correction rot_x %+.3f deg rot_y %+.3f deg dz %+.2f cm -> %s",
                  r.topic.c_str(), 100.0 * r.median_before_m, 100.0 * r.median_after_m,
                  r.matches_after, r.rot_x_deg, r.rot_y_deg, 100.0 * r.dz_m, verdict);
    else
      RCLCPP_WARN(node_->get_logger(),
                  "Extrinsic precheck: %s ground residual median %.2f -> %.2f cm (n=%d), "
                  "correction rot_x %+.3f deg rot_y %+.3f deg dz %+.2f cm -> %s",
                  r.topic.c_str(), 100.0 * r.median_before_m, 100.0 * r.median_after_m,
                  r.matches_after, r.rot_x_deg, r.rot_y_deg, 100.0 * r.dz_m, verdict);
    if (!r.applied) continue;
    any_applied = true;

    LidarSource &source = *lidar_sources[s];
    const V3D old_origin = source.rear_from_lidar_translation;
    source.rear_from_lidar_rotation = r.rear_from_lidar_after.block<3, 3>(0, 0);
    source.rear_from_lidar_translation = r.rear_from_lidar_after.block<3, 1>(0, 3);
    const M3D correction_rotation = r.correction.block<3, 3>(0, 0);
    const V3D correction_translation = r.correction.block<3, 1>(0, 3);
    const V3D new_origin = source.rear_from_lidar_translation;
    // Returns already expressed in the rear frame with the old extrinsic carry
    // their beam origin in the normal fields, which identifies the source.
    const auto correct_cloud = [&](PointCloudXYZI &cloud) {
      for (PointType &point : cloud.points)
      {
        if (std::abs(point.normal_x - static_cast<float>(old_origin.x())) > 1e-4f ||
            std::abs(point.normal_y - static_cast<float>(old_origin.y())) > 1e-4f ||
            std::abs(point.normal_z - static_cast<float>(old_origin.z())) > 1e-4f)
          continue;
        const V3D corrected =
            correction_rotation * V3D(point.x, point.y, point.z) + correction_translation;
        point.x = static_cast<float>(corrected.x());
        point.y = static_cast<float>(corrected.y());
        point.z = static_cast<float>(corrected.z());
        point.normal_x = static_cast<float>(new_origin.x());
        point.normal_y = static_cast<float>(new_origin.y());
        point.normal_z = static_cast<float>(new_origin.z());
      }
    };
    for (auto &pending : source.pending_frames) correct_cloud(*pending.points);
    // Merged frames still queued are replaced by corrected copies. The front
    // element is left alone: the estimator thread may already hold it.
    for (std::size_t i = 1; i < lid_raw_data_buffer.size(); ++i)
    {
      auto copy = std::make_shared<PointCloudXYZI>(*lid_raw_data_buffer[i]);
      correct_cloud(*copy);
      lid_raw_data_buffer[i] = copy;
    }
  }
  string status = "skipped";
  if (any_applied) status = "applied";
  else if (any_solved && !extrinsic_precheck_.config().apply) status = "check_only";
  else if (any_solved && any_gated) status = "gated";
  else if (any_solved) status = "confirmed";
  if (any_applied && !p_imu->imu_need_init)
    RCLCPP_WARN(node_->get_logger(),
                "Extrinsic precheck applied after mapping had started; earlier frames keep the "
                "previous extrinsics");
  try
  {
    std::filesystem::create_directories(root_dir + "Log");
    std::ofstream report(root_dir + "Log/extrinsic_precheck.json");
    report << extrinsic_precheck_.reportJson(status, last_timestamp_lidar);
    RCLCPP_INFO(node_->get_logger(), "Extrinsic precheck %s; report %sLog/extrinsic_precheck.json",
                status.c_str(), root_dir.c_str());
  }
  catch (const std::exception &error)
  {
    RCLCPP_WARN(node_->get_logger(), "Extrinsic precheck report not written: %s", error.what());
  }
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstSharedPtr &msg)
{
  frame_profile::Scoped cb_timer("cb.lidar");
  if (!lidar_en) return;
  mtx_buffer.lock();

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  // GroundExtractor normalizes header.stamp to the first-point/scan-start
  // time. Never subtract a scan period here.
  const double cur_head_time =
      stampToSec(msg->header.stamp) + lidar_time_offset;
  if (!std::isfinite(cur_head_time) || !ptr || ptr->size() <= 1)
  {
    RCLCPP_FATAL(
        node_->get_logger(),
        "Invalid LiDAR frame: timestamp must be finite and at least two "
        "valid time-tagged points must remain");
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    rclcpp::shutdown();
    return;
  }
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "LiDAR timestamp moved backwards; stop this run and restart the node");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    lidar_pushed = false;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    rclcpp::shutdown();
    return;
  }
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stampToSec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stampToSec(msg->header.stamp);
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stampToSec(msg->header.stamp);
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "LiDAR timestamp moved backwards; stop this run and restart the node");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    lidar_pushed = false;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    rclcpp::shutdown();
    return;
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::ins_odom_cbk(const nav_msgs::Odometry::ConstSharedPtr &msg_in)
{
  if (!imu_en || !imu_standard_rfu) return;

  const bool legacy_mislabeled_rear_axle =
      msg_in->child_frame_id == "imu_link_rfu";
  if (msg_in->header.frame_id != "ins_local_enu" ||
      (msg_in->child_frame_id != "vehicle_rear_axle_rfu" &&
       !legacy_mislabeled_rear_axle))
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Ignoring INS odometry with unexpected frames %s -> %s",
        msg_in->header.frame_id.c_str(), msg_in->child_frame_id.c_str());
    return;
  }
  if (legacy_mislabeled_rear_axle)
  {
    RCLCPP_WARN_ONCE(
        node_->get_logger(),
        "Legacy INS Odometry child_frame_id=imu_link_rfu is mislabeled; "
        "treating its translation as rear axle and applying the lever arm.");
  }

  // Position fusion does not require a valid INS attitude. Only reference
  // visualization and initial alignment below require the full quaternion.
  if (rtk_fusion_) rtk_fusion_->observe(*msg_in);  // reference always flows; factor creation is gated on observations_enabled

  MinePose mine_pose;
  mine_pose.stamp = stampToSec(msg_in->header.stamp) - imu_time_offset;
  mine_pose.position << msg_in->pose.pose.position.x,
                        msg_in->pose.pose.position.y,
                        msg_in->pose.pose.position.z;
  mine_pose.orientation = Eigen::Quaterniond(
      msg_in->pose.pose.orientation.w,
      msg_in->pose.pose.orientation.x,
      msg_in->pose.pose.orientation.y,
      msg_in->pose.pose.orientation.z);

  const bool pose_is_finite = std::isfinite(mine_pose.stamp)
      && mine_pose.position.allFinite()
      && mine_pose.orientation.coeffs().allFinite();
  if (!pose_is_finite || mine_pose.orientation.norm() < 1.0e-6)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Ignoring invalid INS odometry pose on %s", ins_odom_topic.c_str());
    return;
  }
  const M3D mine_from_rear_axle =
      mine_pose.orientation.normalized().toRotationMatrix();
  mine_pose.orientation = Eigen::Quaterniond(mine_from_rear_axle);
  mine_pose.orientation.normalize();

  if (rear_axle_to_imu_enabled)
  {
    // The new Odometry preserves the same st_point_3d/f_pos_alt translation
    // carried by the legacy packed INS pose. When the estimator body is moved
    // from the rear axle to the physical IMU, move this reference pose by the
    // same lever arm so LIO and RTK/INS remain at the same physical point.
    mine_pose.position += mine_from_rear_axle * (-imu_to_rear_axle);
  }

  if (!mine_pose_samples.empty() &&
      mine_pose.stamp < mine_pose_samples.back().stamp)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Ignoring out-of-order INS odometry sample");
    return;
  }
  // This separate odometry is used only for the one-time mine-frame anchor
  // and the RViz reference path. It is never inserted into LIO propagation or
  // measurement updates.
  mine_pose_samples.push_back(mine_pose);
}

void LIVMapper::ins_status_cbk(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &msg)
{
  if (rtk_fusion_) rtk_fusion_->observeStatus(*msg);
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstSharedPtr &msg_in)
{
  frame_profile::Scoped cb_timer("cb.imu");
  if (!imu_en) return;

  auto msg = std::make_shared<sensor_msgs::Imu>(*msg_in);
  double timestamp = stampToSec(msg->header.stamp) - imu_time_offset;

  constexpr double degrees_to_radians = M_PI / 180.0;
  M3D mine_from_body = M3D::Identity();
  if (imu_standard_rfu)
  {
    Eigen::Quaterniond orientation(
        msg_in->orientation.w, msg_in->orientation.x,
        msg_in->orientation.y, msg_in->orientation.z);
    if (orientation.coeffs().allFinite() && orientation.norm() >= 1.0e-6)
      mine_from_body = orientation.normalized().toRotationMatrix();
  }
  else
  {
    // Legacy clip_converter contract:
    // pitch=cov[0], roll=cov[1], local x/y=cov[3:4], altitude=orientation.z,
    // heading=orientation.w. Reproduce Rz(-heading)*Ry(roll)*Rx(pitch).
    const double pitch = msg_in->orientation_covariance[0] * degrees_to_radians;
    const double roll = msg_in->orientation_covariance[1] * degrees_to_radians;
    const double heading = msg_in->orientation.w * degrees_to_radians;
    mine_from_body =
        Eigen::AngleAxisd(-heading, V3D::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(roll, V3D::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(pitch, V3D::UnitX()).toRotationMatrix();

    MinePose mine_pose;
    mine_pose.stamp = timestamp;
    mine_pose.position << msg_in->orientation_covariance[3],
                          msg_in->orientation_covariance[4], msg_in->orientation.z;
    if (rear_axle_to_imu_enabled)
    {
      // Legacy packed position is at the rear axle; relocate it to the
      // physical IMU only for that legacy input format.
      mine_pose.position -= mine_from_body * imu_to_rear_axle;
    }
    mine_pose.orientation = Eigen::Quaterniond(mine_from_body).normalized();
    mine_pose_samples.push_back(mine_pose);
  }

  const double gyro_scale = imu_gyro_in_degrees ? degrees_to_radians : 1.0;
  msg->angular_velocity.x = msg_in->angular_velocity.x * gyro_scale;
  msg->angular_velocity.y = msg_in->angular_velocity.y * gyro_scale;
  msg->angular_velocity.z = msg_in->angular_velocity.z * gyro_scale;

  V3D acceleration(msg_in->linear_acceleration.x, msg_in->linear_acceleration.y,
                   msg_in->linear_acceleration.z);
  acceleration = imu_acceleration_transform * acceleration * imu_acceleration_scale;
  if (imu_acceleration_gravity_compensated)
  {
    acceleration += mine_from_body.transpose() * V3D(0.0, 0.0, G_m_s2);
  }
  msg->linear_acceleration.x = acceleration.x();
  msg->linear_acceleration.y = acceleration.y();
  msg->linear_acceleration.z = acceleration.z();
  msg->orientation.x = 0.0;
  msg->orientation.y = 0.0;
  msg->orientation.z = 0.0;
  msg->orientation.w = 1.0;
  msg->orientation_covariance.fill(0.0);
  msg->orientation_covariance[0] = -1.0;
  msg->header.frame_id = "imu";
  msg->header.stamp = stampFromSec(timestamp);

  // Retain the real pre-scan IMU samples. GroundExtractor deliberately keeps
  // a left and right IMU envelope around every LiDAR scan; discarding the
  // samples before the first LiDAR callback destroys that contract.
  if (last_timestamp_lidar >= 0.0 &&
      fabs(last_timestamp_lidar - timestamp) > 0.5 &&
      (!ros_driver_fix_en))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "IMU and LiDAR delta time is %.3f s", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = stampFromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    if (p_imu->imu_need_init)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Dropping an out-of-order startup IMU sample %.9f s older than "
          "the latest sample; no LiDAR deskew has started yet",
          last_timestamp_imu - timestamp);
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      return;
    }
    // A sample at most imu_stale_tolerance older than the newest one is a late or repeated delivery
    // (the recording itself is strictly monotonic; seen once at t=978 s of the 1230 s truck28 round while
    // the mapper lagged under load, which aborted the whole run). The newer samples are already
    // buffered, so dropping it changes nothing; only a real clock jump stops the run.
    constexpr double imu_stale_tolerance = 0.5;
    if (last_timestamp_imu - timestamp <= imu_stale_tolerance)
    {
      ++imu_stale_dropped_;
      RCLCPP_WARN(node_->get_logger(),
                  "Dropping a stale IMU sample %.9f s older than the newest one (late/repeated delivery, "
                  "%zu so far)", last_timestamp_imu - timestamp, imu_stale_dropped_);
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      return;
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_FATAL(node_->get_logger(),
                 "IMU timestamp moved backwards by %.9f s; stop this run and restart the node",
                 last_timestamp_imu - timestamp);
    rclcpp::shutdown();
    return;
  }

  if (last_timestamp_imu > 0.0 &&
      timestamp - last_timestamp_imu > imu_max_time_gap)
  {
    const double gap = timestamp - last_timestamp_imu;
    if (p_imu->imu_need_init)
    {
      // No cloud is deskewed or inserted into the map during IMU
      // initialization. A startup delivery discontinuity can therefore be
      // handled without interpolation: discard the uncommitted prefix and
      // restart the stationary initializer at the first sample after the
      // gap. Once initialization finishes, the same condition remains fatal.
      RCLCPP_WARN(
          node_->get_logger(),
          "Startup IMU delivery has a %.6f s gap (limit %.6f s) between "
          "%.9f and %.9f; restarting IMU initialization from the newer "
          "sample",
          gap, imu_max_time_gap, last_timestamp_imu, timestamp);
      imu_buffer.clear();
      p_imu->Reset();
    }
    else
    {
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      RCLCPP_FATAL(node_->get_logger(),
                   "IMU stream has a %.6f s gap (limit %.6f s); strict "
                   "LiDAR deskew cannot continue",
                   gap, imu_max_time_gap);
      rclcpp::shutdown();
      throw std::runtime_error("Input IMU gap interrupted mapping; exported prefix is not a complete run");
    }
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  frame_profile::Scoped cb_timer("cb.image");
  if (!img_en) return;
  auto msg = std::make_shared<sensor_msgs::Image>(*msg_in);
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // Generic divider for high-rate cameras (30 Hz -> 10 Hz with 3). In LIVO
  // mode every LIO/VIO update is cut at an image time, so the image rate also
  // sets the LiDAR sub-scan length and the estimator update rate.
  if (camera_frame_divider > 1)
  {
    static int divider_counter = 0;
    if (++divider_counter % camera_frame_divider != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = stampToSec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  if (print_frame_timing) ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Image timestamp moved backwards; stop this run and restart the node");
    rclcpp::shutdown();
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    m.point_time_reference = meas.lidar_frame_beg_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stampToSec(imu_buffer.front()->header.stamp) >
          meas.lidar_frame_end_time)
      {
        // Once initialization is complete, retain this future sample in the
        // shared buffer but also give the current measurement a reference to it
        // so IMU values can be interpolated exactly at the LiDAR frame end.
        if (!p_imu->imu_need_init) m.imu.push_back(imu_buffer.front());
        break;
      }
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    if (!lid_timestamp_info_buffer.empty())
    {
      current_lidar_ts_info = lid_timestamp_info_buffer.front();
      lid_timestamp_info_buffer.pop_front();
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stampToSec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      // pcl_proc_cur is assembled below with curvature rebased to the previous
      // estimator update, unlike a complete ONLY_LIO scan whose origin is its
      // own header timestamp.
      m.point_time_reference = meas.last_lio_update_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stampToSec(imu_buffer.front()->header.stamp) > m.lio_time)
        {
          if (!p_imu->imu_need_init) m.imu.push_back(imu_buffer.front());
          break;
        }

        if (stampToSec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      PointCloudXYZI carried_scan_tail;
      carried_scan_tail.swap(*meas.pcl_proc_next);
      const double carried_tail_time_reference =
          meas.pcl_proc_next_time_reference;
      meas.pcl_proc_cur->clear();

      std::size_t maximum_point_count = carried_scan_tail.size();
      for (const auto &buffered_cloud : lid_raw_data_buffer)
        maximum_point_count += buffered_cloud->size();
      meas.pcl_proc_cur->reserve(maximum_point_count);
      meas.pcl_proc_next->reserve(maximum_point_count);

      const auto distribute_points_at_image_boundary =
          [&](const PointCloudXYZI &points,
              double point_time_reference) {
            if (points.empty()) return;
            if (!std::isfinite(point_time_reference) ||
                point_time_reference > m.lio_time + 1.0e-6)
              throw std::runtime_error(
                  "Invalid buffered LiDAR point-time reference in LIVO");

            const double current_cutoff_ms =
                (m.lio_time - point_time_reference) * 1000.0;
            const double current_rebase_ms =
                (point_time_reference - meas.last_lio_update_time) * 1000.0;
            const double next_rebase_ms =
                (point_time_reference - m.lio_time) * 1000.0;
            for (PointType point : points.points)
            {
              if (point.curvature < current_cutoff_ms)
              {
                point.curvature += current_rebase_ms;
                meas.pcl_proc_cur->push_back(point);
              }
              else
              {
                point.curvature += next_rebase_ms;
                meas.pcl_proc_next->push_back(point);
              }
            }
          };

      // A 100 ms LiDAR scan spans about three 30 Hz image updates. Re-split
      // the retained tail at every image boundary instead of consuming the
      // whole tail one update too early.
      distribute_points_at_image_boundary(
          carried_scan_tail, carried_tail_time_reference);

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        distribute_points_at_image_boundary(
            *lid_raw_data_buffer.front(),
            lid_header_time_buffer.front());
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
        if (!lid_timestamp_info_buffer.empty())
          lid_timestamp_info_buffer.pop_front();
      }
      meas.pcl_proc_next_time_reference = m.lio_time;

      const auto point_time_less =
          [](const PointType &left, const PointType &right) {
            return left.curvature < right.curvature;
          };
      std::stable_sort(meas.pcl_proc_cur->points.begin(),
                       meas.pcl_proc_cur->points.end(), point_time_less);
      std::stable_sort(meas.pcl_proc_next->points.begin(),
                       meas.pcl_proc_next->points.end(), point_time_less);

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = stampToSec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.point_time_reference = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    m.point_time_reference = meas.lidar_frame_beg_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    if (!lid_timestamp_info_buffer.empty())
    {
      current_lidar_ts_info = lid_timestamp_info_buffer.front();
      lid_timestamp_info_buffer.pop_front();
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::releaseFrontendMaps()
{
  // After the last scan the LIO voxel map and the visual map are dead weight, but the backend's final phase
  // (loop-closure submaps, colour voxels of rgb_map_N, map chunks) peaked on top of them (truck29raw 20261013
  // LIVO 870 s: 5.8 GB at the last scan -> 9.3 GB). Nothing reads them after the processing loop.
  std::size_t lio = 0, visual = 0;
  if (voxelmap_manager)
  {
    lio = voxelmap_manager->voxel_map_.size();
    for (auto &entry : voxelmap_manager->voxel_map_) delete entry.second;
    voxelmap_manager->voxel_map_.clear();
  }
  if (vio_manager)
  {
    visual = vio_manager->feat_map.size();
    for (auto &entry : vio_manager->feat_map) delete entry.second;
    vio_manager->feat_map.clear();
  }
  malloc_trim(0);
  RCLCPP_INFO(node_->get_logger(), "Released the frontend maps before the final optimization (%zu LIO voxels, %zu visual voxels)",
              lio, visual);
}

void LIVMapper::logMemoryStatus()
{
  // Resident memory and the size of the two front-end maps (status interval; a full traversal costs a few ms).
  double rss_mb = std::numeric_limits<double>::quiet_NaN();
  {
    std::ifstream statm("/proc/self/statm");
    long total_pages = 0, resident_pages = 0;
    if (statm >> total_pages >> resident_pages)
      rss_mb = static_cast<double>(resident_pages) * static_cast<double>(sysconf(_SC_PAGESIZE)) / 1048576.0;
  }
  const MapMemoryStats lio = voxelmap_manager->memoryStats();
  std::size_t visual_voxels = 0, visual_points = 0, observations = 0;
  double image_mb = 0.0;
  if (slam_mode_ == LIVO && vio_manager)
  {
    std::unordered_set<const uchar *> images;
    for (const auto &entry : vio_manager->feat_map)
    {
      ++visual_voxels;
      for (const VisualPoint *point : entry.second->voxel_points)
      {
        ++visual_points;
        for (const Feature *feature : point->obs_)
        {
          ++observations;
          if (!feature->img_.empty() && images.insert(feature->img_.data).second)
            image_mb += static_cast<double>(feature->img_.total() * feature->img_.elemSize()) / 1048576.0;
        }
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(),
              "[memory] RSS %.0f MB | LIO map %zu voxels, %zu nodes, %zu planes (%zu plane objects), %zu held returns "
              "(cap %zu) ~%.0f MB | visual map %zu voxels, %zu points, %zu patches, pinned images %.0f MB",
              rss_mb, lio.roots, lio.nodes, lio.planes, lio.plane_objects, lio.points, lio.point_capacity,
              lio.bytes / 1048576.0,
              visual_voxels, visual_points, observations, image_mb);
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  // Display only: toImageMsg() deep-copies the 6 MB full-HD frame at every VIO update.
  if (pubImage.getNumSubscribers() == 0) return;
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

std::vector<std::uint32_t> LIVMapper::colourScan(const PointCloudXYZI &world)
{
  // 恢复早期同帧投影：当前图像只给同步对应的扫描取色，保留原来的 3 像素边距。
  // 不缓存历史视图，也不跨帧借色；视野外的点保留颜色码 0。
  // 每个扫描回波对应一个颜色码，与现有 LIVORGB2 存档逐点对齐。
  const auto start = std::chrono::steady_clock::now();
  const auto &frame = vio_manager->new_frame_;
  const vk::AbstractCamera &cam = *frame->cam_;
  if (rgb_flat_field_en && !flat_field_coefficients_.empty() && flat_gain_.empty())
  {
    // 沿用已有平场标定补偿边缘暗角：增益为平场的倒数，并限制在 [1/1.6, 1.6]。
    const int width = cam.width(), height = cam.height();
    const double cx = camera_intrinsics[2], cy = camera_intrinsics[3], sx = 0.5 * width, sy = 0.5 * height;
    const auto &c = flat_field_coefficients_;
    flat_gain_.resize(static_cast<std::size_t>(width) * height);
    for (int v = 0; v < height; ++v)
    {
      const double y = std::min(flat_field_y_hi_, std::max(flat_field_y_lo_, (v - cy) / sy));
      for (int u = 0; u < width; ++u)
      {
        const double x = std::min(1.05, std::max(-1.05, (u - cx) / sx)), x2 = x * x;
        const double log_flat = c[0] * x2 + c[1] * x2 * x2 + c[2] * y + c[3] * y * y + c[4] * y * y * y +
                                c[5] * y * y * y * y + c[6] * x2 * y + c[7] * x2 * y * y;
        flat_gain_[static_cast<std::size_t>(v) * width + u] =
            static_cast<float>(std::min(1.6, std::max(1.0 / 1.6, std::exp(-log_flat))));
      }
    }
  }
  const cv::Mat &image = vio_manager->img_rgb;
  const auto channel = [](float v) {
    return static_cast<std::uint32_t>(std::min(255.f, std::max(0.f, v)));
  };
  std::vector<std::uint32_t> colours(world.size(), 0);
  std::size_t coloured = 0;
#ifdef MP_EN
#pragma omp parallel for num_threads(MP_PROC_NUM) schedule(static) reduction(+ : coloured)
#endif
  for (std::size_t i = 0; i < world.size(); ++i)
  {
    const PointType &point = world.points[i];
    const V3D pw(point.x, point.y, point.z);
    const V3D pf = frame->w2f(pw);
    if (pf.z() <= 0.0 || pf.norm() <= blind_rgb_points) continue;
    const V2D px = frame->w2c(pw);
    if (!cam.isInFrame(px.cast<int>(), 3)) continue;
    V3F bgr = vio_manager->getInterpolatedPixel(image, px);
    if (!flat_gain_.empty())
      bgr *= flat_gain_[static_cast<std::size_t>(px.y()) * static_cast<std::size_t>(cam.width()) +
                        static_cast<std::size_t>(px.x())];
    const auto metres = static_cast<std::uint32_t>(std::min(255.0, std::max(1.0, std::ceil(pf.norm()))));
    colours[i] = metres << 24 | channel(bgr[2]) << 16 | channel(bgr[1]) << 8 | channel(bgr[0]);
    ++coloured;
  }
  colour_seconds_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  colour_returns_ += world.size();
  colour_coloured_ += coloured;
  if (++colour_frames_ % 600 == 0)
  {
    RCLCPP_INFO(node_->get_logger(),
                "RGB paired-frame colouring: %.1f ms per frame, %.1f %% of the returns coloured",
                1e3 * colour_seconds_ / static_cast<double>(colour_frames_),
                100.0 * static_cast<double>(colour_coloured_) / static_cast<double>(std::max<std::size_t>(colour_returns_, 1)));
  }
  return colours;
}

// Provide output format for LiDAR-visual BA
PointCloudXYZRGB::Ptr LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudFullRes,
                                    VIOManagerPtr vio_manager)
{
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (pcl_w_wait_pub->empty()) return laserCloudWorldRGB;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    // Camera colour of every return of this frame (colourScan; the RTK archive stores them per return). The
    // coloured returns form the display / legacy PCD cloud.
    scan_colours_ = colourScan(*pcl_w_wait_pub);
    if (pubLaserCloudFullRes->get_subscription_count() > 0 || pcd_save_en)
    {
      laserCloudWorldRGB->reserve(pcl_w_wait_pub->size());
      for (std::size_t i = 0; i < pcl_w_wait_pub->size(); ++i)
      {
        const std::uint32_t colour = scan_colours_[i];
        if (colour == 0) continue;
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_w_wait_pub->points[i].x;
        pointRGB.y = pcl_w_wait_pub->points[i].y;
        pointRGB.z = pcl_w_wait_pub->points[i].z;
        pointRGB.r = static_cast<std::uint8_t>((colour >> 16) & 0xFF);
        pointRGB.g = static_cast<std::uint8_t>((colour >> 8) & 0xFF);
        pointRGB.b = static_cast<std::uint8_t>(colour & 0xFF);
        laserCloudWorldRGB->push_back(pointRGB);
      }
    }
  }

  /*** Publish Frame (display only: skipped without a subscriber) ***/
  if (pubLaserCloudFullRes->get_subscription_count() > 0)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
    {
      pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
    }
    if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
    {
      pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
    }
    laserCloudmsg.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
    laserCloudmsg.header.frame_id = lio_world_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg);
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(root_dir + "Log/pcd/" + ss_time.str() + ".pcd");

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(root_dir + "Log/image/" + ss_time.str() + ".png", vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
  return laserCloudWorldRGB;
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
    laserCloudmsg.header.frame_id = lio_world_frame;
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudEffect,
                                     const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  laserCloudFullRes3.header.frame_id = lio_world_frame;
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr &pubOdomAftMapped)
{
  if (rtk_fusion_ && lio_world_frame == "lio_local" && !local_alignment_published)
  {
    gtsam::Pose3 alignment;
    if (rtk_fusion_->frontendAlignment(alignment))
    {
      geometry_msgs::msg::TransformStamped fixed;
      fixed.header.frame_id = "mine";
      fixed.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
      fixed.child_frame_id = lio_world_frame;
      const auto p = alignment.translation();
      const auto q = alignment.rotation().toQuaternion();
      fixed.transform.translation.x = p.x(); fixed.transform.translation.y = p.y(); fixed.transform.translation.z = p.z();
      fixed.transform.rotation.x = q.x(); fixed.transform.rotation.y = q.y();
      fixed.transform.rotation.z = q.z(); fixed.transform.rotation.w = q.w();
      alignment_broadcaster_->sendTransform(fixed);
      local_alignment_published = true;
    }
  }
  odomAftMapped.header.frame_id = lio_world_frame;
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  set_posestamp(odomAftMapped.pose.pose);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odomAftMapped.header;
  transform.child_frame_id = "aft_mapped";
  transform.transform.translation.x = _state.pos_end(0);
  transform.transform.translation.y = _state.pos_end(1);
  transform.transform.translation.z = _state.pos_end(2);
  transform.transform.rotation = geoQuat;
  tf_broadcaster_->sendTransform(transform);
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = lio_world_frame;
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = lio_world_frame;
  path.header.stamp = msg_body_pose.header.stamp;
  path.header.frame_id = lio_world_frame;
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
