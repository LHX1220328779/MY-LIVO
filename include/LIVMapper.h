/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <filesystem>
#include <csignal>
#include <limits>
#include <sstream>
#include <memory>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "extrinsic_precheck.h"

struct RtkFusion;

class LIVMapper
{
public:
  explicit LIVMapper(const rclcpp::Node::SharedPtr &node);
  ~LIVMapper();
  void initializeSubscribersAndPublishers();
  void initializeComponents();
  void initializeFiles();
  void run(const volatile std::sig_atomic_t &stop_requested);
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstSharedPtr &msg);
  void multi_lidar_pcl_cbk(
      const sensor_msgs::PointCloud2::ConstSharedPtr &msg,
      std::size_t source_index);
  void synchronizeMultiLidarFrames();
  void feedExtrinsicPrecheck(std::size_t source_index);
  void finishExtrinsicPrecheck(const string &reason);
  void loadMultiLidarCalibration();
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstSharedPtr &msg_in);
  void ins_odom_cbk(const nav_msgs::Odometry::ConstSharedPtr &msg_in);
  void ins_status_cbk(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &msg);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  PointCloudXYZRGB::Ptr publish_frame_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &publisher, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &publisher);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &publisher, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr &publisher);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr &publisher);
  void publish_path(const rclcpp::Publisher<nav_msgs::Path>::SharedPtr &publisher);
  void readParameters();
  void initializeMineFrame(double initialization_time);
  void publishReferenceTrajectory(double current_time);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  struct MinePose
  {
    double stamp = 0.0;
    V3D position = V3D::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  };

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, ins_odom_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double gyr_bias_rw_sigma = 0.001, acc_bias_rw_sigma = 0.001;
  double initial_gyr_bias_sigma = std::sqrt(1e-5);
  double blind_rgb_points = 0.0;
  // Look-back colouring of the LIVO RGB map (colourScan): camera views buffered every rgb_view_spacing m of camera
  // travel over the last rgb_view_span m (0 = current image only), colours sampled rgb_view_margin px inside the
  // image border; a view's weight ramps up over the next rgb_view_feather px (0 = the newest view alone).
  double rgb_view_spacing = 2.0, rgb_view_span = 80.0, rgb_view_feather = 90.0;
  int rgb_view_margin = 60;
  // Camera flat field (calibration camera.flat_field, camera-photo stage): log brightness of an image position
  // relative to the principal point, poly8 in x = (u - cx) / (W / 2), y = (v - cy) / (H / 2) (y clamped to the
  // fitted range); colourScan divides it out (publish/rgb_flat_field_en). Empty = none.
  std::vector<double> flat_field_coefficients_;
  double flat_field_y_lo_ = -1.0, flat_field_y_hi_ = 1.0;
  bool rgb_flat_field_en = true;
  std::vector<float> flat_gain_;  // colour gain per pixel, built at the first colourScan
  struct ColourView
  {
    SE3 T_f_w;                 // camera from world (native frontend frame)
    V3D centre;                // camera centre
    cv::Mat image;             // BGR; the VIO clones a new image per frame, the buffer only keeps a reference
    std::vector<float> depth;  // nearest depth of the view's own scan per 8 px cell
    std::vector<float> occluders;  // nearest depth of the recent frames' elevated returns per 8 px cell
  };
  // Occlusion check of colourScan: the elevated returns of the last rgb_occluder_frames frames (0 = own scan only).
  int rgb_occluder_frames = 20;
  std::deque<std::vector<Eigen::Vector3f>> occluder_frames_;
  std::deque<ColourView> colour_views_;
  std::vector<std::uint32_t> scan_colours_;  // colours of the current frame's returns (pcl_w_wait_pub order)
  double colour_seconds_ = 0.0;
  std::size_t colour_frames_ = 0, colour_returns_ = 0, colour_coloured_ = 0;
  std::vector<std::uint32_t> colourScan(const PointCloudXYZI &world);
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  std::size_t imu_stale_dropped_ = 0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;
  double imu_max_time_gap = 0.2;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  bool img_en = true;
  int imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  bool lidar_en = true;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  double rtk_reference_time_shift_ = 0.0;
  // Calibration-file copies used only for the console summary / overrides.
  double calib_lidar_frontend_offset_ = std::numeric_limits<double>::quiet_NaN();
  double calib_lidar_ins_offset_ = std::numeric_limits<double>::quiet_NaN();
  double calib_rtk_position_offset_ = std::numeric_limits<double>::quiet_NaN();
  double calib_ins_attitude_offset_ = std::numeric_limits<double>::quiet_NaN();
  string calibration_identity_, calibration_type_, calibration_file_;
  Eigen::Matrix4d rear_from_camera_ = Eigen::Matrix4d::Identity();
  bool vio_ekf_update_en = true;
  bool print_frame_timing = false;
  double status_interval_sec = 10.0;
  void logMemoryStatus();
  void releaseFrontendMaps();
  double last_status_print_time_ = -1.0;
  void printCalibrationSummary(const char *stage) const;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;

  struct MergedLidarTimestampInfo
  {
    struct SourceInfo
    {
      std::string topic;
      double header_time = 0.0;
      std::size_t point_count = 0;
    };
    std::vector<SourceInfo> sources;
    double merged_header_time = 0.0;
    double start_span_ms = 0.0;
  };
  deque<MergedLidarTimestampInfo> lid_timestamp_info_buffer;
  MergedLidarTimestampInfo current_lidar_ts_info;

  struct PendingLidarFrame
  {
    double header_time = 0.0;
    PointCloudXYZI::Ptr points;
  };

  struct LidarBodyExclusionRectangle
  {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
  };

  struct LidarSource
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    string topic;
    string transform_key;
    M3D rear_from_lidar_rotation = M3D::Identity();
    V3D rear_from_lidar_translation = V3D::Zero();
    vector<LidarBodyExclusionRectangle> body_exclusion_rectangles;
    deque<PendingLidarFrame> pending_frames;
    double last_header_time = -1.0;
  };

  bool multi_lidar_enabled = false;
  string multi_lidar_calibration_file;
  vector<string> multi_lidar_topics;
  vector<string> multi_lidar_transform_keys;
  vector<vector<LidarBodyExclusionRectangle>>
      multi_lidar_body_exclusion_rectangles;
  vector<std::unique_ptr<LidarSource>> lidar_sources;
  double multi_lidar_sync_tolerance = 0.005;
  double multi_lidar_body_exclusion_min_z = -3.0;
  double lidar_max_point_offset = 0.2;
  std::size_t multi_lidar_queue_size = 5;
  std::size_t multi_lidar_frame_count = 0;

  // Static-start extrinsic self-check (see extrinsic_precheck.h).
  livo::ExtrinsicPrecheck extrinsic_precheck_;
  livo::ExtrinsicPrecheckConfig extrinsic_precheck_config_;
  string extrinsic_precheck_reference_topic_ = "/front_lidar";
  // Process every n-th camera frame (30 Hz camera, divider 3 -> 10 Hz VIO).
  int camera_frame_divider = 1;

  deque<sensor_msgs::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  vector<double> camera_intrinsics;
  vector<double> camera_distortion;
  int camera_width = 1920;
  int camera_height = 1080;
  double IMG_POINT_COV;
  double vio_lidar_parity_ratio = 0.0;
  // > 0: side length / 2 (m) of the visual sparse map cube; it slides together with the LiDAR map but
  // can be much smaller, because every retained patch feature pins its full source image in memory
  // (Feature::img_). 0 = same cube as the LiDAR map (local_map/half_map_size * voxel size).
  double vio_visual_half_map_m = 0.0;
  // Visual points more than this far behind the camera (along the heading) are released at map slides; 0 = off.
  double vio_visual_behind_m = 0.0;
  // New visual points only after the camera moved this much since the last generation (standstill guard).
  double vio_new_points_min_move_m = 0.1, vio_new_points_min_turn_deg = 0.5;

  bool mine_frame_initialized = false;
  std::string lio_world_frame = "mine";
  bool local_alignment_published = false;
  bool imu_standard_rfu = false;
  bool imu_gyro_in_degrees = true;
  bool imu_acceleration_gravity_compensated = true;
  double imu_acceleration_scale = 1.0;
  M3D imu_acceleration_transform = M3D::Identity();
  bool rear_axle_to_imu_enabled = false;
  V3D imu_to_rear_axle = V3D::Zero();
  double mine_initialization_time = 0.0;
  std::vector<MinePose> mine_pose_samples;
  std::size_t mine_pose_publish_index = 0;
  nav_msgs::Path imu_reference_path;
  nav_msgs::Odometry imu_reference_odom;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;
  ofstream lio_geometry_log;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;
  std::unique_ptr<vk::PinholeCamera> camera_model;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> alignment_broadcaster_;
  std::unique_ptr<RtkFusion> rtk_fusion_;
  bool rtk_enabled = false;
  // false = run the fusion backend purely as exporter/smoother: no INS/RTK observations are fed,
  // the graph stays pure odometry, products stay in the INS-anchored mine frame (--no-rtk rounds).
  bool rtk_observations_enabled = true;
  // Qualified INS attitude as a bounded absolute attitude measurement in the
  // LIO update (positions/velocities stay unaided).
  bool frontend_attitude_aiding = false;
  double frontend_attitude_sigma_deg = 0.3, frontend_attitude_huber_deg = 1.0;
  double frontend_attitude_pull_fraction = 0.05;
  std::vector<double> frontend_attitude_axis_mask{1.0, 1.0, 0.0};
  // Weak-geometry gate: aid only while the previous scan's LiDAR-only translational information
  // (Schur complement of the rotation block, weakest eigenvalue, 1/m^2) is below this value
  // (0 = off). Released above frontend_attitude_weak_release_factor * threshold.
  double frontend_attitude_weak_geometry_info = 0.0, frontend_attitude_weak_release_factor = 1.5;
  // Armed after the first well-constrained scan: the first scans after map creation carry almost
  // no information (empty map) and must not switch the aid on during the stationary start.
  bool weak_geometry_active_ = false, weak_geometry_armed_ = false;
  long weak_geometry_frames_ = 0, weak_geometry_episodes_ = 0;
  double weakest_translation_information_ = 0.0;
  // Second condition: the propagated roll/pitch disagrees with the qualified INS attitude by more
  // than this [deg] (0 = geometry alone). Latched for the rest of the weak episode.
  double frontend_attitude_weak_tilt_deg = 0.0, attitude_gate_tilt_deg_ = -1.0;
  bool weak_tilt_latched_ = false;
  ofstream attitude_gate_log;
  struct PendingRtkLivoFrame
  {
    bool valid = false;
    double stamp = 0.0;
    double begin = 0.0;
    StatesGroup state;
    PointCloudXYZI::Ptr cloud;
    Eigen::Matrix<double, 6, 6> lidar_information =
        Eigen::Matrix<double, 6, 6>::Zero();
  };
  PendingRtkLivoFrame pending_rtk_livo_frame_;

  rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr voxel_pub;
  rclcpp::Subscription<sensor_msgs::PointCloud2>::SharedPtr sub_pcl;
  vector<rclcpp::Subscription<sensor_msgs::PointCloud2>::SharedPtr> sub_pcls;
  rclcpp::Subscription<sensor_msgs::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<nav_msgs::Odometry>::SharedPtr sub_ins_odom;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_ins_status;
  rclcpp::Subscription<sensor_msgs::Image>::SharedPtr sub_img;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubCurrentFrame;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pubImuPropOdom;
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr pubImuReferencePath;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pubImuReferenceOdom;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
