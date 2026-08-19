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
#include "backend/keyframe_manager.h"
#include "backend/loop_candidate_detector.h"
#include "backend/pose_graph_optimizer.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <filesystem>
#include <memory>

class LIVMapper
{
public:
  explicit LIVMapper(const rclcpp::Node::SharedPtr &node);
  ~LIVMapper();
  void initializeSubscribersAndPublishers();
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void handleBackendKeyframe();
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
  void loadMultiLidarCalibration();
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstSharedPtr &msg_in);
  void ins_odom_cbk(const nav_msgs::Odometry::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &publisher, VIOManagerPtr vio_manager);
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
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
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
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;

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

  bool mine_frame_initialized = false;
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

  bool backend_keyframes_enabled = false;
  bool backend_publish_keyframe_cloud = true;
  bool backend_pose_graph_enabled = false;
  bool backend_loop_detection_enabled = false;
  string backend_frontend_frame_id = "mine";
  string backend_map_frame_id = "map";
  string backend_body_frame_id = "body";
  my_livo::backend::KeyframeManager::Options backend_keyframe_options;
  my_livo::backend::PoseGraphOptimizer::Options backend_pose_graph_options;
  my_livo::backend::LoopCandidateDetector::Options
      backend_loop_detection_options;
  std::unique_ptr<my_livo::backend::KeyframeManager> keyframe_manager;
  std::unique_ptr<my_livo::backend::PoseGraphOptimizer>
      pose_graph_optimizer;
  std::unique_ptr<my_livo::backend::LoopCandidateDetector>
      loop_candidate_detector;
  nav_msgs::Path backend_keyframe_path;
  nav_msgs::Path backend_optimized_path;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

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

  rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr voxel_pub;
  rclcpp::Subscription<sensor_msgs::PointCloud2>::SharedPtr sub_pcl;
  vector<rclcpp::Subscription<sensor_msgs::PointCloud2>::SharedPtr> sub_pcls;
  rclcpp::Subscription<sensor_msgs::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<nav_msgs::Odometry>::SharedPtr sub_ins_odom;
  rclcpp::Subscription<sensor_msgs::Image>::SharedPtr sub_img;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudFullRes;
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
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr pubBackendKeyframePath;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr
      pubBackendKeyframeCloud;
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr pubBackendOptimizedPath;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr
      pubBackendOptimizedOdometry;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr
      pubBackendLoopCandidates;
  std::uint64_t backend_loop_marker_id = 0;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
