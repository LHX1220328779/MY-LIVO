/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include "ros2_compat.h"
#include <thread>
#include <unistd.h>
#include <unordered_map>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

// A map return held inside a voxel until its plane is frozen. Plane fitting (init_plane) and the octree split
// only read the world point and its covariance; storing the full pointWithVar (four vectors + four 3x3
// matrices, 384 B) quadrupled the dominant LIO map memory (truck29raw 20261013: 9.7 GB RSS in 870 s).
// Stored in float: the world point (|p| < 2 km -> < 0.12 mm rounding, far below the 2-3 cm range noise) and the
// six distinct entries of its symmetric covariance. 36 B instead of 96 B (two double Eigen objects); the plane fit
// still accumulates in double.
struct VoxelPoint
{
  Eigen::Vector3f p;
  float c[6];  // xx xy xz yy yz zz
  VoxelPoint() = default;
  VoxelPoint(const pointWithVar &pv)  // NOLINT: implicit by design
      : p(pv.point_w.cast<float>()),
        c{static_cast<float>(pv.var(0, 0)), static_cast<float>(pv.var(0, 1)), static_cast<float>(pv.var(0, 2)),
          static_cast<float>(pv.var(1, 1)), static_cast<float>(pv.var(1, 2)), static_cast<float>(pv.var(2, 2))}
  {
  }
  Eigen::Vector3d point() const { return p.cast<double>(); }
  Eigen::Matrix3d cov() const
  {
    Eigen::Matrix3d m;
    m << c[0], c[1], c[2], c[1], c[3], c[4], c[2], c[4], c[5];
    return m;
  }
};

// Live LIO map size (diagnostic status line): octree nodes each own a VoxelPlane; points are the not yet frozen
// returns held in temp_points_ (capacity = allocated slots).
struct MapMemoryStats
{
  std::size_t roots = 0, nodes = 0, planes = 0, plane_objects = 0, points = 0, point_capacity = 0;
  double bytes = 0;
};

class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<VoxelPoint> temp_points_;
  // Points at the shared read-only empty_plane_ (is_plane_ = false) until the node is first fitted
  // (ensurePlane): about two thirds of the octree nodes never collect enough returns for a fit, and a VoxelPlane
  // per node (~420 B) was the largest part of the LIO map after the returns.
  VoxelPlane *plane_ptr_;
  static VoxelPlane empty_plane_;
  VoxelPlane *ensurePlane()
  {
    if (plane_ptr_ == &empty_plane_) plane_ptr_ = new VoxelPlane;
    return plane_ptr_;
  }
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  // Shared per-layer init thresholds (VoxelMapManager::config_setting_): a per-node vector copy cost ~60 B/node.
  const std::vector<int> *layer_init_num_ = nullptr;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = &empty_plane_;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    if (plane_ptr_ != &empty_plane_) delete plane_ptr_;
  }
  void init_plane(const std::vector<VoxelPoint> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(const rclcpp::Node::SharedPtr &node, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_;
  std::string world_frame_ = "mine";
  int current_frame_id_ = 0;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;
  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_ = 0;
  // Conditional LiDAR-only information with the current map held fixed.
  // This is NOT an independent LIO Between covariance or an IMU posterior.
  bool collect_geometry_diagnostics_ = false;
  Eigen::Matrix<double, 6, 6> lidar_information_body_ = Eigen::Matrix<double, 6, 6>::Zero();
  // Optional absolute body-attitude pseudo-measurement for the iterated update
  // (qualified INS attitude in this LIO world frame). Scan-to-map residuals
  // only observe attitude relative to the self-built map, so a distance-
  // proportional registration bias integrates into unbounded pitch/roll drift
  // and, through gravity leakage, into a diverging along-track velocity. The
  // measurement is convex-Huber bounded and is NOT included in the pure
  // LiDAR information diagnostics above.
  bool attitude_aiding_ = false;
  M3D attitude_measurement_ = M3D::Identity();
  // attitude_information_: absolute floor 1/sigma^2 (INS accuracy) that
  // dominates when LiDAR is weak. attitude_pull_fraction_: fraction of the
  // current LiDAR angular information, so that in rich scenes the attitude
  // still relaxes toward the absolute reference with a bounded time constant
  // (f=0.05 -> ~20 scans) instead of being locked to the self-built map.
  double attitude_information_ = 0.0, attitude_pull_fraction_ = 0.0, attitude_huber_rad_ = 0.0;
  // Per-body-axis mask (x=right/pitch, y=forward/roll, z=up/yaw). Gravity
  // leakage into velocity only involves roll/pitch; short-term yaw is left to
  // LiDAR so that a 0.1-0.2 deg INS heading wobble is not imprinted on the
  // local map (0.2 deg at 30 m is already 10 cm of map smear).
  V3D attitude_axis_mask_ = V3D(1.0, 1.0, 1.0);
  V3D attitude_residual_ = V3D::Zero();
  double attitude_weight_used_ = 0.0, lidar_angular_information_ = 0.0;
  double lidar_residual_rms_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat);
  Eigen::Matrix<double, 6, 6> normalizedLidarInformation(double rotation_radius_m) const;
  M3D pointCovarianceWorld(const M3D &imu_point_cross, const M3D &lidar_covariance) const;
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap();
  V3F RGBFromVoxel(const V3D &input_point);

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  MapMemoryStats memoryStats() const;
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_
