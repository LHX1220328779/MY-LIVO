#pragma once

#include "common_lib.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <filesystem>
#include <thread>
#include <condition_variable>
#include <exception>

class ScanPlaneBatchFactor;

struct FusionVoxelKey
{
  std::int64_t x, y, z;
  bool operator==(const FusionVoxelKey &other) const
  { return x == other.x && y == other.y && z == other.z; }
};
struct FusionVoxelKeyHash
{
  std::size_t operator()(const FusionVoxelKey &key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }
};

// One concrete LIO-only backend. The original estimator is never mutated by
// this worker. All graph, scan-cache and archive access belongs to the worker.
struct RtkFusion
{
  struct Observation
  {
    double stamp;
    V3D position;
    // Display diagnostic only; includes INS attitude/lever uncertainty.
    V3D imu_position = V3D::Constant(std::numeric_limits<double>::quiet_NaN());
    M3D covariance;
    bool position_valid = true;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool orientation_valid = false;
    // Bit 1: unknown-covariance fallback; bit 2: receiver covariance.
    // Interpolation can contain both sources; never label it as reported-only.
    unsigned covariance_source = 0;
  };
  struct Quality { double stamp; int mode; int satellites; double fixed_since = -1; };
  struct Frame
  {
    double stamp, begin;
    gtsam::Pose3 raw;
    PointCloudXYZI::ConstPtr cloud;
    // Camera-colored points are already in the native frontend world frame.
    // The RTK backend applies only C(t) to them; geometry continues to use the
    // immutable native deskewed LiDAR cloud above.
    PointCloudXYZRGB::ConstPtr color_cloud;
    std::uint64_t offset = 0;
    std::uint32_t count = 0;
    std::uint64_t color_offset = 0;
    std::uint32_t color_count = 0;
    double frontend_residual = std::numeric_limits<double>::quiet_NaN();
    double frontend_speed = 0;
    V3D gravity = V3D(0, 0, -G_m_s2);
    gtsam::Matrix6 lidar_information = gtsam::Matrix6::Zero();
    V3D reference_imu = V3D::Constant(std::numeric_limits<double>::quiet_NaN());
  };
  struct Plane
  {
    V3D center = V3D::Zero(), normal = V3D::Zero();
    double variance = 0;
    M3D sqrt_covariance = M3D::Zero();
    // A line is only a candidate here. The other scan must independently
    // support the same type, common footprint and rigid registration.
    bool linear = false;
    V3D tangent = V3D::Zero();
    std::vector<V3D> support;
    Eigen::Vector2d support_min = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d support_max = -support_min;
    Eigen::Vector2d supportCoordinates(const V3D &point) const
    {
      const V3D delta = point - center;
      return linear ? Eigen::Vector2d(normal.dot(delta), 0) :
          Eigen::Vector2d(tangent.dot(delta), normal.cross(tangent).dot(delta));
    }
    void extendSupport(const V3D &point)
    {
      const auto uv = supportCoordinates(point);
      support_min = support_min.cwiseMin(uv);
      support_max = support_max.cwiseMax(uv);
    }
    bool covers(const V3D &point) const
    {
      const auto uv = supportCoordinates(point);
      // Gate measured tangential extent, NOT the normal displacement that
      // registration must recover. Tolerance only covers roundoff at edges.
      return (uv.array() >= support_min.array() - 1e-6).all() &&
          (uv.array() <= support_max.array() + 1e-6).all();
    }
    M3D projector() const
    {
      const M3D outer = normal * normal.transpose();
      return linear ? M3D(M3D::Identity() - outer) : outer;
    }
  };
  struct Keyframe
  {
    double stamp;
    gtsam::Pose3 raw, optimized;
    std::vector<Plane> planes;
    std::unordered_map<FusionVoxelKey, std::vector<std::size_t>, FusionVoxelKeyHash> cells;
    bool has_position = false;
    double position_information_weight = 0;
    // 0 accepted, 1 no bracket, 2 gap, 3 no status, 4 stale,
    // 5 unaccepted mode, 6 too few satellites, 7 recovering, 8 covariance,
    // The per-key temporal weight limits information, not factor omission.
    int reference_failure = 1;
    V3D reference = V3D::Zero();
    M3D reference_covariance = M3D::Zero();
    unsigned reference_covariance_source = 0;
    M3D angular_observability = M3D::Zero();
  };
  enum class GeometryKind { Overlap, Revisit };
  struct GeometryLink
  {
    std::size_t a, b, factor_index;
    GeometryKind kind;
  };
  struct GeometryProbe
  {
    std::size_t a, b;
    std::shared_ptr<const ScanPlaneBatchFactor> factor;
    gtsam::Pose3 previous_a, previous_b;
    double initial_rms = -1;
  };
  rclcpp::Node::SharedPtr node;
  M3D lidar_rotation;
  V3D lidar_translation, lever;
  bool require_status = true, save_map = false, color_enabled = false;
  double recovery_stable_interval = 0.5;
  // A non-fixed startup keeps the frontend in its original local frame.
  // A single fixed frame transform later anchors it; its voxel map never jumps.
  bool local_frontend = false, frontend_alignment_known = true, graph_in_enu = true;
  gtsam::Pose3 frontend_to_enu;
  V3D local_gravity = V3D(0, 0, -G_m_s2);
  // Captured before the first submitted scan. The stationary gravity datum
  // is an initial observation, not an attitude prior repeated along the route.
  double initial_gravity_stamp = -1;
  V3D initial_gravity = V3D::Zero();
  M3D initial_gravity_covariance = M3D::Zero();
  // Relative-motion uncertainty per sqrt(second), NOT absolute frontend
  // marginal covariance. No second IMU factor duplicates native LIO.
  V3D motion_rotation_sigma = V3D(0.02, 0.02, 0.03) * (M_PI / 180.0) / std::sqrt(0.5);
  V3D motion_translation_sigma = V3D(0.03, 0.03, 0.02) / std::sqrt(0.5);
  double motion_huber_rate = 1;
  double time_offset = 0, status_age = 0.1, interpolation_gap = 0.05;
  double horizontal_sigma = 0.03, vertical_sigma = 0.05, covariance_scale = 1;
  double reported_model_sigma_xy = 0, reported_model_sigma_z = 0;
  double max_horizontal_sigma = 0.15, max_vertical_sigma = 0.25;
  double huber_delta = 2.8, key_interval = 1.0, key_min_interval = 0.5;
  double position_factor_interval = 0.5;
  double geometry_anchor_interval = 0.5;
  double key_translation = 3, key_rotation = 3;
  double plane_voxel = 2, plane_thickness = 0.05, plane_noise_floor = 0.05;
  double revisit_radius = 8, revisit_age = 15, match_distance = 1.2;
  double publish_interval = 1, last_publish = -1;
  int minimum_satellites = 12, max_planes = 800, max_pair_factors = 120;
  int local_neighbors = 3, local_map_frames = 30, queue_capacity = 8;
  int export_chunk_points = 1000000;
  std::vector<int> accepted_modes;
  std::filesystem::path directory;
  std::ofstream archive, color_archive, frame_index, diagnostics, optimization_log, history_log, association_log, geometry_history, path_log;
  std::vector<gtsam::Pose3> last_logged_poses;
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::deque<Observation> observations;
  std::deque<Quality> qualities;
  std::deque<Frame> queue, recent_frames;
  std::vector<Frame> frames;
  std::vector<Keyframe> keyframes;
  std::vector<std::size_t> geometry_anchors;
  std::array<double, 4> key_timings{};
  int refinement_steps = 0;
  double refinement_translation = 0, refinement_rotation = 0;
  std::vector<GeometryLink> geometry_links, new_links;
  // Fixed correspondences survive graph rematching. Preserve eight early
  // regions and sixteen recent regions; bounded diagnostic work per update.
  std::vector<GeometryProbe> geometry_probes;
  std::size_t next_probe_slot = 8, last_probe_key = 0;
  std::unordered_map<FusionVoxelKey, std::vector<std::size_t>,
                     FusionVoxelKeyHash> spatial_keys;
  // Coarse actual surface occupancy selects covisible scan candidates only.
  // It is rebuilt from solved poses; it never moves or filters map returns.
  std::unordered_map<FusionVoxelKey, std::vector<std::size_t>,
                     FusionVoxelKeyHash> surface_keys;
  std::thread worker;
  std::exception_ptr failure;
  bool stopping = false;
  gtsam::ISAM2 solver;
  gtsam::Values estimate;
  std::size_t accepted_positions = 0, missing_positions = 0;
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr odom_publisher;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr map_publisher, current_publisher;

  RtkFusion(const rclcpp::Node::SharedPtr &node_in, const M3D &rotation,
            const V3D &translation, const V3D &reference_lever,
            const std::string &output_root);
  ~RtkFusion();
  void observe(const nav_msgs::Odometry &message);
  void observeStatus(const diagnostic_msgs::msg::DiagnosticArray &message);
  bool referenceAt(double stamp, Observation &result, int *failure_reason = nullptr);
  bool frontendReferenceAt(double stamp, const StatesGroup &state, Observation &result);
  bool frontendAlignment(gtsam::Pose3 &alignment);
  void initializeGravity(double stamp, const V3D &gravity, const M3D &covariance);
  void submit(double stamp, double begin, const StatesGroup &state,
              const PointCloudXYZI::Ptr &deskewed_cloud,
              const gtsam::Matrix6 &lidar_information,
              const PointCloudXYZRGB::Ptr &colored_world_cloud = nullptr);
  void finish();
  void process(Frame frame);
  void optimizeKeyframe(const Frame &frame);
  bool fitSurface(const V3D &sum, const M3D &scatter, std::size_t count, Plane &plane) const;
  std::vector<Plane> extractPlanes(const Frame &frame) const;
  std::size_t addGeometry(std::size_t a, std::size_t b,
      gtsam::NonlinearFactorGraph &graph, GeometryKind kind);
  void updatePoses();
  void initializePose(const gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial);
  gtsam::Vector6 motionSigmas(double duration) const;
  void logOptimization(const char *phase, int iteration, double before, double after);
  void finalizeOptimization();
  gtsam::Pose3 correctionAt(double stamp) const;
  gtsam::Pose3 poseAt(const Frame &frame) const;
  void appendCloud(const Frame &frame, pcl::PointCloud<pcl::PointXYZI> &cloud) const;
  void appendDisplayCloud(const Frame &frame, PointCloudXYZRGB &cloud) const;
  void publishCurrent(const Frame &frame);
  void publish(double stamp, bool force = false);
  void exportMap();
};
