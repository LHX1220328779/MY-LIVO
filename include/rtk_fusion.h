#pragma once

#include "common_lib.h"
#include "loop_closure.h"
#include <limits>
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
    // Camera colour of every return of `cloud` (LIVMapper::colourScan): 0 = none, else
    // (view distance m, 1..255) << 24 | R << 16 | G << 8 | B.
    std::shared_ptr<const std::vector<std::uint32_t>> colours;
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
    // Sensor-frame support returns (float: |p| < 200 m -> < 20 um rounding). Kept for every geometry
    // anchor because any keyframe can become a revisit partner; float halves the dominant backend memory.
    std::vector<Eigen::Vector3f> support;
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
    // Native path length from the first keyframe (m): revisit links require driving away and back.
    double path = 0;
    gtsam::Pose3 raw, optimized;
    std::vector<Plane> planes;
    std::unordered_map<FusionVoxelKey, std::vector<std::uint32_t>, FusionVoxelKeyHash> cells;
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
    // INS absolute attitude observation (LIO body frame after the estimated
    // installation misalignment). Added once per keyframe, like positions.
    bool has_attitude = false;
    gtsam::Rot3 reference_rotation;
  };
  enum class GeometryKind { Local, Overlap, Revisit };
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
  // save_map: write the delivered clouds map_N.pcd / rgb_map_N.pcd at the end (false: archives only, e.g. the
  // calibration rounds, whose stages read scans.bin / colors.bin / sources.bin and delete the chunks unread).
  bool export_map_chunks = true;
  double recovery_stable_interval = 0.5;
  // A non-fixed startup keeps the frontend in its original local frame.
  // A single fixed frame transform later anchors it; its voxel map never jumps.
  bool local_frontend = false, frontend_alignment_known = true, graph_in_enu = true;
  // false (no-RTK rounds): maintain the graph/exports but ADOPT the raw frontend poses -- without
  // RTK anchoring the lidar-geometry/motion-prior graph distorts instead of refining (measured:
  // optimized drift 2x native on wuhu 120 s), so the honest product is the native trajectory.
  bool observations_enabled = true;
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
  // INS attitude can carry a different latency than the INS position output.
  // Scan time s uses the INS orientation sample stamped s + attitude_time_offset.
  double attitude_time_offset = 0;
  double positionTimeOffset() const { return time_offset; }
  // Append effective front-end settings to parameters.txt (values the node
  // parameters do not show because the calibration file overrides them).
  void appendParameters(const std::vector<std::pair<std::string, std::string>> &items) const;
  // Physical LiDAR origins (rear-axle frame, multi_lidar.topics order). With save_map
  // every archived return is tagged with the index of its nearest origin (sources.bin)
  // so the map can be split per physical LiDAR offline (extrinsic / timing audits).
  void setLidarOrigins(const std::vector<V3D> &origins) { lidar_origins = origins; }
  const std::filesystem::path &outputDirectory() const { return directory; }
  double attitudeTimeOffset() const { return attitude_time_offset; }
  Eigen::Vector3d installationMisalignmentDeg() const { return final_misalignment_deg_; }
  V3D final_misalignment_deg_ = V3D::Zero();
  double horizontal_sigma = 0.03, vertical_sigma = 0.05, covariance_scale = 1;
  double reported_model_sigma_xy = 0, reported_model_sigma_z = 0;
  double max_horizontal_sigma = 0.15, max_vertical_sigma = 0.25;
  double huber_delta = 2.8, key_interval = 1.0, key_min_interval = 0.5;
  double position_factor_interval = 0.5;
  double geometry_anchor_interval = 0.5;
  double key_translation = 3, key_rotation = 3;
  // Native pose steps that disagree with the INS-derived IMU step by more
  // than this within one frame interval (front-end init settling, map
  // relocalization) get bracketed by immediate correction knots so the
  // interpolated global correction never smears a native discontinuity
  // across a full keyframe interval.
  double key_discontinuity = 0.05;
  bool pending_discontinuity_key = false;
  double plane_voxel = 2, plane_thickness = 0.05, plane_noise_floor = 0.05;
  double revisit_radius = 8, revisit_age = 15, match_distance = 1.2;
  // A revisit needs the vehicle to have driven away and come back: native path length between both
  // keyframes >= revisit_min_path. Without it a standstill links every knot to the knots >= revisit_age
  // earlier at the same spot (truck29raw 20261013: 300 s parking -> dense graph, 0.7-0.9 s per solve).
  double revisit_min_path = 20;
  // Standstill knot spacing: while the native motion since the last keyframe stays below
  // stationary_translation / stationary_rotation and the native speed below stationary_speed, knots are
  // key_stationary_interval apart instead of key_interval (0 = legacy spacing everywhere).
  double key_stationary_interval = 5, stationary_translation = 0.05, stationary_rotation = 0.2 * M_PI / 180,
         stationary_speed = 0.1;
  // Standstill hold: consecutive knots while parked (same thresholds) are also tied by their native increment
  // with fixed sigmas. The odometry of a static scene does not random-walk like while driving (motionSigmas),
  // and without this the optimized yaw of a parking followed the RTK noise once its knots were sparse
  // (truck29raw 20261013: 0.13-0.21 deg across a 300 s parking against the static native pose). <= 0 = off.
  double standstill_hold_rotation = 0.005 * M_PI / 180, standstill_hold_translation = 0.002;
  bool knot_parked = false;
  // iSAM2 relinearization thresholds (pose rotation rad / translation m, misalignment rad). The final
  // batch LM relinearizes everything; online thresholds only bound per-update work.
  double relinearize_rotation = 0.0001, relinearize_translation = 0.002, relinearize_misalignment = 0.0001;
  // Covisibility/revisit candidate indices are rebuilt from the solved poses every index_rebuild_keys
  // keyframes (new anchors are inserted immediately); 1 = every keyframe. Full-graph online cost logging
  // every online_log_interval keyframes (1 = every keyframe).
  int index_rebuild_keys = 1, online_log_interval = 1;
  std::size_t keys_since_index_rebuild = 0;
  double last_memory_log = -1e18;
  // The initial standstill keeps the dense knots: it carries the start anchor (gravity datum, averaged INS
  // fixes) that the first metres of the route are judged against. Spacing applies after the first drive.
  bool motion_seen = false;
  // Final submap-to-submap loop closure over revisited road sections (loop_closure.h); needs the scan archive.
  LoopClosureConfig loop_config;
  // Accepted loop factors (keyframe indices + factor); finalizeOptimization adds them and enforces the RTK bound.
  struct LoopFactorRecord
  {
    std::size_t a = 0, b = 0;
    gtsam::NonlinearFactor::shared_ptr factor;
    std::size_t display = 0;  // index into loop_display
  };
  std::vector<LoopFactorRecord> collectLoopClosures();
  // RViz /rtk/graph: every registered revisit pair of the final loop closure. state 0 = kept in the final
  // solve, 1 = rejected by registration (loops.csv reason), 2 = accepted but dropped by the RTK bound.
  struct LoopDisplay
  {
    std::size_t a = 0, b = 0;
    int state = 1;
  };
  std::vector<LoopDisplay> loop_display;
  bool finalized = false;
  std::size_t last_relinearized = 0, last_reeliminated = 0, last_cliques = 0;
  gtsam::ISAM2Params solverParameters() const;
  void indexAnchor(std::size_t i);
  double publish_interval = 1, last_publish = -1;
  // INS attitude factors: bounded absolute roll/pitch/yaw evidence. Sigmas are
  // per body axis (x=right, y=forward, z=up) in radians after configuration.
  bool attitude_enabled = true, misalignment_initialized = false;
  V3D attitude_sigma = V3D(0.15, 0.15, 0.3) * (M_PI / 180.0);
  V3D misalignment_prior_sigma = V3D(0.3, 0.3, 0.5) * (M_PI / 180.0);
  double attitude_huber = 2.8;
  // A single-antenna INS holds heading on the gyro while stationary and only
  // re-aligns it once the vehicle moves. Attitude factors are therefore added
  // only when the native speed exceeds attitude_min_speed and the vehicle has
  // been moving for attitude_motion_age seconds (sign-plate epochs before the
  // first turn disagreed by ~0.1 deg / 7 cm with stationary INS heading).
  double attitude_min_speed = 1.0, attitude_motion_age = 5.0, motion_start_stamp = -1;
  std::size_t accepted_attitudes = 0;
  int minimum_satellites = 12, max_planes = 800, max_pair_factors = 120;
  int local_neighbors = 3, local_map_frames = 30, queue_capacity = 8;
  int export_chunk_points = 1000000;
  // 仅限制强度地图落盘距离，不改存档和估计器；超过此虚拟雷达距离的点不写入 map_N。
  // 0 表示不截断；RGB 使用下面的独立距离上限。
  double save_map_max_range = 0.0;
  // 独立彩色产品保留全部同帧着色回波，不套用强度地图筛选。
  // <0 表示沿用强度图距离上限，0 表示不截断。
  double save_rgb_max_range = -1.0;
  // 全图每体素保留最优原始观测（距离 + 20 米 × 角速度）；0 表示保留全部着色回波。
  double save_rgb_voxel = 0.03;
  // Map filter of map_N.pcd (intensity map): per chunk one return per save_map_filter cube, the one nearest to
  // its LiDAR; every scan participates, including stationary scans. map_rows.bin follows. 0 = off.
  double save_map_filter = 0.01;
  // Saved geometry map (map_N.pcd): range priority. Per save_map_priority_voxel cell the nearest supported
  // observation r_min (a scan with >= 2 returns in the cell) is found first; returns of that cell farther than
  // max(save_map_priority_ratio * r_min, r_min + save_map_priority_margin) are not written. A far scan therefore
  // only contributes where no nearer scan saw the surface (the coverage of save_map_max_range is kept), and the
  // far-lane copy of a surface no longer blurs the near-lane copy. ratio 0 = off (every return within the cap).
  // Switch of the whole rule (occupied + seen-free), default on. Off: a chunk keeps every return of its own
  // scans (on: a far scan's chunk lacks the poles / opposite lane that another pass saw nearer).
  bool save_map_priority_enabled = true;
  double save_map_priority_ratio = 1.5;
  double save_map_priority_margin = 2.0;
  double save_map_priority_voxel = 0.5;
  // Seen-free part of the same rule: a return within save_map_priority_free_range metres of its LiDAR marks the
  // half-voxel cells 0.6 and 1.2 voxels in front of it along its physical beam (mount from sources.bin) as seen
  // free at its range r_f; a row in such a cell farther than max(ratio * r_f, r_f + margin) is not written. It
  // removes the far-range smear of thin objects that lands in the empty cells beside them (no near scan put
  // returns there, so the occupied rule kept it); a static surface is never seen through. Not applied in the
  // two lowest occupied cells of each column: a grazing beam's "front" lies inside the ground's own cell and
  // would trim the upper half of the far scans' ground band (20261013: lowered > 5 mm on 8 % of the rough
  // columns). 0 = off.
  double save_map_priority_free_range = 25.0;
  // Ground detail fill: prefer the nearest original on-plane return of each empty 5 cm sub-column.
  // If the plane is unsupported or has no agreeing candidate, keep that sub-column's nearest original
  // observation instead of deleting measured road relief. Non-ground pole/wall filtering is unchanged;
  // volumetric vegetation may fill empty 0.1 m sub-voxels. Coordinates are never synthesized or flattened.
  bool save_map_priority_fill = true;
  std::vector<int> accepted_modes;
  std::filesystem::path directory;
  std::ofstream archive, color_archive, frame_index, diagnostics, optimization_log, history_log, association_log, geometry_history, path_log;
  std::ofstream source_archive;
  std::vector<V3D> lidar_origins;
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
  // /rtk/graph (MarkerArray, transient local): keyframes coloured by RTK residual, online revisit links,
  // final loop closures and a status text. /rtk/map_final: 0.25 m voxel map of the delivered cloud,
  // published once by the map export when RViz is subscribed.
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr graph_publisher;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr final_map_publisher;
  void publishGraph(double latest_error, double max_error);
  // Range-priority cells of the saved geometry map: key -> nearest supported range, sharded by key; `free`:
  // half-voxel cells seen free -> nearest range of a scan that looked through them (empty = check off).
  using NearestCells = std::vector<std::unordered_map<std::uint64_t, float>>;
  struct PriorityCells
  {
    NearestCells occupied, free;
    std::unordered_map<std::uint64_t, std::int64_t> ground;  // column (x,y cell) -> lowest occupied z cell
    // detail fill (save_map_priority_fill, detailFill): ground column -> plane z = z0 + gx dx + gy dy of its written
    // ground rows (dx, dy from the column corner), its 10x10 sub-columns holding one (bit iy * 10 + ix) and per
    // empty sub-column the nearest on-plane and fallback original range codes (0 = none); volumetric cell ->
    // its 5x5x5 sub-voxels holding a written row
    struct GroundPlane
    {
      double z0, gx, gy;
      std::array<std::uint64_t, 2> occupied;
      // First 100: closest on-plane candidate; last 100: closest original candidate if no on-plane one exists.
      std::array<std::uint8_t, 200> nearest;
    };
    std::unordered_map<std::uint64_t, GroundPlane> ground_planes;
    std::unordered_map<std::uint64_t, std::array<std::uint64_t, 2>> vegetation;
  };
  // One archived scan -> delivered points (map_N.pcd rows), identical to appendCloud on the range-capped scan;
  // with `priority` the range-priority rule is applied, `ranges` receives the range of every written row,
  // `written` one bit per archived return of the scan (LSB first, 1 = written; map_rows.bin) and `origins` the
  // world position of every written row's physical beam origin (mount of its sources.bin LiDAR; `sources` =
  // open sources.bin, null = virtual LiDAR origin), `rows` the archive index of every written row (its colour in
  // colors.bin), `dropped` the rows the rule drops in the ground layer with their range (detail fill candidates).
  // Returns the number of returns inside the range cap.
  std::size_t exportFrame(const Frame &frame, std::ifstream &input, std::vector<float> &packed,
                          pcl::PointCloud<pcl::PointXYZI> &output, const PriorityCells *priority = nullptr,
                          std::vector<float> *ranges = nullptr, std::vector<std::uint8_t> *written = nullptr,
                          std::ifstream *sources = nullptr, std::vector<Eigen::Vector3f> *origins = nullptr,
                          std::size_t *filled = nullptr, std::vector<std::uint32_t> *rows = nullptr,
                          std::vector<std::pair<pcl::PointXYZI, float>> *dropped = nullptr) const;
  PriorityCells nearestObservation() const;
  void detailFill(PriorityCells &priority) const;
  bool fillsGap(const PriorityCells &priority, std::uint64_t key, bool ground_layer, const pcl::PointXYZI &p,
                float range) const;

  RtkFusion(const rclcpp::Node::SharedPtr &node_in, const M3D &rotation,
            const V3D &translation, const V3D &reference_lever,
            const std::string &output_root, double reference_time_shift = 0.0,
            double position_time_offset_override = std::numeric_limits<double>::quiet_NaN(),
            double attitude_time_offset_override = std::numeric_limits<double>::quiet_NaN());
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
              const std::vector<std::uint32_t> *colours = nullptr);
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
  void exportRgbMap() const;
  void exportIntensityMap() const;
  void exportMap();
  void accumulateSolverStats(const gtsam::ISAM2Result &result);
};
