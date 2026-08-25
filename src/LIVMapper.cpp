/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include "backend/frame_transform.h"

#include <json/json.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <fstream>
#include <limits>
#include <set>

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

double SmoothWeakness(double condition_ratio, double soft_ratio,
                      double full_ratio)
{
  const double x = std::clamp(
      (soft_ratio - condition_ratio) / (soft_ratio - full_ratio),
      0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}
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
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = stampFromSec(node_->now().seconds());
  path.header.frame_id = "mine";
  imu_reference_path.header.frame_id = "mine";
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

LIVMapper::~LIVMapper()
{
  // Drain queued NDT jobs while the node, logger and publishers captured by
  // the result callback are still alive.
  if (loop_registration)
  {
    loop_registration->WaitUntilIdle();
    const auto statistics = loop_registration->statistics();
    RCLCPP_INFO(
        node_->get_logger(),
        "Loop registration stopped: enqueued=%lu, completed=%lu, "
        "converged=%lu, failed=%lu, queue_drops=%lu, max_queue=%zu, "
        "total_time=%.1f ms",
        static_cast<unsigned long>(statistics.enqueued),
        static_cast<unsigned long>(statistics.completed),
        static_cast<unsigned long>(statistics.converged),
        static_cast<unsigned long>(statistics.failed),
        static_cast<unsigned long>(statistics.queue_drops),
        statistics.maximum_queue_depth,
        statistics.total_registration_time_ms);
    loop_registration.reset();
  }
  if (loop_verifier)
  {
    const auto flushed = loop_verifier->Flush();
    handleLoopVerificationDecisions(flushed, false);
    const auto statistics = loop_verifier->statistics();
    RCLCPP_INFO(
        node_->get_logger(),
        "Loop verifier stopped: registrations=%lu, individually_valid=%lu, "
        "accepted=%lu, rejected=%lu, flushed=%zu",
        static_cast<unsigned long>(statistics.registrations),
        static_cast<unsigned long>(statistics.individually_valid),
        static_cast<unsigned long>(statistics.accepted),
        static_cast<unsigned long>(statistics.rejected), flushed.size());
  }
  // Stop and join the online tile worker before the exact final rebuild. Any
  // coalesced periodic request is superseded by the complete final snapshot.
  stopBackendGlobalMapWorker();
  if (optimized_global_map && keyframe_manager && pose_graph_optimizer)
  {
    const auto keyframes = keyframe_manager->keyframes();
    std::vector<my_livo::backend::Pose3d> optimized_poses;
    std::vector<std::uint8_t> eligible;
    if (global_pose_layer)
    {
      auto snapshot = global_pose_layer->global_map_snapshot();
      optimized_poses = std::move(snapshot.poses);
      eligible = std::move(snapshot.eligible);
    }
    else
    {
      optimized_poses = pose_graph_optimizer->optimized_poses();
      eligible.assign(optimized_poses.size(), 1U);
    }
    if (!keyframes.empty() && keyframes.size() == optimized_poses.size())
    {
      const auto result = optimized_global_map->Build(
          keyframes, optimized_poses, eligible, "final");
      const bool saved = optimized_global_map->SaveLatest();
      RCLCPP_INFO(
          node_->get_logger(),
          "Final optimized global map: revision=%zu, keyframes=%zu/%zu, "
          "submaps=%zu, suppressed_warp=%.3f m/%.3f deg, "
          "local_voxels=%zu, deformation_nodes=%zu, points=%zu->%zu, "
          "time=%.1f ms, saved=%d",
          result.revision, result.included_keyframes, result.keyframes,
          result.rigid_submaps,
          result.maximum_suppressed_position_warp_m,
          result.maximum_suppressed_angle_warp_deg,
          result.local_voxel_points,
          result.spatial_deformation_nodes,
          result.input_points,
          result.output_points, result.build_time_ms,
          static_cast<int>(saved));
    }
  }
  if (rtk_observation_buffer)
  {
    const auto statistics = rtk_observation_buffer->statistics();
    RCLCPP_INFO(
        node_->get_logger(),
        "RTK stopped: status=%lu (parsed=%lu), solutions=%lu, queries=%lu, "
        "observations=%lu, selected=%lu, rejected selection=%lu, "
        "innovation=%lu, max buffers=%zu/%zu",
        static_cast<unsigned long>(statistics.status_messages),
        static_cast<unsigned long>(statistics.status_parsed),
        static_cast<unsigned long>(statistics.solutions),
        static_cast<unsigned long>(statistics.queries),
        static_cast<unsigned long>(statistics.observations),
        static_cast<unsigned long>(backend_rtk_selected),
        static_cast<unsigned long>(backend_rtk_rejected_selection),
        static_cast<unsigned long>(backend_rtk_rejected_innovation),
        statistics.maximum_status_buffer_size,
        statistics.maximum_solution_buffer_size);
  }
}

void LIVMapper::publishBackendOptimizedProducts(
    bool allow_map_build, const std::string &map_build_reason)
{
  if (!pose_graph_optimizer || !keyframe_manager) return;
  const auto latest_keyframe = keyframe_manager->latest_keyframe();
  const std::size_t keyframe_count = keyframe_manager->size();
  if (!latest_keyframe || keyframe_count == 0) return;

  // Decide whether an O(N) graph snapshot is actually needed. High-rate
  // optimized odometry and map->odom use only the latest cached graph pose.
  const bool publish_full_path =
      keyframe_count == 1 || map_build_reason == "loop" ||
      backend_global_map_dirty.load() ||
      keyframe_count >= backend_last_optimized_path_count.load() +
                            backend_path_publish_interval;
  bool request_map = false;
  std::string map_reason;
  if (allow_map_build && optimized_global_map)
  {
    // exchange() avoids clearing a graph correction that arrives
    // concurrently after this scheduling decision.
    const bool dirty = backend_global_map_dirty.exchange(false);
    const bool force = map_build_reason == "final" ||
                       map_build_reason == "loop";
    if (dirty && !force &&
        !optimized_global_map->CanBuildGraphUpdate(keyframe_count))
    {
      backend_global_map_dirty.store(true);
    }
    else if (force || dirty ||
             optimized_global_map->NeedsPeriodicBuild(keyframe_count))
    {
      request_map = true;
      map_reason = dirty ? "graph_update" : map_build_reason;
      if (map_reason.empty()) map_reason = "periodic";
    }
  }

  std::vector<my_livo::backend::Keyframe::Ptr> keyframes;
  std::vector<my_livo::backend::Pose3d> local_slam_poses;
  std::vector<my_livo::backend::Pose3d> global_poses;
  std::vector<std::uint8_t> global_map_eligible;
  if (publish_full_path || request_map)
  {
    keyframes = keyframe_manager->keyframes();
    local_slam_poses = pose_graph_optimizer->optimized_poses();
    if (global_pose_layer)
    {
      global_pose_layer->SynchronizeLocalPoses(keyframes,
                                               local_slam_poses);
      auto snapshot = global_pose_layer->global_map_snapshot();
      global_poses = std::move(snapshot.poses);
      global_map_eligible = std::move(snapshot.eligible);
    }
    else
    {
      global_poses = local_slam_poses;
      global_map_eligible.assign(global_poses.size(), 1U);
    }
    if (keyframes.size() != keyframe_count ||
        local_slam_poses.size() != keyframe_count ||
        global_poses.size() != keyframe_count ||
        global_map_eligible.size() != keyframe_count)
      throw std::runtime_error(
          "Backend raw/local/global pose snapshots differ.");
  }

  const my_livo::backend::Pose3d latest_pose =
      global_poses.empty() ? latest_keyframe->T_global_body()
                           : global_poses.back();
  std_msgs::msg::Header output_header;
  output_header.frame_id = backend_map_frame_id;
  output_header.stamp = stampFromSec(latest_keyframe->timestamp());

  if (publish_full_path)
  {
    // Publish the local-SLAM reference in the fixed global frame using only
    // A0. Publishing it in the moving frontend frame would make RViz rigidly
    // move the entire historical line whenever map->odom changes, which looks
    // like a local-graph update even though T_slam is unchanged.
    nav_msgs::Path local_slam_path;
    local_slam_path.header.frame_id = backend_map_frame_id;
    local_slam_path.header.stamp = output_header.stamp;
    nav_msgs::Path optimized_path;
    optimized_path.header = output_header;
    nav_msgs::Path quarantined_path;
    quarantined_path.header = output_header;
    optimized_path.poses.reserve(keyframes.size());
    local_slam_path.poses.reserve(keyframes.size());
    for (std::size_t index = 0; index < keyframes.size(); ++index)
    {
      const auto &T_slam_body = local_slam_poses[index];
      const auto T_aligned_local_body =
          backend_global_pose_options.T_global_slam * T_slam_body;
      geometry_msgs::PoseStamped local_pose;
      local_pose.header.frame_id = backend_map_frame_id;
      local_pose.header.stamp = stampFromSec(keyframes[index]->timestamp());
      local_pose.pose.position.x = T_aligned_local_body.translation.x();
      local_pose.pose.position.y = T_aligned_local_body.translation.y();
      local_pose.pose.position.z = T_aligned_local_body.translation.z();
      local_pose.pose.orientation.x = T_aligned_local_body.rotation.x();
      local_pose.pose.orientation.y = T_aligned_local_body.rotation.y();
      local_pose.pose.orientation.z = T_aligned_local_body.rotation.z();
      local_pose.pose.orientation.w = T_aligned_local_body.rotation.w();
      local_slam_path.poses.push_back(local_pose);

      const auto &T_global_body = global_poses[index];
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = backend_map_frame_id;
      pose.header.stamp = stampFromSec(keyframes[index]->timestamp());
      pose.pose.position.x = T_global_body.translation.x();
      pose.pose.position.y = T_global_body.translation.y();
      pose.pose.position.z = T_global_body.translation.z();
      pose.pose.orientation.x = T_global_body.rotation.x();
      pose.pose.orientation.y = T_global_body.rotation.y();
      pose.pose.orientation.z = T_global_body.rotation.z();
      pose.pose.orientation.w = T_global_body.rotation.w();
      if (global_map_eligible[index] != 0U)
      {
        // Keep the Path topologically honest after a quarantined gap. RViz
        // otherwise connects two accepted components with one false segment.
        if (index > 0 && global_map_eligible[index - 1] == 0U)
          optimized_path.poses.clear();
        quarantined_path.poses.clear();
        optimized_path.poses.push_back(pose);
      }
      else
      {
        // nav_msgs/Path has no segment-break primitive. Keep only the newest
        // continuous quarantined component so RViz never draws a fictitious
        // diagonal line across a deliberate recovery reanchor.
        if (!quarantined_path.poses.empty() && index > 0)
        {
          const double local_step =
              (local_slam_poses[index].translation -
               local_slam_poses[index - 1].translation).norm();
          const double global_step =
              (global_poses[index].translation -
               global_poses[index - 1].translation).norm();
          const auto local_relative = local_slam_poses[index - 1].inverse() *
              local_slam_poses[index];
          const auto global_relative = global_poses[index - 1].inverse() *
              global_poses[index];
          const double relative_angle_error_deg = 180.0 / std::acos(-1.0) *
              local_relative.rotation.angularDistance(
                  global_relative.rotation);
          if (std::abs(global_step - local_step) > 1.0 ||
              relative_angle_error_deg > 1.0)
            quarantined_path.poses.clear();
        }
        quarantined_path.poses.push_back(pose);
      }
    }
    pubBackendLocalSlamPath->publish(local_slam_path);
    pubBackendOptimizedPath->publish(optimized_path);
    pubBackendQuarantinedPath->publish(quarantined_path);
    backend_last_optimized_path_count.store(keyframe_count);
  }

  nav_msgs::Odometry optimized_odometry;
  optimized_odometry.header = output_header;
  optimized_odometry.child_frame_id = backend_body_frame_id;
  optimized_odometry.pose.pose.position.x = latest_pose.translation.x();
  optimized_odometry.pose.pose.position.y = latest_pose.translation.y();
  optimized_odometry.pose.pose.position.z = latest_pose.translation.z();
  optimized_odometry.pose.pose.orientation.x = latest_pose.rotation.x();
  optimized_odometry.pose.pose.orientation.y = latest_pose.rotation.y();
  optimized_odometry.pose.pose.orientation.z = latest_pose.rotation.z();
  optimized_odometry.pose.pose.orientation.w = latest_pose.rotation.w();
  for (int row = 0; row < 6; ++row)
    for (int column = 0; column < 6; ++column)
      optimized_odometry.pose.covariance[row * 6 + column] =
          latest_keyframe->odom_covariance()(row, column);
  pubBackendOptimizedOdometry->publish(optimized_odometry);

  if (backend_map_frame_id != backend_frontend_frame_id)
  {
    const auto T_map_odom = my_livo::backend::ComputeMapToOdom(
        latest_pose, latest_keyframe->T_odom_body());
    geometry_msgs::msg::TransformStamped transform;
    transform.header = output_header;
    transform.child_frame_id = backend_frontend_frame_id;
    transform.transform.translation.x = T_map_odom.translation.x();
    transform.transform.translation.y = T_map_odom.translation.y();
    transform.transform.translation.z = T_map_odom.translation.z();
    transform.transform.rotation.x = T_map_odom.rotation.x();
    transform.transform.rotation.y = T_map_odom.rotation.y();
    transform.transform.rotation.z = T_map_odom.rotation.z();
    transform.transform.rotation.w = T_map_odom.rotation.w();
    tf_broadcaster_->sendTransform(transform);
  }

  if (request_map)
    requestBackendGlobalMapBuild(
        keyframes, global_poses, global_map_eligible, map_reason,
        latest_keyframe->timestamp());
}

void LIVMapper::requestBackendGlobalMapBuild(
    const std::vector<my_livo::backend::Keyframe::Ptr> &keyframes,
    const std::vector<my_livo::backend::Pose3d> &optimized_poses,
    const std::vector<std::uint8_t> &eligible,
    const std::string &reason, double timestamp)
{
  if (!optimized_global_map || keyframes.empty() ||
      keyframes.size() != optimized_poses.size() ||
      keyframes.size() != eligible.size())
    return;
  std::lock_guard<std::mutex> lock(backend_global_map_worker_mutex);
  if (backend_global_map_worker_stop) return;
  if (!backend_global_map_worker_started)
  {
    backend_global_map_worker_started = true;
    backend_global_map_worker_thread =
        std::thread(&LIVMapper::backendGlobalMapWorkerLoop, this);
  }
  BackendGlobalMapRequest request;
  request.keyframes = keyframes;
  request.optimized_poses = optimized_poses;
  request.eligible = eligible;
  request.reason = reason;
  request.timestamp = timestamp;
  // Only the newest snapshot is useful. Preserve full-rebuild priority if a
  // graph update is coalesced with newer periodic append requests.
  if (backend_global_map_pending &&
      backend_global_map_pending->reason == "graph_update")
    request.reason = "graph_update";
  backend_global_map_pending = std::move(request);
  backend_global_map_worker_cv.notify_one();
}

void LIVMapper::publishBackendCurrentFrame()
{
  if (!pubBackendCurrentFrameGlobal || !keyframe_manager ||
      !global_pose_layer || !feats_undistort || feats_undistort->empty())
    return;
  const auto latest_keyframe = keyframe_manager->latest_keyframe();
  if (!latest_keyframe) return;

  // Hold the most recently optimized global<-frontend correction constant
  // between keyframes. This affects only this display cloud; scan matching
  // and the frontend voxel map continue to use the untouched LIO state.
  const my_livo::backend::Pose3d T_global_frontend =
      latest_keyframe->T_global_body() *
      latest_keyframe->T_odom_body().inverse();
  const my_livo::backend::Pose3d T_frontend_body(
      _state.rot_end, _state.pos_end);
  const my_livo::backend::Pose3d T_global_body =
      T_global_frontend * T_frontend_body;

  my_livo::backend::KeyframeCloud cloud_global;
  cloud_global.reserve(feats_undistort->size());
  for (const PointType &point_lidar : feats_undistort->points)
  {
    const V3D point_body =
        extR * V3D(point_lidar.x, point_lidar.y, point_lidar.z) + extT;
    const V3D point_global = T_global_body * point_body;
    if (!point_global.allFinite()) continue;
    my_livo::backend::KeyframePoint point;
    point.x = static_cast<float>(point_global.x());
    point.y = static_cast<float>(point_global.y());
    point.z = static_cast<float>(point_global.z());
    point.intensity = point_lidar.intensity;
    cloud_global.push_back(point);
  }
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud_global, message);
  message.header.frame_id = backend_map_frame_id;
  message.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  pubBackendCurrentFrameGlobal->publish(message);
}

void LIVMapper::backendGlobalMapWorkerLoop()
{
  while (true)
  {
    BackendGlobalMapRequest request;
    {
      std::unique_lock<std::mutex> lock(backend_global_map_worker_mutex);
      backend_global_map_worker_cv.wait(lock, [this]() {
        return backend_global_map_worker_stop ||
               backend_global_map_pending.has_value();
      });
      if (backend_global_map_worker_stop) return;
      // Give factors belonging to one keyframe/check a short window to merge
      // into a single newest-pose snapshot. The wait releases the mutex, so
      // producers remain non-blocking.
      backend_global_map_worker_cv.wait_for(
          lock,
          std::chrono::milliseconds(
              backend_global_map_request_coalesce_ms),
          [this]() { return backend_global_map_worker_stop; });
      if (backend_global_map_worker_stop) return;
      request = std::move(*backend_global_map_pending);
      backend_global_map_pending.reset();
    }
    try
    {
      const auto result = optimized_global_map->Build(
          request.keyframes, request.optimized_poses, request.eligible,
          request.reason);
      sensor_msgs::PointCloud2 message;
      pcl::toROSMsg(*result.cloud, message);
      message.header.frame_id = backend_map_frame_id;
      message.header.stamp = stampFromSec(request.timestamp);
      pubBackendOptimizedGlobalMap->publish(message);
      RCLCPP_INFO(
          node_->get_logger(),
          "Optimized global map revision %zu: reason=%s, mode=%s, "
          "keyframes=%zu/%zu, quarantined=%zu, processed=%zu, "
          "tiles=%zu/%zu, submaps=%zu, suppressed_warp=%.3f m/%.3f deg, "
          "local_voxels=%zu, deformation_nodes=%zu, points=%zu->%zu, "
          "time=%.1f ms",
          result.revision, request.reason.c_str(), result.build_mode.c_str(),
          result.included_keyframes, result.keyframes,
          result.quarantined_keyframes, result.processed_keyframes,
          result.updated_tiles, result.total_tiles,
          result.rigid_submaps,
          result.maximum_suppressed_position_warp_m,
          result.maximum_suppressed_angle_warp_deg,
          result.local_voxel_points,
          result.spatial_deformation_nodes,
          result.input_points, result.output_points, result.build_time_ms);
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Optimized global-map worker failed: %s",
          error.what());
    }
  }
}

void LIVMapper::stopBackendGlobalMapWorker()
{
  {
    std::lock_guard<std::mutex> lock(backend_global_map_worker_mutex);
    if (!backend_global_map_worker_started) return;
    backend_global_map_worker_stop = true;
    backend_global_map_pending.reset();
  }
  backend_global_map_worker_cv.notify_all();
  if (backend_global_map_worker_thread.joinable())
    backend_global_map_worker_thread.join();
  backend_global_map_worker_started = false;
}

void LIVMapper::handleRtkForKeyframe(
    const my_livo::backend::Keyframe::Ptr &keyframe)
{
  if (!keyframe || !rtk_observation_buffer) return;
  const auto query =
      rtk_observation_buffer->GetObservationAt(keyframe->timestamp());
  const my_livo::backend::Pose3d backend_pose_before =
      keyframe->T_slam_body();
  const auto write_diagnostics =
      [this, &keyframe, &query, &backend_pose_before](
          const char *decision, bool factor_added,
          const my_livo::backend::Pose3d &backend_pose_after) {
        if (!rtk_fusion_diagnostics) return;
        rtk_fusion_diagnostics->RecordKeyframe(
            *keyframe, query, decision, factor_added,
            backend_pose_before, backend_pose_after);
      };
  const auto write_decision = [this, &keyframe](
      const char *decision,
      const std::optional<my_livo::backend::RtkObservation> &observation,
      const Eigen::Vector3d &innovation, double chi2, bool factor_added) {
    if (!backend_rtk_decision_stream.is_open()) return;
    backend_rtk_decision_stream << std::setprecision(17) << keyframe->id()
        << ',' << keyframe->timestamp() << ',' << decision << ','
        << static_cast<int>(factor_added);
    if (observation)
      backend_rtk_decision_stream << ',' << observation->position.x() << ','
          << observation->position.y() << ',' << observation->position.z();
    else
      backend_rtk_decision_stream << ",,,";
    backend_rtk_decision_stream << ',' << innovation.x() << ','
        << innovation.y() << ',' << innovation.z() << ',' << chi2 << '\n';
    backend_rtk_decision_stream.flush();
  };
  if (!query.observation)
  {
    write_decision(
        my_livo::backend::RtkObservationRejectReasonToString(query.reason),
        std::nullopt, Eigen::Vector3d::Zero(),
        std::numeric_limits<double>::quiet_NaN(), false);
    write_diagnostics(
        my_livo::backend::RtkObservationRejectReasonToString(query.reason),
        false, backend_pose_before);
    return;
  }
  ++backend_rtk_candidates;
  if (!backend_rtk_fusion_enabled)
  {
    write_decision("fusion_disabled", query.observation,
                   Eigen::Vector3d::Zero(), 0.0, false);
    write_diagnostics("fusion_disabled", false, backend_pose_before);
    return;
  }
  if (!global_pose_layer)
    throw std::logic_error("RTK fusion requires the global pose layer.");

  const auto &lio_observability = keyframe->lio_observability();
  double lio_observability_score = 0.0;
  if (backend_lio_observability_guard_enabled &&
      lio_observability.valid &&
      lio_observability.effective_features >= 1000U)
  {
    const double translation_weakness = SmoothWeakness(
        lio_observability.translation_condition_ratio,
        backend_lio_observability_translation_soft_ratio,
        backend_lio_observability_translation_full_ratio);
    const double rotation_weakness = SmoothWeakness(
        lio_observability.rotation_condition_ratio,
        backend_lio_observability_rotation_soft_ratio,
        backend_lio_observability_rotation_full_ratio);
    // Both position and attitude geometry must be weak. A single flat-ground
    // direction is not sufficient evidence for stronger global assistance.
    lio_observability_score = std::min(
        translation_weakness, rotation_weakness);
  }
  const double lio_constraint_gain = 1.0 +
      (backend_lio_observability_maximum_constraint_gain - 1.0) *
          lio_observability_score;

  const bool select_global_observation =
      rtk_factor_selector->ShouldSelect(
          keyframe->id(), *query.observation);
  Eigen::Vector3d innovation = Eigen::Vector3d::Zero();
  double chi2 = 0.0;
  my_livo::backend::GlobalPoseLayer::RtkUpdate rtk_update;
  if (select_global_observation)
  {
    innovation = keyframe->T_global_body().translation -
        query.observation->position;
    Eigen::Matrix3d innovation_covariance =
        query.observation->position_covariance;
    innovation_covariance.diagonal().array() +=
        backend_rtk_innovation_prediction_sigma_m *
        backend_rtk_innovation_prediction_sigma_m;
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(
        innovation_covariance);
    if (decomposition.info() != Eigen::Success)
      throw std::runtime_error(
          "RTK innovation covariance is not invertible.");
    chi2 = innovation.dot(decomposition.solve(innovation));
    if (!std::isfinite(chi2))
      throw std::runtime_error(
          "RTK innovation statistic is not finite.");

    // Position observations remain sparse and affect only C(s). The velocity
    // safety observer below has an independent, faster failure-only cadence.
    rtk_update = global_pose_layer->AddRtkObservation(
        keyframe->id(), *query.observation);
    backend_rtk_velocity_regime = rtk_update.feasibility.regime;
    if (rtk_update.correction_field.field_changed ||
        rtk_update.quarantine_started)
      backend_global_map_dirty.store(true);
    if (rtk_update.quarantine_started)
      RCLCPP_ERROR(
          node_->get_logger(),
          "Global-map quarantine started at KF %lu: later keyframe clouds "
          "remain available for recovery but are excluded from the trusted "
          "global map until a new segment is accepted.",
          static_cast<unsigned long>(keyframe->id()));
  }

  const bool update_velocity_guard = rtk_velocity_guard &&
      rtk_velocity_selector && rtk_velocity_selector->ShouldSelect(
          keyframe->id(), *query.observation);
  if (update_velocity_guard)
  {
    std::optional<Eigen::Vector3d> receiver_velocity;
    if (query.observation->has_velocity)
      receiver_velocity = query.observation->velocity;
    const auto velocity_guard = rtk_velocity_guard->AddObservation(
        keyframe->id(), query.observation->timestamp,
        query.observation->position,
        keyframe->T_global_body().translation, _state.vel_end,
        backend_rtk_velocity_regime, receiver_velocity);
    rtk_velocity_selector->MarkSelected(
        keyframe->id(), *query.observation);
    backend_latest_rtk_velocity_result = velocity_guard;
    backend_have_latest_rtk_velocity_result = true;
    if (velocity_guard.correction_applied)
    {
      _state.vel_end = velocity_guard.lio_velocity_after;
      const double variance_floor =
          backend_rtk_velocity_covariance_floor_mps *
          backend_rtk_velocity_covariance_floor_mps;
      for (int axis = 0; axis < 3; ++axis)
        _state.cov(7 + axis, 7 + axis) = std::max(
            _state.cov(7 + axis, 7 + axis), variance_floor);
      state_propagat.vel_end = _state.vel_end;
      state_propagat.cov = _state.cov;
      voxelmap_manager->state_.vel_end = _state.vel_end;
      voxelmap_manager->state_.cov = _state.cov;
      if (imu_prop_enable)
      {
        std::lock_guard<std::mutex> propagation_lock(mtx_buffer_imu_prop);
        latest_ekf_state = _state;
        latest_ekf_time = LidarMeasures.last_lio_update_time;
        state_update_flg = true;
      }
      RCLCPP_WARN(
          node_->get_logger(),
          "RTK velocity guard KF %lu: %s/%s, error=%.3f m/s, ratio=%.2f, "
          "bounded velocity correction=[%.3f, %.3f, %.3f] m/s",
          static_cast<unsigned long>(keyframe->id()),
          my_livo::backend::RtkVelocityGuardDecisionToString(
              velocity_guard.decision),
          my_livo::backend::RtkVelocitySourceToString(
              velocity_guard.velocity_source),
          velocity_guard.velocity_error_mps, velocity_guard.speed_ratio,
          velocity_guard.applied_correction.x(),
          velocity_guard.applied_correction.y(),
          velocity_guard.applied_correction.z());
    }
    if (velocity_guard.emergency_restart_state_changed)
    {
      bool automatic_restart_suppressed = false;
      ++backend_frontend_restart_request_count;
      automatic_restart_suppressed =
          !backend_frontend_segment_restart_enabled ||
          backend_frontend_segment_id >=
              backend_frontend_segment_maximum_automatic_restarts;
      if (!automatic_restart_suppressed)
      {
        if (backend_pending_frontend_segment_restart)
          throw std::logic_error(
              "Velocity recovery requested a second pending frontend "
              "segment restart.");
        backend_pending_frontend_segment_restart =
            PendingFrontendSegmentRestart{
                keyframe->id(), query.observation->timestamp,
                "velocity_divergence", 0.0, true,
                velocity_guard.filtered_rtk_velocity,
                query.observation->position};
      }
      else
      {
        rtk_velocity_guard->AcknowledgeEmergencyRestart(keyframe->id());
      }
      if (backend_frontend_restart_supervisor_stream.is_open())
      {
        backend_frontend_restart_supervisor_stream << std::setprecision(17)
            << backend_frontend_restart_request_count << ','
            << keyframe->id() << ',' << query.observation->timestamp << ','
            << (automatic_restart_suppressed ? "suppressed" : "scheduled")
            << ",velocity_divergence,0,velocity_divergence,0,1,0\n";
        backend_frontend_restart_supervisor_stream.flush();
      }
      RCLCPP_ERROR(
          node_->get_logger(),
          "Persistent status-4 receiver/LIO velocity divergence at KF %lu "
          "(error %.3f m/s, evidence %d): controlled local-segment "
          "recovery %s.",
          static_cast<unsigned long>(keyframe->id()),
          velocity_guard.velocity_error_mps,
          velocity_guard.emergency_restart_evidence,
          automatic_restart_suppressed ? "suppressed" : "scheduled");
    }
    const double recovery_post_velocity_error_mps =
        velocity_guard.baseline_available
        ? (velocity_guard.lio_velocity_after -
           velocity_guard.filtered_rtk_velocity).norm()
        : 0.0;
    const auto recovery_update =
        global_pose_layer->AddRecoveryObservation(
            keyframe->id(), *query.observation,
            velocity_guard.baseline_available, velocity_guard.active,
            recovery_post_velocity_error_mps);
    if (recovery_update.segment_reanchored)
    {
      backend_global_map_dirty.store(true);
      const auto &fit = recovery_update.relocalization;
      RCLCPP_WARN(
          node_->get_logger(),
          "Stable recovery segment reanchored at KF %lu (window %lu..%lu, "
          "path %.1f/%.1f m, scale %.4f, fit %.3f/%.3f m, yaw %.3f "
          "deg, attitude yaw %.3f deg). Strong velocity support remains "
          "active during quarantined elastic probation.",
          static_cast<unsigned long>(keyframe->id()),
          static_cast<unsigned long>(fit.window_start_keyframe_id),
          static_cast<unsigned long>(fit.keyframe_id),
          fit.local_path_length_m, fit.rtk_path_length_m,
          fit.path_scale_ratio, fit.planar_rms_m, fit.vertical_rms_m,
          fit.yaw_rad * 180.0 / M_PI,
          recovery_update.orientation_yaw_rad * 180.0 / M_PI);
    }
  }

  // Every valid query is audited, but only the independent ~1 Hz status-4
  // position cadence may create a production correction knot. Receiver
  // quaternion is never a direct correction-field observation.
  const auto elastic_update = global_pose_layer->AddElasticObservation(
      keyframe->id(), *query.observation, update_velocity_guard,
      lio_constraint_gain,
      lio_observability.translation_condition_ratio,
      lio_observability.rotation_condition_ratio);
  if (elastic_update.field_changed)
  {
    backend_global_map_dirty.store(true);
    RCLCPP_DEBUG(
        node_->get_logger(),
        "Regularized C2 elastic field KF %lu: segment=%lu, knots=%zu, "
        "position=%d, stiffness=%.3f, residual=%.3f m, gradient=%.4f m/m",
        static_cast<unsigned long>(keyframe->id()),
        static_cast<unsigned long>(elastic_update.segment_id),
        elastic_update.field.knot_count,
        static_cast<int>(update_velocity_guard),
        elastic_update.field.elastic_stiffness,
        elastic_update.residual_after_m,
        elastic_update.evaluation.first_derivative.displacement.norm());
  }
  if (elastic_update.segment_accepted)
  {
    if (!rtk_velocity_guard)
      throw std::logic_error(
          "Elastic recovery acceptance lacks its velocity guard.");
    rtk_velocity_guard->CompleteRecoveryTracking(keyframe->id());
    backend_latest_rtk_velocity_result.recovery_tracking = false;
    backend_latest_rtk_velocity_result.active = false;
    backend_have_recovery_frame_tracking_time = false;
    backend_global_map_dirty.store(true);
    RCLCPP_WARN(
        node_->get_logger(),
        "Elastic recovery segment accepted at KF %lu: trusted global-map "
        "output resumes from KF %lu after %zu low-rate position checks over "
        "%.1f m; strong recovery velocity tracking is now released.",
        static_cast<unsigned long>(keyframe->id()),
        static_cast<unsigned long>(
            elastic_update.trusted_start_keyframe_id),
        elastic_update.acceptance.window_size,
        elastic_update.acceptance.window_path_length_m);
  }

  // A sustained simultaneous loss of translational and rotational geometry,
  // confirmed while independent RTK velocity still agrees, is an earlier and
  // safer restart trigger than waiting for a ten-metre position failure.
  if (update_velocity_guard && backend_have_latest_rtk_velocity_result &&
      backend_frontend_segment_id == 0 &&
      !backend_pending_frontend_segment_restart)
  {
    const auto &velocity = backend_latest_rtk_velocity_result;
    const double velocity_error_after = velocity.baseline_available
        ? (velocity.lio_velocity_after - velocity.filtered_rtk_velocity).norm()
        : std::numeric_limits<double>::infinity();
    const bool geometry_failure =
        backend_lio_observability_guard_enabled &&
        backend_rtk_velocity_regime ==
            my_livo::backend::CorrectionRegime::kDegraded &&
        lio_observability_score >=
            backend_lio_observability_restart_score &&
        elastic_update.residual_after_m >=
            backend_lio_observability_restart_residual_m;
    const double planar_gradient_ratio =
        elastic_update.field.interval_peak_planar_gradient_m_per_m /
        backend_global_pose_options.regularized_field
            .maximum_planar_gradient_m_per_m;
    const double vertical_gradient_ratio =
        elastic_update.field.interval_peak_vertical_gradient_m_per_m /
        backend_global_pose_options.regularized_field
            .maximum_vertical_gradient_m_per_m;
    // A healthy-looking feature count is not sufficient when the causal
    // correction field is already saturated and its residual keeps growing.
    // This is direct evidence that frontend propagation is outrunning the
    // admissible low-frequency deformation; restart before metres of error
    // accumulate instead of increasing the map-warp gradient.
    const bool elastic_tracking_saturation =
        backend_elastic_saturation_restart_enabled &&
        backend_rtk_velocity_regime ==
            my_livo::backend::CorrectionRegime::kDegraded &&
        elastic_update.residual_after_m >=
            backend_elastic_saturation_restart_residual_m &&
        std::max(planar_gradient_ratio, vertical_gradient_ratio) >=
            backend_elastic_saturation_restart_gradient_ratio;
    const bool velocity_evidence_valid =
        velocity.baseline_available &&
        velocity.velocity_source ==
            my_livo::backend::RtkVelocitySource::kReceiverTwist &&
        velocity_error_after <=
            backend_lio_observability_restart_velocity_error_mps;
    const bool proactive_evidence =
        (geometry_failure || elastic_tracking_saturation) &&
        velocity_evidence_valid;
    backend_lio_observability_restart_evidence = proactive_evidence
        ? backend_lio_observability_restart_evidence + 1 : 0;
    if (backend_lio_observability_restart_evidence >=
        backend_lio_observability_restart_required_observations)
    {
      const bool saturation_trigger = elastic_tracking_saturation;
      const char *restart_reason = saturation_trigger
          ? "elastic_tracking_saturation" : "lio_observability";
      ++backend_frontend_restart_request_count;
      const bool suppressed =
          !backend_frontend_segment_restart_enabled ||
          backend_frontend_segment_id >=
              backend_frontend_segment_maximum_automatic_restarts;
      if (!suppressed)
        backend_pending_frontend_segment_restart =
            PendingFrontendSegmentRestart{
                keyframe->id(), query.observation->timestamp,
                restart_reason, 0.0, true,
                velocity.filtered_rtk_velocity,
                query.observation->position};
      if (backend_frontend_restart_supervisor_stream.is_open())
      {
        backend_frontend_restart_supervisor_stream << std::setprecision(17)
            << backend_frontend_restart_request_count << ','
            << keyframe->id() << ',' << query.observation->timestamp << ','
            << (suppressed ? "suppressed" : "scheduled")
            << ',' << restart_reason << ",0," << restart_reason << ','
            << (saturation_trigger
                    ? std::max(planar_gradient_ratio,
                               vertical_gradient_ratio)
                    : lio_observability_score)
            << ",1,"
            << elastic_update.residual_after_m << '\n';
        backend_frontend_restart_supervisor_stream.flush();
      }
      RCLCPP_ERROR(
          node_->get_logger(),
          "Causal elastic tracking failure at KF %lu: reason=%s, score="
          "%.3f, condition=%.5f/%.5f, global residual=%.3f m, velocity "
          "error=%.3f m/s; proactive continuous recovery %s.",
          static_cast<unsigned long>(keyframe->id()),
          restart_reason,
          saturation_trigger
              ? std::max(planar_gradient_ratio, vertical_gradient_ratio)
              : lio_observability_score,
          lio_observability.translation_condition_ratio,
          lio_observability.rotation_condition_ratio,
          elastic_update.residual_after_m, velocity_error_after,
          suppressed ? "suppressed" : "scheduled");
      backend_lio_observability_restart_evidence = 0;
    }
  }

  if (!select_global_observation)
  {
    ++backend_rtk_rejected_selection;
    write_decision("selector_spacing", query.observation,
                   Eigen::Vector3d::Zero(), 0.0, false);
    write_diagnostics("selector_spacing", false, backend_pose_before);
    return;
  }

  const auto &feasibility = rtk_update.feasibility;
  const auto &field = rtk_update.correction_field;
  const double velocity_evidence_age_sec =
      backend_have_latest_rtk_velocity_result
      ? query.observation->timestamp -
          backend_latest_rtk_velocity_result.timestamp
      : std::numeric_limits<double>::infinity();
  const bool velocity_evidence_fresh =
      velocity_evidence_age_sec >= -1.0e-9 &&
      velocity_evidence_age_sec <=
          backend_rtk_velocity_evidence_max_age_sec;
  const bool velocity_baseline_available =
      velocity_evidence_fresh &&
      backend_latest_rtk_velocity_result.baseline_available;
  const double post_velocity_error_mps = velocity_baseline_available
      ? (backend_latest_rtk_velocity_result.lio_velocity_after -
         backend_latest_rtk_velocity_result.filtered_rtk_velocity).norm()
      : 0.0;
  const auto relocalization =
      global_pose_layer->AddRelocalizationEvidence(
          keyframe->id(), velocity_baseline_available,
          velocity_evidence_fresh &&
              backend_latest_rtk_velocity_result.active,
          post_velocity_error_mps);
  if (relocalization.state_changed)
  {
    RCLCPP_WARN(
        node_->get_logger(),
        "Rigid 4DOF relocalization is ready at KF %lu: window=%zu, "
        "path ratio=%.3f, fit RMS=%.3f/%.3f m, yaw=%.3f deg. "
        "This phase is shadow-only and has not switched map segments.",
        static_cast<unsigned long>(keyframe->id()),
        relocalization.window_size, relocalization.path_scale_ratio,
        relocalization.planar_rms_m, relocalization.vertical_rms_m,
        relocalization.yaw_rad * 180.0 / M_PI);
  }
  if (relocalization.restart_state_changed)
  {
    bool automatic_restart_suppressed = false;
    RCLCPP_ERROR(
        node_->get_logger(),
        "Frontend local-segment restart required at KF %lu: %d "
        "consecutive %s results over %.1f m. The quarantined tail will "
        "not be joined by an invalid rigid transform.",
        static_cast<unsigned long>(keyframe->id()),
        relocalization.consecutive_structural_rejections,
        my_livo::backend::RigidRelocalizationDecisionToString(
            relocalization.restart_reason),
        relocalization.structural_rejection_span_m);
    if (backend_frontend_segment_restart_enabled)
    {
      if (backend_pending_frontend_segment_restart)
        throw std::logic_error(
            "A second frontend restart was requested before the first "
            "boundary was executed.");
      ++backend_frontend_restart_request_count;
      automatic_restart_suppressed =
          backend_frontend_segment_id >=
          backend_frontend_segment_maximum_automatic_restarts;
      if (!automatic_restart_suppressed)
        backend_pending_frontend_segment_restart =
            PendingFrontendSegmentRestart{
                keyframe->id(), query.observation->timestamp,
                my_livo::backend::RigidRelocalizationDecisionToString(
                    relocalization.restart_reason),
                relocalization.structural_rejection_span_m, false,
                V3D::Zero(), V3D::Zero()};
      if (backend_frontend_restart_supervisor_stream.is_open())
      {
        backend_frontend_restart_supervisor_stream << std::setprecision(17)
            << backend_frontend_restart_request_count << ','
            << keyframe->id() << ',' << query.observation->timestamp << ','
            << (automatic_restart_suppressed ? "suppressed" : "scheduled")
            << ','
            << my_livo::backend::RigidRelocalizationDecisionToString(
                   relocalization.restart_reason)
            << ',' << relocalization.structural_rejection_span_m << ','
            << my_livo::backend::RelocalizationGeometryFailureToString(
                   relocalization.geometry_failure)
            << ',' << relocalization.geometry_condition_ratio << ','
            << relocalization.similarity_scale << ','
            << relocalization.similarity_rms_m << '\n';
        backend_frontend_restart_supervisor_stream.flush();
      }
      if (automatic_restart_suppressed)
        RCLCPP_ERROR(
            node_->get_logger(),
            "Automatic LIO submap restart suppressed at KF %lu: the "
            "maximum of %zu restart(s) was already used without rigid "
            "recovery. Failure=%s, observable=%d, scale=%.4f, Sim3 "
            "RMS=%.3f m. Manual calibration/data diagnosis is required.",
            static_cast<unsigned long>(keyframe->id()),
            backend_frontend_segment_maximum_automatic_restarts,
            my_livo::backend::RelocalizationGeometryFailureToString(
                relocalization.geometry_failure),
            static_cast<int>(relocalization.full_rotation_observable),
            relocalization.similarity_scale,
            relocalization.similarity_rms_m);
    }
    if (pubBackendRelocalizationStatus)
    {
      visualization_msgs::MarkerArray markers;
      visualization_msgs::Marker marker;
      marker.header.frame_id = backend_map_frame_id;
      marker.header.stamp = stampFromSec(query.observation->timestamp);
      marker.ns = "frontend_restart_required";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      const auto &position = keyframe->T_global_body().translation;
      marker.pose.position.x = position.x();
      marker.pose.position.y = position.y();
      marker.pose.position.z = position.z() + 3.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 1.0;
      marker.color.r = 1.0F;
      marker.color.g = 0.15F;
      marker.color.b = 0.05F;
      marker.color.a = 1.0F;
      marker.text = automatic_restart_suppressed
          ? "LIO SYSTEMATIC GEOMETRY FAULT - RESTART SUPPRESSED"
          : "LIO LOCAL SEGMENT RESTART REQUIRED";
      markers.markers.push_back(marker);
      pubBackendRelocalizationStatus->publish(markers);
    }
    if (pubBackendFrontendRestartRequest)
    {
      diagnostic_msgs::msg::DiagnosticArray message;
      message.header.stamp = stampFromSec(query.observation->timestamp);
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.name = "backend/frontend_restart_required";
      status.hardware_id = "fast_livo_backend";
      status.message = automatic_restart_suppressed
          ? "automatic restart suppressed after failed recovery"
          : "persistent structural RTK/LIO disagreement";
      diagnostic_msgs::msg::KeyValue keyframe_value;
      keyframe_value.key = "keyframe_id";
      keyframe_value.value = std::to_string(keyframe->id());
      status.values.push_back(keyframe_value);
      diagnostic_msgs::msg::KeyValue reason_value;
      reason_value.key = "reason";
      reason_value.value =
          my_livo::backend::RigidRelocalizationDecisionToString(
              relocalization.restart_reason);
      status.values.push_back(reason_value);
      diagnostic_msgs::msg::KeyValue span_value;
      span_value.key = "evidence_span_m";
      span_value.value =
          std::to_string(relocalization.structural_rejection_span_m);
      status.values.push_back(span_value);
      diagnostic_msgs::msg::KeyValue geometry_value;
      geometry_value.key = "geometry_failure";
      geometry_value.value =
          my_livo::backend::RelocalizationGeometryFailureToString(
              relocalization.geometry_failure);
      status.values.push_back(geometry_value);
      message.status.push_back(status);
      pubBackendFrontendRestartRequest->publish(message);
    }
  }
  rtk_factor_selector->MarkSelected(keyframe->id(), *query.observation);
  ++backend_rtk_selected;
  write_decision("accepted_global_observation", query.observation,
                 innovation, chi2, false);
  write_diagnostics("accepted_global_observation", false,
                    keyframe->T_slam_body());
  RCLCPP_INFO(
      node_->get_logger(),
      "Global RTK observation KF %lu: innovation=%.3f m, chi2=%.3f, "
      "regime=%s, gradient=%.4f m/m, field=%s, knots=%zu, "
      "step=%.3f m/%.3f deg; local graph unchanged",
      static_cast<unsigned long>(keyframe->id()),
      innovation.norm(), chi2,
      my_livo::backend::CorrectionRegimeToString(feasibility.regime),
      feasibility.correction_gradient_m_per_m,
      my_livo::backend::CorrectionFieldDecisionToString(field.decision),
      field.knot_count,
      std::hypot(field.planar_step_m, field.vertical_step_m),
      field.yaw_step_deg);
  if (feasibility.state_changed &&
      feasibility.regime ==
          my_livo::backend::CorrectionRegime::kRelocalizationRequired)
  {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Elastic global correction disabled at KF %lu: sustained required "
        "correction gradient %.3f m/m exceeds the safe field limit. "
        "Healthy RTK remains valid; a new segment/relocalization is required.",
        static_cast<unsigned long>(keyframe->id()),
        feasibility.correction_gradient_m_per_m);
  }
}

LIVMapper::FrontendHistorySeedStatistics
LIVMapper::seedFrontendLocalMapFromHistory(
    std::uint64_t trigger_keyframe_id)
{
  FrontendHistorySeedStatistics statistics;
  if (!keyframe_manager || !voxelmap_manager)
    return statistics;
  const auto keyframes = keyframe_manager->keyframes();
  if (trigger_keyframe_id >= keyframes.size())
    throw std::logic_error(
        "Frontend history seed references an unknown keyframe.");

  std::vector<my_livo::backend::Keyframe::Ptr> selected;
  std::size_t cursor = static_cast<std::size_t>(trigger_keyframe_id);
  while (cursor > 0U &&
         selected.size() <
             backend_frontend_segment_maximum_history_seed_keyframes)
  {
    const auto &newer = keyframes.at(cursor);
    const auto &older = keyframes.at(cursor - 1U);
    statistics.path_length_m +=
        (newer->T_odom_body().translation -
         older->T_odom_body().translation).norm();
    selected.push_back(older);
    --cursor;
    if (statistics.path_length_m + 1.0e-9 >=
        backend_frontend_segment_history_seed_path_m)
      break;
  }
  std::reverse(selected.begin(), selected.end());

  std::size_t point_count = 0;
  for (const auto &keyframe : selected)
    point_count += keyframe->cloud_body()->size();
  std::vector<pointWithVar> history_points;
  history_points.reserve(point_count);
  const double point_variance =
      backend_frontend_segment_history_seed_point_sigma_m *
      backend_frontend_segment_history_seed_point_sigma_m;
  const M3D seed_covariance = M3D::Identity() * point_variance;
  for (const auto &keyframe : selected)
  {
    const auto T_odom_body = keyframe->T_odom_body();
    for (const auto &stored : keyframe->cloud_body()->points)
    {
      pointWithVar point;
      point.point_b = V3D(stored.x, stored.y, stored.z);
      point.point_w = T_odom_body * point.point_b;
      point.body_var = seed_covariance;
      point.var = seed_covariance;
      history_points.push_back(point);
    }
  }
  if (!history_points.empty())
    voxelmap_manager->UpdateVoxelMap(history_points);
  statistics.keyframes = selected.size();
  statistics.points = history_points.size();
  return statistics;
}

void LIVMapper::applyBackendRecoveryFrameVelocityTracking(double timestamp)
{
  if (!backend_rtk_recovery_frame_tracking_enabled ||
      !backend_have_latest_rtk_velocity_result ||
      !backend_latest_rtk_velocity_result.recovery_tracking ||
      !backend_latest_rtk_velocity_result.active)
  {
    backend_have_recovery_frame_tracking_time = false;
    return;
  }
  const double target_age =
      timestamp - backend_latest_rtk_velocity_result.timestamp;
  if (!std::isfinite(timestamp) || target_age < -1.0e-6 ||
      target_age > backend_rtk_recovery_frame_tracking_maximum_age_sec)
  {
    backend_have_recovery_frame_tracking_time = false;
    return;
  }
  if (!backend_have_recovery_frame_tracking_time)
  {
    backend_last_recovery_frame_tracking_time = timestamp;
    backend_have_recovery_frame_tracking_time = true;
    return;
  }
  const double interval =
      timestamp - backend_last_recovery_frame_tracking_time;
  if (interval <= 0.0) return;
  backend_last_recovery_frame_tracking_time = timestamp;

  const V3D before = _state.vel_end;
  V3D correction =
      backend_latest_rtk_velocity_result.tracking_target_velocity - before;
  const double planar_limit =
      backend_rtk_recovery_frame_tracking_planar_acceleration_mps2 *
      interval;
  const double planar_norm = correction.head<2>().norm();
  if (planar_norm > planar_limit && planar_norm > 1.0e-12)
    correction.head<2>() *= planar_limit / planar_norm;
  const double vertical_limit =
      backend_rtk_recovery_frame_tracking_vertical_acceleration_mps2 *
      interval;
  correction.z() = std::clamp(
      correction.z(), -vertical_limit, vertical_limit);
  _state.vel_end += correction;
  // Preserve the frontend covariance and all cross-correlations. This is a
  // bounded mean-state hold after scan matching, not an RTK measurement
  // update and not a pose correction.
  state_propagat.vel_end = _state.vel_end;
  voxelmap_manager->state_.vel_end = _state.vel_end;

  if (backend_rtk_recovery_frame_tracking_stream.is_open())
  {
    const V3D &target =
        backend_latest_rtk_velocity_result.tracking_target_velocity;
    backend_rtk_recovery_frame_tracking_stream << std::setprecision(17)
        << timestamp << ','
        << backend_latest_rtk_velocity_result.keyframe_id << ','
        << backend_latest_rtk_velocity_result.timestamp << ','
        << target_age << ',' << interval << ',' << target.x() << ','
        << target.y() << ',' << target.z() << ',' << before.x() << ','
        << before.y() << ',' << before.z() << ',' << _state.vel_end.x()
        << ',' << _state.vel_end.y() << ',' << _state.vel_end.z() << ','
        << correction.x() << ',' << correction.y() << ','
        << correction.z() << '\n';
    backend_rtk_recovery_frame_tracking_stream.flush();
  }
}

bool LIVMapper::executePendingFrontendSegmentRestart()
{
  if (!backend_pending_frontend_segment_restart) return false;
  if (!backend_frontend_segment_restart_enabled ||
      slam_mode_ != ONLY_LIO)
    throw std::logic_error(
        "A frontend segment restart escaped its pure-LIO safety gate.");
  if (!feats_down_body ||
      feats_down_body->size() <
          backend_frontend_segment_minimum_seed_points)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Waiting to rebuild the LIO local submap: current seed has %zu/%zu "
        "points.",
        feats_down_body ? feats_down_body->size() : 0U,
        backend_frontend_segment_minimum_seed_points);
    return false;
  }
  if (!global_pose_layer || !voxelmap_manager)
    throw std::logic_error(
        "Frontend segment restart requires global and voxel-map layers.");

  const PendingFrontendSegmentRestart request =
      *backend_pending_frontend_segment_restart;
  const StatesGroup state_before = _state;
  if (request.reset_velocity)
  {
    if (!request.target_velocity.allFinite() ||
        !request.rtk_anchor_position.allFinite())
      throw std::logic_error(
          "Frontend velocity-recovery request is not finite.");
    _state.vel_end = request.target_velocity;
    const double velocity_variance_floor =
        backend_rtk_velocity_covariance_floor_mps *
        backend_rtk_velocity_covariance_floor_mps;
    for (int axis = 0; axis < 3; ++axis)
      _state.cov(7 + axis, 7 + axis) = std::max(
          _state.cov(7 + axis, 7 + axis), velocity_variance_floor);
  }
  const double position_variance_floor =
      backend_frontend_segment_position_sigma_floor_m *
      backend_frontend_segment_position_sigma_floor_m;
  const double rotation_sigma_rad =
      backend_frontend_segment_rotation_sigma_floor_deg * M_PI / 180.0;
  const double rotation_variance_floor =
      rotation_sigma_rad * rotation_sigma_rad;
  for (int axis = 0; axis < 3; ++axis)
  {
    _state.cov(axis, axis) = std::max(
        _state.cov(axis, axis), rotation_variance_floor);
    _state.cov(axis + 3, axis + 3) = std::max(
        _state.cov(axis + 3, axis + 3), position_variance_floor);
  }
  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(
      _state.rot_end, _state.pos_end, feats_down_body,
      feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_body->size();

  const std::size_t deleted_voxels =
      voxelmap_manager->ResetLocalMap(_state);
  voxelmap_manager->BuildVoxelMap();
  const FrontendHistorySeedStatistics history_seed =
      seedFrontendLocalMapFromHistory(request.trigger_keyframe_id);
  const std::size_t total_seed_points =
      feats_down_body->size() + history_seed.points;
  lidar_map_inited = true;
  if (request.reset_velocity)
  {
    global_pose_layer->StartEmergencyGlobalSegment(
        request.trigger_keyframe_id, request.rtk_anchor_position);
    if (!rtk_velocity_guard)
      throw std::logic_error(
          "Velocity recovery lacks its RTK velocity guard.");
    if (request.reason == "velocity_divergence")
      rtk_velocity_guard->AcknowledgeEmergencyRestart(
          request.trigger_keyframe_id);
    else
      rtk_velocity_guard->BeginRecoveryTracking(
          request.trigger_keyframe_id);
    backend_global_map_dirty.store(true);
    backend_have_recovery_frame_tracking_time = false;
  }
  else
  {
    global_pose_layer->AcknowledgeFrontendRestart(
        request.trigger_keyframe_id);
  }

  {
    std::lock_guard<std::mutex> propagation_lock(mtx_buffer_imu_prop);
    imu_propagate = _state;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }
  pcl_w_wait_pub->clear();
  pcl_wait_pub->clear();

  ++backend_frontend_segment_id;
  const StatesGroup &state_after = _state;
  const double pose_delta_m =
      (state_after.pos_end - state_before.pos_end).norm();
  const double rotation_delta_deg =
      Eigen::Quaterniond(state_after.rot_end).angularDistance(
          Eigen::Quaterniond(state_before.rot_end)) * 180.0 / M_PI;
  const double velocity_delta_mps =
      (state_after.vel_end - state_before.vel_end).norm();
  const double bias_g_delta =
      (state_after.bias_g - state_before.bias_g).norm();
  const double bias_a_delta =
      (state_after.bias_a - state_before.bias_a).norm();
  const double gravity_delta =
      (state_after.gravity - state_before.gravity).norm();
  if (backend_frontend_segment_stream.is_open())
  {
    backend_frontend_segment_stream << std::setprecision(17)
        << backend_frontend_segment_id << ','
        << request.trigger_keyframe_id << ','
        << request.request_timestamp << ','
        << LidarMeasures.last_lio_update_time << ',' << request.reason << ','
        << request.evidence_span_m << ',' << total_seed_points << ','
        << feats_down_body->size() << ',' << history_seed.keyframes << ','
        << history_seed.path_length_m << ',' << history_seed.points << ','
        << deleted_voxels << ',' << voxelmap_manager->voxel_map_.size()
        << ',' << pose_delta_m << ',' << rotation_delta_deg << ','
        << velocity_delta_mps << ',' << bias_g_delta << ','
        << bias_a_delta << ',' << gravity_delta << ",1,"
        << (request.reason == "lio_observability"
                ? "observability"
                : (request.reason == "elastic_tracking_saturation"
                    ? "saturation"
                : (request.reset_velocity ? "velocity" : "structural"))
                )
        << ','
        << request.target_velocity.x() << ','
        << request.target_velocity.y() << ','
        << request.target_velocity.z() << ','
        << request.rtk_anchor_position.x() << ','
        << request.rtk_anchor_position.y() << ','
        << request.rtk_anchor_position.z() << '\n';
    backend_frontend_segment_stream.flush();
  }

  if (pubBackendRelocalizationStatus)
  {
    const auto global_poses = global_pose_layer->global_poses();
    if (request.trigger_keyframe_id >= global_poses.size())
      throw std::logic_error(
          "Frontend segment boundary lacks a global pose.");
    // This marker denotes the actual frontend/global segment boundary, not
    // the RTK evidence point stored for later recovery. Placing it at the RTK
    // anchor falsely suggested that the still-quarantined trajectory should
    // already pass through the sphere.
    const V3D position =
        global_poses[request.trigger_keyframe_id].translation;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker delete_request;
    delete_request.header.frame_id = backend_map_frame_id;
    delete_request.header.stamp =
        stampFromSec(LidarMeasures.last_lio_update_time);
    delete_request.ns = "frontend_restart_required";
    delete_request.id = 0;
    delete_request.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(delete_request);

    visualization_msgs::Marker boundary;
    boundary.header = delete_request.header;
    boundary.ns = "lio_segment_boundaries";
    boundary.id = static_cast<int>(backend_frontend_segment_id);
    boundary.type = visualization_msgs::Marker::SPHERE;
    boundary.action = visualization_msgs::Marker::ADD;
    boundary.pose.position.x = position.x();
    boundary.pose.position.y = position.y();
    boundary.pose.position.z = position.z();
    boundary.pose.orientation.w = 1.0;
    boundary.scale.x = 2.0;
    boundary.scale.y = 2.0;
    boundary.scale.z = 2.0;
    boundary.color.r = 1.0F;
    boundary.color.g = 1.0F;
    boundary.color.b = 1.0F;
    boundary.color.a = 0.35F;
    markers.markers.push_back(boundary);
    pubBackendRelocalizationStatus->publish(markers);
  }

  if (pubBackendFrontendRestartRequest)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp =
        stampFromSec(LidarMeasures.last_lio_update_time);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "backend/frontend_restart_required";
    status.hardware_id = "fast_livo_backend";
    status.message = request.reset_velocity
        ? "local voxel submap rebuilt; velocity stabilized; continuous "
          "global recovery started"
        : "local voxel submap rebuilt; recovery gate restarted";
    diagnostic_msgs::msg::KeyValue segment_value;
    segment_value.key = "segment_id";
    segment_value.value = std::to_string(backend_frontend_segment_id);
    status.values.push_back(segment_value);
    message.status.push_back(status);
    pubBackendFrontendRestartRequest->publish(message);
  }

  RCLCPP_WARN(
      node_->get_logger(),
      "Started LIO local segment %lu at KF %lu: %s, deleted %zu old voxels "
      "and seeded %zu current plus %zu historical points from %zu "
      "keyframes/%.1f m into %zu voxels; the segment remains "
      "quarantined pending a fresh rigid gate.",
      static_cast<unsigned long>(backend_frontend_segment_id),
      static_cast<unsigned long>(request.trigger_keyframe_id),
      request.reset_velocity
          ? "preserved pose/attitude/IMU biases, stabilized propagation "
            "velocity and opened a path-continuous recovery field"
          : "preserved pose/IMU state",
      deleted_voxels, feats_down_body->size(), history_seed.points,
      history_seed.keyframes, history_seed.path_length_m,
      voxelmap_manager->voxel_map_.size());
  backend_pending_frontend_segment_restart.reset();
  return true;
}

void LIVMapper::handleLoopVerificationDecisions(
    const std::vector<my_livo::backend::LoopVerificationDecision> &decisions,
    bool publish_outputs)
{
  if (decisions.empty()) return;
  visualization_msgs::MarkerArray markers;
  bool graph_changed = false;
  for (const auto &decision : decisions)
  {
    const auto &registration = decision.registration;
    RCLCPP_INFO(
        node_->get_logger(),
        "Loop verify %lu<-%lu: accepted=%d, reason=%s, neighbors=%zu, "
        "best_consistency=%.3f m/%.3f deg, graph_factor=%d",
        static_cast<unsigned long>(registration.candidate.candidate_id),
        static_cast<unsigned long>(registration.candidate.current_id),
        static_cast<int>(decision.accepted),
        my_livo::backend::LoopRejectReasonToString(decision.reject_reason),
        decision.consistent_neighbors,
        decision.best_neighbor_translation_error_m,
        decision.best_neighbor_rotation_error_deg,
        static_cast<int>(decision.accepted &&
                         backend_loop_factors_enabled));

    if (decision.accepted && backend_loop_factors_enabled)
    {
      try
      {
        const auto update = pose_graph_optimizer->AddLoopFactor(
            registration.candidate.candidate_id,
            registration.candidate.current_id,
            registration.T_candidate_current);
        RCLCPP_INFO(
            node_->get_logger(),
            "GTSAM loop %lu<-%lu: added=%d, residual %.4f m/%.4f deg -> "
            "%.4f m/%.4f deg, max pose correction %.4f m/%.4f deg, "
            "time=%.3f ms",
            static_cast<unsigned long>(registration.candidate.candidate_id),
            static_cast<unsigned long>(registration.candidate.current_id),
            static_cast<int>(update.added),
            update.residual_translation_before_m,
            update.residual_rotation_before_deg,
            update.residual_translation_after_m,
            update.residual_rotation_after_deg,
            update.maximum_pose_correction_m,
            update.maximum_pose_correction_deg,
            update.optimization_time_ms);
        if (update.added)
        {
          graph_changed = true;
          if (optimized_global_map &&
              optimized_global_map->CorrectionRequiresFullRebuild(
                  update.maximum_pose_correction_m,
                  update.maximum_pose_correction_deg))
            backend_global_map_dirty.store(true);
        }
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to add verified loop %lu<-%lu: %s",
            static_cast<unsigned long>(registration.candidate.candidate_id),
            static_cast<unsigned long>(registration.candidate.current_id),
            error.what());
      }
    }

    const my_livo::backend::Pose3d registered_current =
        registration.T_map_candidate_snapshot *
        registration.T_candidate_current;
    visualization_msgs::Marker marker;
    marker.header.frame_id = backend_frontend_frame_id;
    marker.header.stamp =
        stampFromSec(registration.candidate.current_timestamp);
    marker.ns = decision.accepted ? "backend_verified_loops_accepted"
                                  : "backend_verified_loops_rejected";
    marker.id = static_cast<int>(backend_loop_verification_marker_id++);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = decision.accepted ? 0.32 : 0.16;
    marker.color.r = decision.accepted ? 0.05F : 1.0F;
    marker.color.g = decision.accepted ? 1.0F : 0.10F;
    marker.color.b = decision.accepted ? 0.20F : 0.05F;
    marker.color.a = decision.accepted ? 1.0F : 0.65F;
    geometry_msgs::msg::Point historical;
    historical.x = registration.T_map_candidate_snapshot.translation.x();
    historical.y = registration.T_map_candidate_snapshot.translation.y();
    historical.z = registration.T_map_candidate_snapshot.translation.z();
    geometry_msgs::msg::Point current;
    current.x = registered_current.translation.x();
    current.y = registered_current.translation.y();
    current.z = registered_current.translation.z();
    marker.points.push_back(historical);
    marker.points.push_back(current);
    markers.markers.push_back(marker);
  }
  if (graph_changed && global_pose_layer && keyframe_manager &&
      pose_graph_optimizer)
  {
    global_pose_layer->SynchronizeLocalPoses(
        keyframe_manager->keyframes(),
        pose_graph_optimizer->optimized_poses());
  }
  if (graph_changed && publish_outputs)
    publishBackendOptimizedProducts(true, "loop");
  if (publish_outputs && pubBackendVerifiedLoops)
    pubBackendVerifiedLoops->publish(markers);
}

void LIVMapper::readParameters()
{
  Ros2ParameterReader nh(node_);
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<string>("common/ins_odom_topic", ins_odom_topic, "/imu_data/odometry");
  nh.param<string>("common/ins_status_topic", ins_status_topic,
                   "/imu_data/ins_status");
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
  }

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);

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
  nh.param<double>("imu/maximum_time_gap", imu_max_time_gap, 0.2);
  nh.param<double>("imu/maximum_recoverable_gap",
                   imu_max_recoverable_gap, 2.0);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);
  if (!std::isfinite(imu_max_time_gap) || imu_max_time_gap <= 0.0)
    throw std::runtime_error("imu.maximum_time_gap must be finite and positive.");
  if (!std::isfinite(imu_max_recoverable_gap) ||
      imu_max_recoverable_gap <= imu_max_time_gap)
    throw std::runtime_error(
        "imu.maximum_recoverable_gap must be finite and greater than "
        "imu.maximum_time_gap.");

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

  nh.param<bool>("backend/keyframes/enabled",
                 backend_keyframes_enabled, false);
  nh.param<double>("backend/keyframes/translation_threshold_m",
                   backend_keyframe_options.translation_threshold_m, 1.0);
  nh.param<double>("backend/keyframes/rotation_threshold_deg",
                   backend_keyframe_options.rotation_threshold_deg, 10.0);
  nh.param<double>("backend/keyframes/maximum_interval_sec",
                   backend_keyframe_options.maximum_interval_sec, 2.0);
  nh.param<double>("backend/keyframes/minimum_interval_sec",
                   backend_keyframe_options.minimum_interval_sec, 0.2);
  nh.param<double>("backend/keyframes/cloud_leaf_size_m",
                   backend_keyframe_options.cloud_leaf_size_m, 0.5);
  nh.param<string>("backend/keyframes/csv_path",
                   backend_keyframe_options.csv_path,
                   std::string(ROOT_DIR) + "Log/backend/keyframes.csv");
  nh.param<bool>("backend/keyframes/publish_latest_cloud",
                 backend_publish_keyframe_cloud, true);
  nh.param<string>("backend/frontend_frame_id",
                   backend_frontend_frame_id, "mine");
  nh.param<string>("backend/map_frame_id", backend_map_frame_id, "map");
  nh.param<string>("backend/body_frame_id", backend_body_frame_id, "body");
  int backend_path_interval = 10;
  nh.param<int>("backend/path_publish_keyframe_interval",
                backend_path_interval, 10);
  if (backend_path_interval <= 0)
    throw std::runtime_error(
        "backend.path_publish_keyframe_interval must be positive.");
  backend_path_publish_interval =
      static_cast<std::size_t>(backend_path_interval);
  if (backend_frontend_frame_id.empty())
    throw std::runtime_error("backend.frontend_frame_id must not be empty.");
  if (backend_map_frame_id.empty() || backend_body_frame_id.empty())
    throw std::runtime_error(
        "backend map/body frame IDs must not be empty.");

  nh.param<bool>("backend/pose_graph/enabled",
                 backend_pose_graph_enabled, false);
  nh.param<double>("backend/pose_graph/odometry_translation_sigma_m",
                   backend_pose_graph_options.odometry_translation_sigma_m,
                   0.10);
  nh.param<double>("backend/pose_graph/odometry_rotation_sigma_deg",
                   backend_pose_graph_options.odometry_rotation_sigma_deg,
                   0.50);
  nh.param<double>("backend/pose_graph/prior_translation_sigma_m",
                   backend_pose_graph_options.prior_translation_sigma_m,
                   1.0e-6);
  nh.param<double>("backend/pose_graph/prior_rotation_sigma_deg",
                   backend_pose_graph_options.prior_rotation_sigma_deg,
                   1.0e-4);
  nh.param<double>("backend/pose_graph/relinearize_threshold",
                   backend_pose_graph_options.relinearize_threshold, 0.01);
  nh.param<int>("backend/pose_graph/relinearize_skip",
                backend_pose_graph_options.relinearize_skip, 1);
  nh.param<double>("backend/pose_graph/wildfire_threshold",
                   backend_pose_graph_options.wildfire_threshold, 0.001);
  nh.param<int>("backend/pose_graph/additional_update_steps",
                backend_pose_graph_options.additional_update_steps, 1);
  nh.param<string>("backend/pose_graph/csv_path",
                   backend_pose_graph_options.csv_path,
                   std::string(ROOT_DIR) + "Log/backend/pose_graph.csv");
  if (backend_pose_graph_enabled && !backend_keyframes_enabled)
    throw std::runtime_error(
        "backend.pose_graph requires backend.keyframes.enabled=true.");

  nh.param<bool>("backend/loop_detection/enabled",
                 backend_loop_detection_enabled, false);
  int loop_check_interval = 20;
  int loop_minimum_id_separation = 50;
  int loop_candidate_id_separation = 20;
  int loop_maximum_candidates = 3;
  nh.param<int>("backend/loop_detection/check_interval_keyframes",
                loop_check_interval, 20);
  nh.param<int>("backend/loop_detection/minimum_id_separation_keyframes",
                loop_minimum_id_separation, 50);
  nh.param<double>("backend/loop_detection/minimum_time_separation_sec",
                   backend_loop_detection_options.minimum_time_separation_sec,
                   30.0);
  nh.param<double>("backend/loop_detection/maximum_planar_distance_m",
                   backend_loop_detection_options.maximum_planar_distance_m,
                   20.0);
  nh.param<double>("backend/loop_detection/maximum_height_difference_m",
                   backend_loop_detection_options.maximum_height_difference_m,
                   5.0);
  nh.param<int>(
      "backend/loop_detection/minimum_candidate_id_separation_keyframes",
      loop_candidate_id_separation, 20);
  nh.param<int>("backend/loop_detection/maximum_candidates_per_keyframe",
                loop_maximum_candidates, 3);
  if (loop_check_interval <= 0 || loop_minimum_id_separation <= 0 ||
      loop_candidate_id_separation <= 0 || loop_maximum_candidates <= 0)
    throw std::runtime_error(
        "backend.loop_detection integer parameters must be positive.");
  backend_loop_detection_options.check_interval_keyframes =
      static_cast<std::uint64_t>(loop_check_interval);
  backend_loop_detection_options.minimum_id_separation_keyframes =
      static_cast<std::uint64_t>(loop_minimum_id_separation);
  backend_loop_detection_options.minimum_candidate_id_separation_keyframes =
      static_cast<std::uint64_t>(loop_candidate_id_separation);
  backend_loop_detection_options.maximum_candidates_per_keyframe =
      static_cast<std::size_t>(loop_maximum_candidates);
  nh.param<string>("backend/loop_detection/detection_csv_path",
                   backend_loop_detection_options.detection_csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/loop_detection.csv");
  nh.param<string>("backend/loop_detection/candidate_csv_path",
                   backend_loop_detection_options.candidate_csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/loop_candidates.csv");
  if (backend_loop_detection_enabled && !backend_pose_graph_enabled)
    throw std::runtime_error(
        "backend.loop_detection requires backend.pose_graph.enabled=true.");

  nh.param<bool>("backend/loop_registration/enabled",
                 backend_loop_registration_enabled, false);
  nh.param<int>(
      "backend/loop_registration/target_submap_half_width_keyframes",
      backend_loop_registration_options.target_submap_half_width_keyframes,
      40);
  nh.param<int>("backend/loop_registration/target_submap_stride_keyframes",
                backend_loop_registration_options
                    .target_submap_stride_keyframes,
                4);
  nh.param<vector<double>>(
      "backend/loop_registration/resolutions_m",
      backend_loop_registration_options.resolutions_m,
      vector<double>{10.0, 5.0, 2.0, 1.0});
  nh.param<double>("backend/loop_registration/voxel_leaf_size_ratio",
                   backend_loop_registration_options.voxel_leaf_size_ratio,
                   0.25);
  nh.param<double>("backend/loop_registration/minimum_voxel_leaf_size_m",
                   backend_loop_registration_options
                       .minimum_voxel_leaf_size_m,
                   0.50);
  nh.param<double>("backend/loop_registration/transformation_epsilon",
                   backend_loop_registration_options
                       .transformation_epsilon,
                   0.05);
  nh.param<double>("backend/loop_registration/step_size",
                   backend_loop_registration_options.step_size, 0.70);
  nh.param<int>("backend/loop_registration/maximum_iterations",
                backend_loop_registration_options.maximum_iterations, 40);
  nh.param<double>(
      "backend/loop_registration/overlap_max_correspondence_distance_m",
      backend_loop_registration_options
          .overlap_max_correspondence_distance_m,
      1.0);
  int loop_registration_minimum_source_points = 100;
  int loop_registration_minimum_target_points = 300;
  int loop_registration_maximum_queue_size = 64;
  nh.param<int>("backend/loop_registration/minimum_source_points",
                loop_registration_minimum_source_points, 100);
  nh.param<int>("backend/loop_registration/minimum_target_points",
                loop_registration_minimum_target_points, 300);
  nh.param<int>("backend/loop_registration/maximum_queue_size",
                loop_registration_maximum_queue_size, 64);
  if (loop_registration_minimum_source_points <= 0 ||
      loop_registration_minimum_target_points <= 0 ||
      loop_registration_maximum_queue_size <= 0)
    throw std::runtime_error(
        "backend.loop_registration point and queue limits must be "
        "positive.");
  backend_loop_registration_options.minimum_source_points =
      static_cast<std::size_t>(loop_registration_minimum_source_points);
  backend_loop_registration_options.minimum_target_points =
      static_cast<std::size_t>(loop_registration_minimum_target_points);
  backend_loop_registration_options.maximum_queue_size =
      static_cast<std::size_t>(loop_registration_maximum_queue_size);
  nh.param<string>("backend/loop_registration/registration_csv_path",
                   backend_loop_registration_options.registration_csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/loop_registrations.csv");
  nh.param<string>("backend/loop_registration/level_csv_path",
                   backend_loop_registration_options.level_csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/loop_registration_levels.csv");
  if (backend_loop_registration_enabled &&
      !backend_loop_detection_enabled)
    throw std::runtime_error(
        "backend.loop_registration requires "
        "backend.loop_detection.enabled=true.");

  nh.param<bool>("backend/loop_verification/enabled",
                 backend_loop_verification_enabled, false);
  nh.param<bool>("backend/loop_verification/require_final_convergence",
                 backend_loop_verification_options
                     .require_final_convergence,
                 true);
  int loop_minimum_converged_levels = 4;
  int loop_neighbor_current_window = 40;
  int loop_neighbor_candidate_window = 40;
  int loop_minimum_consistent_neighbors = 1;
  nh.param<int>("backend/loop_verification/minimum_converged_levels",
                loop_minimum_converged_levels, 4);
  nh.param<double>(
      "backend/loop_verification/minimum_transformation_probability",
      backend_loop_verification_options
          .minimum_transformation_probability,
      0.35);
  nh.param<double>("backend/loop_verification/maximum_fitness_score_m2",
                   backend_loop_verification_options
                       .maximum_fitness_score_m2,
                   0.09);
  nh.param<double>("backend/loop_verification/minimum_overlap",
                   backend_loop_verification_options.minimum_overlap, 0.90);
  nh.param<double>("backend/loop_verification/maximum_overlap_rmse_m",
                   backend_loop_verification_options
                       .maximum_overlap_rmse_m,
                   0.32);
  nh.param<double>(
      "backend/loop_verification/maximum_translation_correction_m",
      backend_loop_verification_options
          .maximum_translation_correction_m,
      5.0);
  nh.param<double>(
      "backend/loop_verification/maximum_rotation_correction_deg",
      backend_loop_verification_options.maximum_rotation_correction_deg,
      10.0);
  nh.param<int>("backend/loop_verification/neighbor_current_id_window",
                loop_neighbor_current_window, 40);
  nh.param<int>("backend/loop_verification/neighbor_candidate_id_window",
                loop_neighbor_candidate_window, 40);
  nh.param<double>(
      "backend/loop_verification/maximum_neighbor_translation_error_m",
      backend_loop_verification_options
          .maximum_neighbor_translation_error_m,
      0.50);
  nh.param<double>(
      "backend/loop_verification/maximum_neighbor_rotation_error_deg",
      backend_loop_verification_options
          .maximum_neighbor_rotation_error_deg,
      2.0);
  nh.param<int>("backend/loop_verification/minimum_consistent_neighbors",
                loop_minimum_consistent_neighbors, 1);
  if (loop_minimum_converged_levels <= 0 ||
      loop_neighbor_current_window <= 0 ||
      loop_neighbor_candidate_window <= 0 ||
      loop_minimum_consistent_neighbors <= 0)
    throw std::runtime_error(
        "backend.loop_verification integer parameters must be positive.");
  backend_loop_verification_options.minimum_converged_levels =
      static_cast<std::size_t>(loop_minimum_converged_levels);
  backend_loop_verification_options.neighbor_current_id_window =
      static_cast<std::uint64_t>(loop_neighbor_current_window);
  backend_loop_verification_options.neighbor_candidate_id_window =
      static_cast<std::uint64_t>(loop_neighbor_candidate_window);
  backend_loop_verification_options.minimum_consistent_neighbors =
      static_cast<std::size_t>(loop_minimum_consistent_neighbors);
  nh.param<string>("backend/loop_verification/csv_path",
                   backend_loop_verification_options.csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/loop_verification.csv");
  if (backend_loop_verification_enabled &&
      !backend_loop_registration_enabled)
    throw std::runtime_error(
        "backend.loop_verification requires "
        "backend.loop_registration.enabled=true.");

  nh.param<bool>("backend/loop_pose_graph/enabled",
                 backend_loop_factors_enabled, false);
  nh.param<double>("backend/loop_pose_graph/translation_sigma_m",
                   backend_pose_graph_options.loop_translation_sigma_m,
                   0.20);
  nh.param<double>("backend/loop_pose_graph/rotation_sigma_deg",
                   backend_pose_graph_options.loop_rotation_sigma_deg,
                   3.0);
  nh.param<string>("backend/loop_pose_graph/robust_kernel",
                   backend_pose_graph_options.loop_robust_kernel,
                   "cauchy");
  nh.param<double>("backend/loop_pose_graph/robust_delta",
                   backend_pose_graph_options.loop_robust_delta, 2.0);
  nh.param<int>("backend/loop_pose_graph/additional_update_steps",
                backend_pose_graph_options.loop_additional_update_steps, 2);
  nh.param<string>("backend/loop_pose_graph/csv_path",
                   backend_pose_graph_options.loop_csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/loop_factors.csv");
  nh.param<string>("backend/optimized_trajectory_csv_path",
                   backend_pose_graph_options.optimized_trajectory_csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/optimized_trajectory.csv");
  if (backend_loop_factors_enabled && !backend_loop_verification_enabled)
    throw std::runtime_error(
        "backend.loop_pose_graph requires "
        "backend.loop_verification.enabled=true.");

  nh.param<bool>("backend/global_map/enabled",
                 backend_global_map_enabled, false);
  nh.param<double>("backend/global_map/voxel_leaf_size_m",
                   backend_global_map_options.voxel_leaf_size_m, 0.75);
  nh.param<double>("backend/global_map/tile_size_m",
                   backend_global_map_options.tile_size_m, 60.0);
  int global_map_coalesce_ms = 50;
  nh.param<int>("backend/global_map/request_coalesce_ms",
                global_map_coalesce_ms, 50);
  if (global_map_coalesce_ms < 0)
    throw std::runtime_error(
        "backend.global_map.request_coalesce_ms must be non-negative.");
  backend_global_map_request_coalesce_ms =
      static_cast<std::size_t>(global_map_coalesce_ms);
  int global_map_interval = 20;
  nh.param<int>("backend/global_map/periodic_keyframe_interval",
                global_map_interval, 20);
  if (global_map_interval <= 0)
    throw std::runtime_error(
        "backend.global_map.periodic_keyframe_interval must be positive.");
  backend_global_map_options.periodic_keyframe_interval =
      static_cast<std::size_t>(global_map_interval);
  int minimum_graph_map_interval = 10;
  nh.param<int>("backend/global_map/minimum_graph_rebuild_keyframe_interval",
                minimum_graph_map_interval, 10);
  if (minimum_graph_map_interval <= 0)
    throw std::runtime_error(
        "backend.global_map.minimum_graph_rebuild_keyframe_interval must "
        "be positive.");
  backend_global_map_options.minimum_graph_rebuild_keyframe_interval =
      static_cast<std::size_t>(minimum_graph_map_interval);
  nh.param<double>("backend/global_map/incremental_max_pose_change_m",
                   backend_global_map_options.incremental_max_pose_change_m,
                   0.10);
  nh.param<double>("backend/global_map/incremental_max_pose_change_deg",
                   backend_global_map_options.incremental_max_pose_change_deg,
                   0.25);
  nh.param<bool>("backend/global_map/preserve_rigid_local_submaps",
                 backend_global_map_options.preserve_rigid_local_submaps,
                 true);
  nh.param<double>("backend/global_map/rigid_submap_length_m",
                   backend_global_map_options.rigid_submap_length_m, 10.0);
  nh.param<double>(
      "backend/global_map/rigid_submap_boundary_position_change_m",
      backend_global_map_options
          .rigid_submap_boundary_position_change_m,
      0.50);
  nh.param<double>(
      "backend/global_map/rigid_submap_boundary_angle_change_deg",
      backend_global_map_options.rigid_submap_boundary_angle_change_deg,
      0.50);
  nh.param<bool>("backend/global_map/spatial_deformation_enabled",
                 backend_global_map_options.spatial_deformation_enabled,
                 false);
  int spatial_deformation_neighbors = 4;
  nh.param<int>("backend/global_map/spatial_deformation_neighbors",
                spatial_deformation_neighbors, 4);
  if (spatial_deformation_neighbors <= 0)
    throw std::runtime_error(
        "backend.global_map.spatial_deformation_neighbors must be positive.");
  backend_global_map_options.spatial_deformation_neighbors =
      static_cast<std::size_t>(spatial_deformation_neighbors);
  nh.param<double>("backend/global_map/spatial_deformation_sigma_m",
                   backend_global_map_options.spatial_deformation_sigma_m,
                   5.0);
  nh.param<string>("backend/global_map/csv_path",
                   backend_global_map_options.csv_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/global_map_updates.csv");
  nh.param<string>("backend/global_map/pcd_path",
                   backend_global_map_options.pcd_path,
                   std::string(ROOT_DIR) +
                       "Log/backend/optimized_global_map.pcd");
  if (backend_global_map_enabled && !backend_pose_graph_enabled)
    throw std::runtime_error(
        "backend.global_map requires backend.pose_graph.enabled=true.");

  nh.param<bool>("backend/rtk/input_enabled",
                 backend_rtk_input_enabled, false);
  nh.param<bool>("backend/rtk/fusion_enabled",
                 backend_rtk_fusion_enabled, false);
  nh.param<int>("backend/rtk/required_ins_pos_mode",
                backend_rtk_buffer_options.required_ins_pos_mode, 4);
  nh.param<double>("backend/rtk/maximum_status_age_sec",
                   backend_rtk_buffer_options.maximum_status_age_sec, 0.03);
  nh.param<double>("backend/rtk/maximum_interpolation_gap_sec",
                   backend_rtk_buffer_options.maximum_interpolation_gap_sec,
                   0.05);
  nh.param<double>("backend/rtk/maximum_endpoint_distance_sec",
                   backend_rtk_buffer_options.maximum_endpoint_distance_sec,
                   0.03);
  nh.param<double>("backend/rtk/buffer_retention_sec",
                   backend_rtk_buffer_options.retention_sec, 20.0);
  nh.param<double>("backend/rtk/configured_sigma_xy_m",
                   backend_rtk_buffer_options.configured_sigma_xy_m, 0.30);
  nh.param<double>("backend/rtk/configured_sigma_z_m",
                   backend_rtk_buffer_options.configured_sigma_z_m, 0.50);
  nh.param<double>("backend/rtk/minimum_sigma_xy_m",
                   backend_rtk_buffer_options.minimum_sigma_xy_m, 0.10);
  nh.param<double>("backend/rtk/minimum_sigma_z_m",
                   backend_rtk_buffer_options.minimum_sigma_z_m, 0.20);
  nh.param<double>("backend/rtk/configured_velocity_sigma_xy_mps",
                   backend_rtk_buffer_options
                       .configured_velocity_sigma_xy_mps,
                   0.25);
  nh.param<double>("backend/rtk/configured_velocity_sigma_z_mps",
                   backend_rtk_buffer_options
                       .configured_velocity_sigma_z_mps,
                   0.40);
  nh.param<double>("backend/rtk/minimum_velocity_sigma_xy_mps",
                   backend_rtk_buffer_options
                       .minimum_velocity_sigma_xy_mps,
                   0.05);
  nh.param<double>("backend/rtk/minimum_velocity_sigma_z_mps",
                   backend_rtk_buffer_options
                       .minimum_velocity_sigma_z_mps,
                   0.10);
  nh.param<double>("backend/rtk/factor_minimum_time_interval_sec",
                   backend_rtk_selector_options.minimum_time_interval_sec,
                   2.0);
  nh.param<double>("backend/rtk/factor_maximum_time_interval_sec",
                   backend_rtk_selector_options.maximum_time_interval_sec,
                   10.0);
  nh.param<double>("backend/rtk/factor_minimum_translation_m",
                   backend_rtk_selector_options.minimum_translation_m, 5.0);
  nh.param<double>("backend/rtk/innovation_prediction_sigma_m",
                   backend_rtk_innovation_prediction_sigma_m, 2.0);
  nh.param<string>("backend/rtk/status_csv_path",
                   backend_rtk_buffer_options.status_csv_path,
                   std::string(ROOT_DIR) + "Log/backend/rtk_status.csv");
  nh.param<string>("backend/rtk/solution_csv_path",
                   backend_rtk_buffer_options.solution_csv_path,
                   std::string(ROOT_DIR) + "Log/backend/rtk_solutions.csv");
  nh.param<string>("backend/rtk/query_csv_path",
                   backend_rtk_buffer_options.query_csv_path,
                   std::string(ROOT_DIR) + "Log/backend/rtk_queries.csv");
  nh.param<string>("backend/rtk/factor_csv_path",
                   backend_pose_graph_options.rtk_csv_path,
                   std::string(ROOT_DIR) + "Log/backend/rtk_factors.csv");
  nh.param<string>("backend/rtk/decision_csv_path",
                   backend_rtk_decision_csv_path,
                   std::string(ROOT_DIR) + "Log/backend/rtk_decisions.csv");
  nh.param<string>(
      "backend/rtk/diagnostics_csv_path",
      backend_rtk_diagnostics_options.keyframe_csv_path,
      std::string(ROOT_DIR) + "Log/backend/rtk_diagnostics.csv");
  nh.param<string>(
      "backend/rtk/initial_alignment_csv_path",
      backend_rtk_diagnostics_options.initial_alignment_csv_path,
      std::string(ROOT_DIR) + "Log/backend/rtk_initial_alignment.csv");
  nh.param<string>(
      "backend/global_pose/trajectory_csv_path",
      backend_global_pose_options.trajectory_csv_path,
      std::string(ROOT_DIR) + "Log/backend/global_trajectory.csv");
  nh.param<string>(
      "backend/global_pose/observation_csv_path",
      backend_global_pose_options.observation_csv_path,
      std::string(ROOT_DIR) + "Log/backend/global_rtk_observations.csv");
  nh.param<double>(
      "backend/global_pose/correction_monitor/minimum_baseline_m",
      backend_global_pose_options.correction_monitor.minimum_baseline_m,
      20.0);
  nh.param<double>(
      "backend/global_pose/correction_monitor/elastic_gradient_limit_m_per_m",
      backend_global_pose_options.correction_monitor
          .elastic_gradient_limit_m_per_m,
      0.02);
  nh.param<double>(
      "backend/global_pose/correction_monitor/"
      "relocalization_gradient_m_per_m",
      backend_global_pose_options.correction_monitor
          .relocalization_gradient_m_per_m,
      0.15);
  nh.param<int>(
      "backend/global_pose/correction_monitor/"
      "relocalization_consecutive_observations",
      backend_global_pose_options.correction_monitor
          .relocalization_consecutive_observations,
      2);
  nh.param<string>(
      "backend/global_pose/correction_monitor/csv_path",
      backend_global_pose_options.correction_monitor.csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/correction_feasibility.csv");
  nh.param<bool>(
      "backend/global_pose/correction_field/enabled",
      backend_global_pose_options.correction_field.enabled, true);
  nh.param<double>(
      "backend/global_pose/correction_field/knot_spacing_m",
      backend_global_pose_options.correction_field.knot_spacing_m, 15.0);
  nh.param<double>(
      "backend/global_pose/correction_field/spatial_low_pass_length_m",
      backend_global_pose_options.correction_field
          .spatial_low_pass_length_m,
      40.0);
  nh.param<double>(
      "backend/global_pose/correction_field/elastic_soft_radius_m",
      backend_global_pose_options.correction_field.elastic_soft_radius_m,
      0.10);
  nh.param<double>(
      "backend/global_pose/correction_field/elastic_full_radius_m",
      backend_global_pose_options.correction_field.elastic_full_radius_m,
      0.28);
  nh.param<double>(
      "backend/global_pose/correction_field/elastic_minimum_stiffness",
      backend_global_pose_options.correction_field
          .elastic_minimum_stiffness,
      0.05);
  nh.param<double>(
      "backend/global_pose/correction_field/elastic_maximum_stiffness",
      backend_global_pose_options.correction_field
          .elastic_maximum_stiffness,
      1.0);
  nh.param<double>(
      "backend/global_pose/correction_field/"
      "maximum_planar_gradient_m_per_m",
      backend_global_pose_options.correction_field
          .maximum_planar_gradient_m_per_m,
      0.005);
  nh.param<double>(
      "backend/global_pose/correction_field/"
      "maximum_vertical_gradient_m_per_m",
      backend_global_pose_options.correction_field
          .maximum_vertical_gradient_m_per_m,
      0.003);
  nh.param<double>(
      "backend/global_pose/correction_field/"
      "maximum_yaw_gradient_deg_per_m",
      backend_global_pose_options.correction_field
          .maximum_yaw_gradient_deg_per_m,
      0.01);
  nh.param<double>(
      "backend/global_pose/correction_field/maximum_planar_update_m",
      backend_global_pose_options.correction_field.maximum_planar_update_m,
      0.05);
  nh.param<double>(
      "backend/global_pose/correction_field/maximum_vertical_update_m",
      backend_global_pose_options.correction_field.maximum_vertical_update_m,
      0.03);
  nh.param<double>(
      "backend/global_pose/correction_field/maximum_yaw_update_deg",
      backend_global_pose_options.correction_field.maximum_yaw_update_deg,
      0.15);
  nh.param<string>(
      "backend/global_pose/correction_field/csv_path",
      backend_global_pose_options.correction_field.csv_path,
      std::string(ROOT_DIR) + "Log/backend/correction_field.csv");
  auto &regularized_field = backend_global_pose_options.regularized_field;
  nh.param<double>(
      "backend/global_pose/regularized_field/position_observation_weight",
      regularized_field.position_observation_weight, 1.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/yaw_observation_weight",
      regularized_field.orientation_yaw_observation_weight, 10.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/position_second_difference_lambda",
      regularized_field.position_second_difference_lambda, 10.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/yaw_second_difference_lambda",
      regularized_field.orientation_yaw_second_difference_lambda, 10.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/yaw_window_m",
      regularized_field.orientation_yaw_window_m, 0.1);
  nh.param<double>(
      "backend/global_pose/regularized_field/minimum_yaw_confidence",
      regularized_field.minimum_orientation_yaw_confidence, 1.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/elastic_soft_radius_m",
      regularized_field.elastic_soft_radius_m, 0.15);
  nh.param<double>(
      "backend/global_pose/regularized_field/elastic_full_radius_m",
      regularized_field.elastic_full_radius_m, 0.50);
  nh.param<double>(
      "backend/global_pose/regularized_field/vertical_elastic_soft_radius_m",
      regularized_field.vertical_elastic_soft_radius_m, 0.15);
  nh.param<double>(
      "backend/global_pose/regularized_field/vertical_elastic_full_radius_m",
      regularized_field.vertical_elastic_full_radius_m, 0.60);
  nh.param<double>(
      "backend/global_pose/regularized_field/yaw_elastic_soft_radius_deg",
      regularized_field.yaw_elastic_soft_radius_deg, 0.25);
  nh.param<double>(
      "backend/global_pose/regularized_field/yaw_elastic_full_radius_deg",
      regularized_field.yaw_elastic_full_radius_deg, 1.00);
  nh.param<double>(
      "backend/global_pose/regularized_field/elastic_minimum_stiffness",
      regularized_field.elastic_minimum_stiffness, 0.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/elastic_maximum_stiffness",
      regularized_field.elastic_maximum_stiffness, 1.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/alignment_window_length_m",
      regularized_field.alignment_window_length_m, 30.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/minimum_alignment_path_length_m",
      regularized_field.minimum_alignment_path_length_m, 10.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/alignment_huber_delta_m",
      regularized_field.alignment_huber_delta_m, 0.15);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_alignment_planar_rms_m",
      regularized_field.maximum_alignment_planar_rms_m, 0.30);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_planar_gradient_m_per_m",
      regularized_field.maximum_planar_gradient_m_per_m, 0.05);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_vertical_gradient_m_per_m",
      regularized_field.maximum_vertical_gradient_m_per_m, 0.08);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_yaw_gradient_deg_per_m",
      regularized_field.maximum_yaw_gradient_deg_per_m, 0.10);
  nh.param<bool>(
      "backend/global_pose/regularized_field/adaptive_gradient_enabled",
      regularized_field.adaptive_gradient_enabled, true);
  nh.param<double>(
      "backend/global_pose/regularized_field/"
      "adaptive_position_gradient_full_distance_m",
      regularized_field.adaptive_position_gradient_full_distance_m, 1.50);
  nh.param<double>(
      "backend/global_pose/regularized_field/"
      "adaptive_yaw_gradient_full_distance_deg",
      regularized_field.adaptive_yaw_gradient_full_distance_deg, 3.00);
  nh.param<double>(
      "backend/global_pose/regularized_field/"
      "adaptive_position_gradient_maximum_gain",
      regularized_field.adaptive_position_gradient_maximum_gain, 1.5);
  nh.param<double>(
      "backend/global_pose/regularized_field/"
      "adaptive_yaw_gradient_maximum_gain",
      regularized_field.adaptive_yaw_gradient_maximum_gain, 2.0);
  nh.param<int>(
      "backend/global_pose/regularized_field/extrapolation_regression_knots",
      regularized_field.extrapolation_regression_knots, 3);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_extrapolation_distance_m",
      regularized_field.maximum_extrapolation_distance_m, 8.0);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_planar_extrapolation_slope_m_per_m",
      regularized_field.maximum_planar_extrapolation_slope_m_per_m, 0.25);
  nh.param<double>(
      "backend/global_pose/regularized_field/maximum_vertical_extrapolation_slope_m_per_m",
      regularized_field.maximum_vertical_extrapolation_slope_m_per_m, 0.15);
  nh.param<string>(
      "backend/global_pose/regularized_field/csv_path",
      backend_global_pose_options.regularized_field_csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/regularized_correction_field.csv");
  auto &elastic_acceptance =
      backend_global_pose_options.elastic_acceptance;
  int elastic_acceptance_minimum_observations = 6;
  nh.param<int>(
      "backend/global_pose/elastic_acceptance/minimum_position_observations",
      elastic_acceptance_minimum_observations, 6);
  if (elastic_acceptance_minimum_observations < 0)
    throw std::invalid_argument(
        "Elastic acceptance observation count cannot be negative.");
  elastic_acceptance.minimum_position_observations =
      static_cast<std::size_t>(elastic_acceptance_minimum_observations);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/minimum_path_length_m",
      elastic_acceptance.minimum_path_length_m, 30.0);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/maximum_position_residual_m",
      elastic_acceptance.maximum_position_residual_m, 0.30);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/maximum_yaw_residual_deg",
      elastic_acceptance.maximum_yaw_residual_deg, 0.50);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/maximum_post_velocity_error_mps",
      elastic_acceptance.maximum_post_velocity_error_mps, 0.80);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/maximum_field_gradient_m_per_m",
      elastic_acceptance.maximum_field_gradient_m_per_m, 0.10);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/maximum_yaw_gradient_deg_per_m",
      elastic_acceptance.maximum_yaw_gradient_deg_per_m, 0.10);
  nh.param<double>(
      "backend/global_pose/elastic_acceptance/maximum_outlier_fraction",
      elastic_acceptance.maximum_outlier_fraction, 0.25);
  nh.param<string>(
      "backend/global_pose/elastic_acceptance/csv_path",
      backend_global_pose_options.elastic_acceptance_csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/elastic_segment_acceptance.csv");
  nh.param<bool>(
      "backend/global_pose/relocalization_monitor/enabled",
      backend_global_pose_options.relocalization_monitor.enabled, true);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "maximum_post_velocity_error_mps",
      backend_global_pose_options.relocalization_monitor
          .maximum_post_velocity_error_mps,
      0.8);
  nh.param<int>(
      "backend/global_pose/relocalization_monitor/minimum_observations",
      backend_global_pose_options.relocalization_monitor
          .minimum_observations,
      4);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/minimum_path_length_m",
      backend_global_pose_options.relocalization_monitor
          .minimum_path_length_m,
      30.0);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/maximum_window_length_m",
      backend_global_pose_options.relocalization_monitor
          .maximum_window_length_m,
      80.0);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "maximum_path_scale_error",
      backend_global_pose_options.relocalization_monitor
          .maximum_path_scale_error,
      0.05);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/maximum_planar_rms_m",
      backend_global_pose_options.relocalization_monitor
          .maximum_planar_rms_m,
      0.35);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/maximum_vertical_rms_m",
      backend_global_pose_options.relocalization_monitor
          .maximum_vertical_rms_m,
      0.50);
  nh.param<int>(
      "backend/global_pose/relocalization_monitor/"
      "required_consecutive_candidates",
      backend_global_pose_options.relocalization_monitor
          .required_consecutive_candidates,
      2);
  nh.param<int>(
      "backend/global_pose/relocalization_monitor/"
      "restart_after_consecutive_structural_rejections",
      backend_global_pose_options.relocalization_monitor
          .restart_after_consecutive_structural_rejections,
      3);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "restart_minimum_rejection_span_m",
      backend_global_pose_options.relocalization_monitor
          .restart_minimum_rejection_span_m,
      15.0);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "diagnostic_minimum_condition_ratio",
      backend_global_pose_options.relocalization_monitor
          .diagnostic_minimum_condition_ratio,
      0.01);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "diagnostic_minimum_scale_error",
      backend_global_pose_options.relocalization_monitor
          .diagnostic_minimum_scale_error,
      0.03);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "diagnostic_minimum_nonyaw_rotation_deg",
      backend_global_pose_options.relocalization_monitor
          .diagnostic_minimum_nonyaw_rotation_deg,
      1.0);
  nh.param<double>(
      "backend/global_pose/relocalization_monitor/"
      "diagnostic_maximum_similarity_rms_m",
      backend_global_pose_options.relocalization_monitor
          .diagnostic_maximum_similarity_rms_m,
      0.40);
  nh.param<string>(
      "backend/global_pose/relocalization_monitor/csv_path",
      backend_global_pose_options.relocalization_monitor.csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/rigid_relocalization.csv");
  backend_global_pose_options.recovery_monitor =
      backend_global_pose_options.relocalization_monitor;
  auto &recovery_monitor = backend_global_pose_options.recovery_monitor;
  recovery_monitor.maximum_planar_rms_m = 0.75;
  recovery_monitor.maximum_vertical_rms_m = 1.0;
  recovery_monitor.restart_after_consecutive_structural_rejections = 1000;
  recovery_monitor.restart_minimum_rejection_span_m = 1000.0;
  recovery_monitor.csv_path =
      std::string(ROOT_DIR) + "Log/backend/recovery_relocalization.csv";
  nh.param<double>(
      "backend/global_pose/recovery_monitor/"
      "maximum_post_velocity_error_mps",
      recovery_monitor.maximum_post_velocity_error_mps, 0.8);
  nh.param<int>(
      "backend/global_pose/recovery_monitor/minimum_observations",
      recovery_monitor.minimum_observations, 4);
  nh.param<double>(
      "backend/global_pose/recovery_monitor/minimum_path_length_m",
      recovery_monitor.minimum_path_length_m, 30.0);
  nh.param<double>(
      "backend/global_pose/recovery_monitor/maximum_window_length_m",
      recovery_monitor.maximum_window_length_m, 80.0);
  nh.param<double>(
      "backend/global_pose/recovery_monitor/maximum_path_scale_error",
      recovery_monitor.maximum_path_scale_error, 0.05);
  nh.param<double>(
      "backend/global_pose/recovery_monitor/maximum_planar_rms_m",
      recovery_monitor.maximum_planar_rms_m, 0.75);
  nh.param<double>(
      "backend/global_pose/recovery_monitor/maximum_vertical_rms_m",
      recovery_monitor.maximum_vertical_rms_m, 1.0);
  nh.param<int>(
      "backend/global_pose/recovery_monitor/"
      "required_consecutive_candidates",
      recovery_monitor.required_consecutive_candidates, 2);
  nh.param<bool>(
      "backend/global_pose/recovery_monitor/select_best_valid_suffix",
      recovery_monitor.select_best_valid_suffix, true);
  nh.param<string>(
      "backend/global_pose/recovery_monitor/csv_path",
      recovery_monitor.csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/recovery_relocalization.csv");
  nh.param<bool>(
      "backend/frontend_segment_restart/enabled",
      backend_frontend_segment_restart_enabled, true);
  int frontend_segment_minimum_seed_points = 1000;
  nh.param<int>(
      "backend/frontend_segment_restart/minimum_seed_points",
      frontend_segment_minimum_seed_points, 1000);
  if (frontend_segment_minimum_seed_points <= 0)
    throw std::runtime_error(
        "backend.frontend_segment_restart.minimum_seed_points must be "
        "positive.");
  backend_frontend_segment_minimum_seed_points =
      static_cast<std::size_t>(frontend_segment_minimum_seed_points);
  int frontend_segment_maximum_automatic_restarts = 1;
  nh.param<int>(
      "backend/frontend_segment_restart/maximum_automatic_restarts",
      frontend_segment_maximum_automatic_restarts, 1);
  if (frontend_segment_maximum_automatic_restarts <= 0)
    throw std::runtime_error(
        "backend.frontend_segment_restart.maximum_automatic_restarts must "
        "be positive.");
  backend_frontend_segment_maximum_automatic_restarts =
      static_cast<std::size_t>(
          frontend_segment_maximum_automatic_restarts);
  nh.param<double>(
      "backend/frontend_segment_restart/history_seed_path_length_m",
      backend_frontend_segment_history_seed_path_m, 20.0);
  int frontend_segment_maximum_history_seed_keyframes = 20;
  nh.param<int>(
      "backend/frontend_segment_restart/maximum_history_seed_keyframes",
      frontend_segment_maximum_history_seed_keyframes, 20);
  nh.param<double>(
      "backend/frontend_segment_restart/history_seed_point_sigma_m",
      backend_frontend_segment_history_seed_point_sigma_m, 0.10);
  if (!std::isfinite(backend_frontend_segment_history_seed_path_m) ||
      backend_frontend_segment_history_seed_path_m <= 0.0 ||
      frontend_segment_maximum_history_seed_keyframes <= 0 ||
      !std::isfinite(
          backend_frontend_segment_history_seed_point_sigma_m) ||
      backend_frontend_segment_history_seed_point_sigma_m <= 0.0)
    throw std::runtime_error(
        "backend.frontend_segment_restart history-seed parameters must be "
        "positive.");
  backend_frontend_segment_maximum_history_seed_keyframes =
      static_cast<std::size_t>(
          frontend_segment_maximum_history_seed_keyframes);
  nh.param<double>(
      "backend/frontend_segment_restart/position_sigma_floor_m",
      backend_frontend_segment_position_sigma_floor_m, 0.10);
  nh.param<double>(
      "backend/frontend_segment_restart/rotation_sigma_floor_deg",
      backend_frontend_segment_rotation_sigma_floor_deg, 1.0);
  if (!std::isfinite(backend_frontend_segment_position_sigma_floor_m) ||
      backend_frontend_segment_position_sigma_floor_m <= 0.0 ||
      !std::isfinite(backend_frontend_segment_rotation_sigma_floor_deg) ||
      backend_frontend_segment_rotation_sigma_floor_deg <= 0.0)
    throw std::runtime_error(
        "backend.frontend_segment_restart covariance floors must be "
        "positive.");
  nh.param<string>(
      "backend/frontend_segment_restart/csv_path",
      backend_frontend_segment_csv_path,
      std::string(ROOT_DIR) + "Log/backend/frontend_segments.csv");
  nh.param<string>(
      "backend/frontend_segment_restart/supervisor_csv_path",
      backend_frontend_restart_supervisor_csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/frontend_restart_supervisor.csv");
  nh.param<bool>(
      "backend/frontend_segment_restart/observability_guard/enabled",
      backend_lio_observability_guard_enabled, true);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/"
      "translation_soft_ratio",
      backend_lio_observability_translation_soft_ratio, 0.15);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/"
      "translation_full_ratio",
      backend_lio_observability_translation_full_ratio, 0.05);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/"
      "rotation_soft_ratio",
      backend_lio_observability_rotation_soft_ratio, 0.25);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/"
      "rotation_full_ratio",
      backend_lio_observability_rotation_full_ratio, 0.08);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/maximum_gain",
      backend_lio_observability_maximum_constraint_gain, 3.0);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/restart_score",
      backend_lio_observability_restart_score, 0.65);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/"
      "restart_residual_m",
      backend_lio_observability_restart_residual_m, 1.0);
  nh.param<double>(
      "backend/frontend_segment_restart/observability_guard/"
      "restart_velocity_error_mps",
      backend_lio_observability_restart_velocity_error_mps, 0.8);
  nh.param<int>(
      "backend/frontend_segment_restart/observability_guard/"
      "restart_required_observations",
      backend_lio_observability_restart_required_observations, 3);
  nh.param<bool>(
      "backend/frontend_segment_restart/elastic_saturation_guard/enabled",
      backend_elastic_saturation_restart_enabled, false);
  nh.param<double>(
      "backend/frontend_segment_restart/elastic_saturation_guard/"
      "restart_residual_m",
      backend_elastic_saturation_restart_residual_m, 0.50);
  nh.param<double>(
      "backend/frontend_segment_restart/elastic_saturation_guard/"
      "restart_gradient_ratio",
      backend_elastic_saturation_restart_gradient_ratio, 0.95);
  if (!(backend_lio_observability_translation_full_ratio > 0.0 &&
        backend_lio_observability_translation_soft_ratio >
            backend_lio_observability_translation_full_ratio &&
        backend_lio_observability_rotation_full_ratio > 0.0 &&
        backend_lio_observability_rotation_soft_ratio >
            backend_lio_observability_rotation_full_ratio &&
        backend_lio_observability_maximum_constraint_gain >= 1.0 &&
        backend_lio_observability_restart_score > 0.0 &&
        backend_lio_observability_restart_score <= 1.0 &&
        backend_lio_observability_restart_residual_m > 0.0 &&
        backend_lio_observability_restart_velocity_error_mps > 0.0 &&
        backend_lio_observability_restart_required_observations > 0 &&
        backend_elastic_saturation_restart_residual_m > 0.0 &&
        backend_elastic_saturation_restart_gradient_ratio > 0.0 &&
        backend_elastic_saturation_restart_gradient_ratio <= 1.0))
    throw std::runtime_error(
        "backend observability-guard parameters are invalid.");
  nh.param<bool>(
      "backend/rtk/velocity_guard/enabled",
      backend_rtk_velocity_guard_options.enabled, true);
  nh.param<double>(
      "backend/rtk/velocity_guard/minimum_time_interval_sec",
      backend_rtk_velocity_selector_options.minimum_time_interval_sec,
      0.8);
  nh.param<double>(
      "backend/rtk/velocity_guard/maximum_time_interval_sec",
      backend_rtk_velocity_selector_options.maximum_time_interval_sec,
      1.2);
  nh.param<double>(
      "backend/rtk/velocity_guard/minimum_translation_m",
      backend_rtk_velocity_selector_options.minimum_translation_m,
      0.5);
  nh.param<double>(
      "backend/rtk/velocity_guard/evidence_max_age_sec",
      backend_rtk_velocity_evidence_max_age_sec, 1.5);
  nh.param<double>(
      "backend/rtk/velocity_guard/low_pass_time_constant_sec",
      backend_rtk_velocity_guard_options.low_pass_time_constant_sec, 2.0);
  nh.param<double>(
      "backend/rtk/velocity_guard/receiver_low_pass_time_constant_sec",
      backend_rtk_velocity_guard_options
          .receiver_low_pass_time_constant_sec,
      0.25);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "maximum_receiver_position_difference_error_mps",
      backend_rtk_velocity_guard_options
          .maximum_receiver_position_difference_error_mps,
      1.5);
  nh.param<double>(
      "backend/rtk/velocity_guard/activation_velocity_error_mps",
      backend_rtk_velocity_guard_options.activation_velocity_error_mps,
      1.0);
  nh.param<double>(
      "backend/rtk/velocity_guard/activation_speed_ratio",
      backend_rtk_velocity_guard_options.activation_speed_ratio, 1.35);
  nh.param<int>(
      "backend/rtk/velocity_guard/activation_consecutive_observations",
      backend_rtk_velocity_guard_options
          .activation_consecutive_observations,
      2);
  nh.param<double>(
      "backend/rtk/velocity_guard/recovery_velocity_error_mps",
      backend_rtk_velocity_guard_options.recovery_velocity_error_mps, 0.5);
  nh.param<int>(
      "backend/rtk/velocity_guard/recovery_consecutive_observations",
      backend_rtk_velocity_guard_options.recovery_consecutive_observations,
      3);
  nh.param<double>(
      "backend/rtk/velocity_guard/correction_time_constant_sec",
      backend_rtk_velocity_guard_options.correction_time_constant_sec,
      0.8);
  nh.param<double>(
      "backend/rtk/velocity_guard/maximum_planar_correction_mps",
      backend_rtk_velocity_guard_options.maximum_planar_correction_mps,
      2.0);
  nh.param<double>(
      "backend/rtk/velocity_guard/maximum_vertical_correction_mps",
      backend_rtk_velocity_guard_options.maximum_vertical_correction_mps,
      0.75);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "maximum_planar_correction_acceleration_mps2",
      backend_rtk_velocity_guard_options
          .maximum_planar_correction_acceleration_mps2,
      0.75);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "maximum_vertical_correction_acceleration_mps2",
      backend_rtk_velocity_guard_options
          .maximum_vertical_correction_acceleration_mps2,
      0.30);
  nh.param<bool>(
      "backend/rtk/velocity_guard/healthy_vertical_aiding_enabled",
      backend_rtk_velocity_guard_options.healthy_vertical_aiding_enabled,
      true);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "healthy_vertical_activation_error_mps",
      backend_rtk_velocity_guard_options
          .healthy_vertical_activation_error_mps,
      0.08);
  nh.param<int>(
      "backend/rtk/velocity_guard/"
      "healthy_vertical_activation_observations",
      backend_rtk_velocity_guard_options
          .healthy_vertical_activation_observations,
      5);
  nh.param<double>(
      "backend/rtk/velocity_guard/healthy_vertical_time_constant_sec",
      backend_rtk_velocity_guard_options
          .healthy_vertical_time_constant_sec,
      4.0);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "healthy_vertical_maximum_acceleration_mps2",
      backend_rtk_velocity_guard_options
          .healthy_vertical_maximum_acceleration_mps2,
      0.08);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_tracking_time_constant_sec",
      backend_rtk_velocity_guard_options
          .recovery_tracking_time_constant_sec,
      0.35);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_tracking_planar_acceleration_mps2",
      backend_rtk_velocity_guard_options
          .recovery_tracking_planar_acceleration_mps2,
      1.50);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_tracking_vertical_acceleration_mps2",
      backend_rtk_velocity_guard_options
          .recovery_tracking_vertical_acceleration_mps2,
      0.75);
  nh.param<bool>(
      "backend/rtk/velocity_guard/recovery_frame_tracking_enabled",
      backend_rtk_recovery_frame_tracking_enabled, true);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_frame_tracking_maximum_age_sec",
      backend_rtk_recovery_frame_tracking_maximum_age_sec, 1.5);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_frame_tracking_planar_acceleration_mps2",
      backend_rtk_recovery_frame_tracking_planar_acceleration_mps2, 3.0);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_frame_tracking_vertical_acceleration_mps2",
      backend_rtk_recovery_frame_tracking_vertical_acceleration_mps2, 2.0);
  nh.param<string>(
      "backend/rtk/velocity_guard/recovery_frame_tracking_csv_path",
      backend_rtk_recovery_frame_tracking_csv_path,
      std::string(ROOT_DIR) +
          "Log/backend/rtk_recovery_frame_tracking.csv");
  if (!std::isfinite(
          backend_rtk_recovery_frame_tracking_maximum_age_sec) ||
      backend_rtk_recovery_frame_tracking_maximum_age_sec <= 0.0 ||
      !std::isfinite(
          backend_rtk_recovery_frame_tracking_planar_acceleration_mps2) ||
      backend_rtk_recovery_frame_tracking_planar_acceleration_mps2 <= 0.0 ||
      !std::isfinite(
          backend_rtk_recovery_frame_tracking_vertical_acceleration_mps2) ||
      backend_rtk_recovery_frame_tracking_vertical_acceleration_mps2 <= 0.0)
    throw std::runtime_error(
        "backend.rtk.velocity_guard recovery-frame tracking parameters "
        "must be positive.");
  nh.param<double>(
      "backend/rtk/velocity_guard/recovery_position_soft_radius_m",
      backend_rtk_velocity_guard_options.recovery_position_soft_radius_m,
      0.15);
  nh.param<double>(
      "backend/rtk/velocity_guard/recovery_position_full_radius_m",
      backend_rtk_velocity_guard_options.recovery_position_full_radius_m,
      1.50);
  nh.param<double>(
      "backend/rtk/velocity_guard/recovery_position_release_radius_m",
      backend_rtk_velocity_guard_options.recovery_position_release_radius_m,
      0.10);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_position_capture_minimum_stiffness",
      backend_rtk_velocity_guard_options
          .recovery_position_capture_minimum_stiffness,
      0.50);
  nh.param<double>(
      "backend/rtk/velocity_guard/recovery_position_time_constant_sec",
      backend_rtk_velocity_guard_options.recovery_position_time_constant_sec,
      4.0);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_maximum_planar_closure_velocity_mps",
      backend_rtk_velocity_guard_options
          .recovery_maximum_planar_closure_velocity_mps,
      0.60);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "recovery_maximum_vertical_closure_velocity_mps",
      backend_rtk_velocity_guard_options
          .recovery_maximum_vertical_closure_velocity_mps,
      0.60);
  nh.param<bool>(
      "backend/rtk/velocity_guard/emergency_restart_enabled",
      backend_rtk_velocity_guard_options.emergency_restart_enabled, true);
  nh.param<double>(
      "backend/rtk/velocity_guard/"
      "emergency_restart_velocity_error_mps",
      backend_rtk_velocity_guard_options
          .emergency_restart_velocity_error_mps,
      2.0);
  nh.param<int>(
      "backend/rtk/velocity_guard/"
      "emergency_restart_consecutive_observations",
      backend_rtk_velocity_guard_options
          .emergency_restart_consecutive_observations,
      3);
  nh.param<double>(
      "backend/rtk/velocity_guard/state_velocity_covariance_floor_mps",
      backend_rtk_velocity_covariance_floor_mps, 0.50);
  if (!std::isfinite(backend_rtk_velocity_covariance_floor_mps) ||
      backend_rtk_velocity_covariance_floor_mps <= 0.0)
    throw std::runtime_error(
        "backend.rtk.velocity_guard state velocity covariance floor must "
        "be positive.");
  nh.param<string>(
      "backend/rtk/velocity_guard/csv_path",
      backend_rtk_velocity_guard_options.csv_path,
      std::string(ROOT_DIR) + "Log/backend/rtk_velocity_guard.csv");
  if (backend_rtk_input_enabled && !imu_standard_rfu)
    throw std::runtime_error(
        "backend.rtk input requires the standard CGI-610 RFU data chain.");
  if (backend_rtk_fusion_enabled &&
      (!backend_rtk_input_enabled || !backend_pose_graph_enabled))
    throw std::runtime_error(
        "backend.rtk fusion requires RTK input and the pose graph.");
  if (!std::isfinite(backend_rtk_innovation_prediction_sigma_m) ||
      backend_rtk_innovation_prediction_sigma_m <= 0.0)
    throw std::runtime_error(
        "backend.rtk innovation diagnostic sigma must be positive.");
  if (!std::isfinite(backend_rtk_velocity_evidence_max_age_sec) ||
      backend_rtk_velocity_evidence_max_age_sec <= 0.0)
    throw std::runtime_error(
        "backend.rtk velocity evidence age must be positive.");
  if ((backend_loop_factors_enabled || backend_global_map_enabled ||
       backend_rtk_fusion_enabled) &&
      backend_map_frame_id == backend_frontend_frame_id)
    throw std::runtime_error(
        "Global backend products require distinct backend.map_frame_id and "
        "backend.frontend_frame_id for map->odom.");

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
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

  string source_description;
  for (std::size_t index = 0; index < lidar_sources.size(); ++index)
  {
    if (index != 0) source_description += ", ";
    source_description += lidar_sources[index]->topic + "->" +
                          lidar_sources[index]->transform_key;
  }
  RCLCPP_INFO(
      node_->get_logger(),
      "Loaded %zu LiDAR source(s) from %s: %s",
      lidar_sources.size(), multi_lidar_calibration_file.c_str(),
      source_description.c_str());
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
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;

  if (backend_frontend_segment_restart_enabled)
  {
    if (slam_mode_ != ONLY_LIO)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Frontend segment restart is disabled outside pure LIO mode; "
          "camera/VIO state is intentionally untouched.");
      backend_frontend_segment_restart_enabled = false;
    }
    else if (!backend_rtk_fusion_enabled || !backend_pose_graph_enabled)
    {
      throw std::runtime_error(
          "backend.frontend_segment_restart requires RTK fusion and the "
          "separated pose graph.");
    }
  }

  if (backend_rtk_input_enabled)
  {
    rtk_observation_buffer = std::make_unique<
        my_livo::backend::RtkObservationBuffer>(
            backend_rtk_buffer_options);
    rtk_factor_selector = std::make_unique<
        my_livo::backend::RtkFactorSelector>(
            backend_rtk_selector_options);
    rtk_velocity_selector = std::make_unique<
        my_livo::backend::RtkFactorSelector>(
            backend_rtk_velocity_selector_options);
    rtk_fusion_diagnostics = std::make_unique<
        my_livo::backend::RtkFusionDiagnostics>(
            backend_rtk_diagnostics_options);
    rtk_velocity_guard = std::make_unique<
        my_livo::backend::RtkVelocityGuard>(
            backend_rtk_velocity_guard_options);
    RCLCPP_INFO(
        node_->get_logger(),
        "CGI-610 RTK buffer enabled: required mode=%d, status age<=%.3f "
        "s, interpolation gap<=%.3f s, global_fusion=%d, "
        "velocity_guard=%d",
        backend_rtk_buffer_options.required_ins_pos_mode,
        backend_rtk_buffer_options.maximum_status_age_sec,
        backend_rtk_buffer_options.maximum_interpolation_gap_sec,
        static_cast<int>(backend_rtk_fusion_enabled),
        static_cast<int>(backend_rtk_velocity_guard_options.enabled));
  }

  if (backend_keyframes_enabled)
  {
    keyframe_manager = std::make_unique<
        my_livo::backend::KeyframeManager>(backend_keyframe_options);
    backend_keyframe_path.header.frame_id = backend_frontend_frame_id;
    RCLCPP_INFO(
        node_->get_logger(),
        "Backend keyframes enabled: translation=%.3f m, rotation=%.3f deg, "
        "interval=[%.3f, %.3f] s, cloud leaf=%.3f m, frame=%s",
        backend_keyframe_options.translation_threshold_m,
        backend_keyframe_options.rotation_threshold_deg,
        backend_keyframe_options.minimum_interval_sec,
        backend_keyframe_options.maximum_interval_sec,
        backend_keyframe_options.cloud_leaf_size_m,
        backend_frontend_frame_id.c_str());

    if (backend_pose_graph_enabled)
    {
      pose_graph_optimizer = std::make_unique<
          my_livo::backend::PoseGraphOptimizer>(backend_pose_graph_options);
      global_pose_layer = std::make_unique<
          my_livo::backend::GlobalPoseLayer>(backend_global_pose_options);
      backend_optimized_path.header.frame_id = backend_map_frame_id;
      RCLCPP_INFO(
          node_->get_logger(),
          "Separated global pose layer enabled: A0 fixed, bounded C(s) "
          "spacing/low-pass=%.1f/%.1f m, planar/yaw gradients="
          "%.4f m/m/%.4f deg/m; monitor elastic/relocalization="
          "%.3f/%.3f m/m; RTK cannot modify the local SLAM graph",
          backend_global_pose_options.correction_field.knot_spacing_m,
          backend_global_pose_options.correction_field
              .spatial_low_pass_length_m,
          backend_global_pose_options.correction_field
              .maximum_planar_gradient_m_per_m,
          backend_global_pose_options.correction_field
              .maximum_yaw_gradient_deg_per_m,
          backend_global_pose_options.correction_monitor
              .elastic_gradient_limit_m_per_m,
          backend_global_pose_options.correction_monitor
              .relocalization_gradient_m_per_m);
      if (backend_global_map_enabled)
      {
        optimized_global_map = std::make_unique<
            my_livo::backend::OptimizedGlobalMap>(
                backend_global_map_options);
        RCLCPP_INFO(
            node_->get_logger(),
            "Optimized tiled global map enabled: leaf=%.3f m, tile=%.1f m, "
            "periodic/graph interval=%zu/%zu KFs, correction "
            "threshold=%.2f m/%.2f deg, rigid_submaps=%d/%.1f m, "
            "spatial_deformation=%d/%zu/%.1f m, "
            "output=%s",
            backend_global_map_options.voxel_leaf_size_m,
            backend_global_map_options.tile_size_m,
            backend_global_map_options.periodic_keyframe_interval,
            backend_global_map_options
                .minimum_graph_rebuild_keyframe_interval,
            backend_global_map_options.incremental_max_pose_change_m,
            backend_global_map_options.incremental_max_pose_change_deg,
            static_cast<int>(
                backend_global_map_options.preserve_rigid_local_submaps),
            backend_global_map_options.rigid_submap_length_m,
            static_cast<int>(
                backend_global_map_options.spatial_deformation_enabled),
            backend_global_map_options.spatial_deformation_neighbors,
            backend_global_map_options.spatial_deformation_sigma_m,
            backend_global_map_options.pcd_path.c_str());
      }
      RCLCPP_INFO(
          node_->get_logger(),
          "GTSAM iSAM2 odometry graph enabled: translation sigma=%.3f m, "
          "rotation sigma=%.3f deg, relin threshold=%.4f, relin skip=%d, "
          "extra updates=%d, frame=%s",
          backend_pose_graph_options.odometry_translation_sigma_m,
          backend_pose_graph_options.odometry_rotation_sigma_deg,
          backend_pose_graph_options.relinearize_threshold,
          backend_pose_graph_options.relinearize_skip,
          backend_pose_graph_options.additional_update_steps,
          backend_map_frame_id.c_str());

      if (backend_loop_detection_enabled)
      {
        loop_candidate_detector = std::make_unique<
            my_livo::backend::LoopCandidateDetector>(
                backend_loop_detection_options);
        RCLCPP_INFO(
            node_->get_logger(),
            "Loop candidate detector enabled: check every %lu KFs, "
            "history gap >= %lu KFs / %.1f s, range <= %.1f m, "
            "height <= %.1f m, at most %zu candidates",
            static_cast<unsigned long>(
                backend_loop_detection_options.check_interval_keyframes),
            static_cast<unsigned long>(backend_loop_detection_options
                                           .minimum_id_separation_keyframes),
            backend_loop_detection_options.minimum_time_separation_sec,
            backend_loop_detection_options.maximum_planar_distance_m,
            backend_loop_detection_options.maximum_height_difference_m,
            backend_loop_detection_options.maximum_candidates_per_keyframe);

        if (backend_loop_registration_enabled)
        {
          loop_registration = std::make_unique<
              my_livo::backend::LoopRegistration>(
                  backend_loop_registration_options);
          if (backend_loop_verification_enabled)
          {
            loop_verifier = std::make_unique<
                my_livo::backend::LoopVerifier>(
                    backend_loop_verification_options);
            RCLCPP_INFO(
                node_->get_logger(),
                "Loop verifier enabled: probability>=%.3f, fitness<=%.3f "
                "m^2, overlap>=%.3f, RMSE<=%.3f m, neighbor error<=%.3f "
                "m/%.3f deg",
                backend_loop_verification_options
                    .minimum_transformation_probability,
                backend_loop_verification_options.maximum_fitness_score_m2,
                backend_loop_verification_options.minimum_overlap,
                backend_loop_verification_options.maximum_overlap_rmse_m,
                backend_loop_verification_options
                    .maximum_neighbor_translation_error_m,
                backend_loop_verification_options
                    .maximum_neighbor_rotation_error_deg);
          }
          loop_registration->SetResultCallback(
              [this](const my_livo::backend::LoopRegistrationResult &result) {
                const char *status =
                    my_livo::backend::LoopRegistrationStatusToString(
                        result.status);
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Loop NDT %lu<-%lu: status=%s, converged=%d (%zu/%zu "
                    "levels), probability=%.4f, fitness=%.4f m^2, "
                    "overlap=%.3f, correction=%.3f m/%.3f deg, time=%.1f "
                    "ms (diagnostic only; no graph factor)",
                    static_cast<unsigned long>(result.candidate.candidate_id),
                    static_cast<unsigned long>(result.candidate.current_id),
                    status, static_cast<int>(result.converged),
                    result.converged_levels, result.levels.size(),
                    result.transformation_probability,
                    result.fitness_score_m2, result.overlap,
                    result.correction_translation_m,
                    result.correction_angle_deg,
                    result.registration_time_ms);
                if (loop_verifier)
                  handleLoopVerificationDecisions(
                      loop_verifier->Add(result));
                if (result.status !=
                    my_livo::backend::LoopRegistrationStatus::kCompleted)
                {
                  RCLCPP_WARN(
                      node_->get_logger(),
                      "Loop NDT %lu<-%lu diagnostic: %s",
                      static_cast<unsigned long>(
                          result.candidate.candidate_id),
                      static_cast<unsigned long>(result.candidate.current_id),
                      result.diagnostic.c_str());
                  return;
                }

                visualization_msgs::MarkerArray markers;
                const auto make_marker = [this, &result](
                    const char *name, const Eigen::Vector3d &from,
                    const Eigen::Vector3d &to, float red, float green,
                    float blue) {
                  visualization_msgs::Marker marker;
                  marker.header.frame_id = backend_frontend_frame_id;
                  marker.header.stamp =
                      stampFromSec(result.candidate.current_timestamp);
                  marker.ns = name;
                  marker.id = static_cast<int>(
                      backend_loop_registration_marker_id++);
                  marker.type = visualization_msgs::Marker::LINE_LIST;
                  marker.action = visualization_msgs::Marker::ADD;
                  marker.pose.orientation.w = 1.0;
                  marker.scale.x = 0.05;
                  marker.color.r = red;
                  marker.color.g = green;
                  marker.color.b = blue;
                  marker.color.a = 0.95F;
                  geometry_msgs::msg::Point start;
                  start.x = from.x();
                  start.y = from.y();
                  start.z = from.z();
                  geometry_msgs::msg::Point finish;
                  finish.x = to.x();
                  finish.y = to.y();
                  finish.z = to.z();
                  marker.points.push_back(start);
                  marker.points.push_back(finish);
                  return marker;
                };
                const my_livo::backend::Pose3d registered_current =
                    result.T_map_candidate_snapshot *
                    result.T_candidate_current;
                markers.markers.push_back(make_marker(
                    "backend_loop_ndt_links",
                    result.T_map_candidate_snapshot.translation,
                    registered_current.translation,
                    result.converged ? 0.10F : 0.65F,
                    result.converged ? 0.85F : 0.65F, 1.0F));
                markers.markers.push_back(make_marker(
                    "backend_loop_ndt_corrections",
                    result.T_map_current_snapshot.translation,
                    registered_current.translation, 1.0F, 0.15F, 0.85F));
                pubBackendLoopRegistrations->publish(markers);
              });
          RCLCPP_INFO(
              node_->get_logger(),
              "Asynchronous multi-resolution NDT enabled: levels=%zu, "
              "target window=+/- %d KFs stride %d, max queue=%zu",
              backend_loop_registration_options.resolutions_m.size(),
              backend_loop_registration_options
                  .target_submap_half_width_keyframes,
              backend_loop_registration_options
                  .target_submap_stride_keyframes,
              backend_loop_registration_options.maximum_queue_size);
        }
      }
    }
  }
}

void LIVMapper::initializeFiles() 
{
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/result");
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/pcd");
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/image");
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/backend");
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/Colmap/sparse/0");
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
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
  if (backend_rtk_input_enabled && !backend_rtk_decision_csv_path.empty())
  {
    backend_rtk_decision_stream.open(
        backend_rtk_decision_csv_path, std::ios::out | std::ios::trunc);
    if (!backend_rtk_decision_stream.is_open())
      throw std::runtime_error(
          "Cannot open RTK decision CSV: " +
          backend_rtk_decision_csv_path);
    backend_rtk_decision_stream
        << "keyframe_id,timestamp,decision,factor_added,measurement_x,"
           "measurement_y,measurement_z,innovation_x,innovation_y,"
           "innovation_z,innovation_chi2\n";
  }
  if (backend_rtk_input_enabled &&
      !backend_rtk_recovery_frame_tracking_csv_path.empty())
  {
    backend_rtk_recovery_frame_tracking_stream.open(
        backend_rtk_recovery_frame_tracking_csv_path,
        std::ios::out | std::ios::trunc);
    if (!backend_rtk_recovery_frame_tracking_stream.is_open())
      throw std::runtime_error(
          "Cannot open recovery frame-tracking CSV: " +
          backend_rtk_recovery_frame_tracking_csv_path);
    backend_rtk_recovery_frame_tracking_stream
        << "timestamp,source_keyframe_id,target_timestamp,target_age_sec,"
           "interval_sec,target_vx,target_vy,target_vz,before_vx,"
           "before_vy,before_vz,after_vx,after_vy,after_vz,correction_vx,"
           "correction_vy,correction_vz\n";
  }
  if (backend_frontend_segment_restart_enabled &&
      !backend_frontend_segment_csv_path.empty())
  {
    backend_frontend_segment_stream.open(
        backend_frontend_segment_csv_path,
        std::ios::out | std::ios::trunc);
    if (!backend_frontend_segment_stream.is_open())
      throw std::runtime_error(
          "Cannot open frontend-segment CSV: " +
          backend_frontend_segment_csv_path);
    backend_frontend_segment_stream
        << "segment_id,trigger_keyframe_id,request_timestamp,"
           "restart_timestamp,reason,evidence_span_m,seed_points,"
           "current_seed_points,history_seed_keyframes,history_seed_path_m,"
           "history_seed_points,"
           "deleted_voxels,map_voxels_after,pose_delta_m,"
           "rotation_delta_deg,velocity_delta_mps,bias_g_delta,"
           "bias_a_delta,gravity_delta,gate_acknowledged,restart_kind,"
           "target_velocity_x,target_velocity_y,target_velocity_z,"
           "rtk_anchor_x,rtk_anchor_y,rtk_anchor_z\n";
  }
  if (backend_frontend_segment_restart_enabled &&
      !backend_frontend_restart_supervisor_csv_path.empty())
  {
    backend_frontend_restart_supervisor_stream.open(
        backend_frontend_restart_supervisor_csv_path,
        std::ios::out | std::ios::trunc);
    if (!backend_frontend_restart_supervisor_stream.is_open())
      throw std::runtime_error(
          "Cannot open frontend-restart supervisor CSV: " +
          backend_frontend_restart_supervisor_csv_path);
    backend_frontend_restart_supervisor_stream
        << "request_id,trigger_keyframe_id,timestamp,action,reason,"
           "evidence_span_m,geometry_failure,condition_ratio,"
           "similarity_scale,similarity_rms_m\n";
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
    if (backend_rtk_input_enabled)
      sub_ins_status = node_->create_subscription<
          diagnostic_msgs::msg::DiagnosticArray>(
              ins_status_topic, imu_qos,
              std::bind(&LIVMapper::ins_status_cbk, this,
                        std::placeholders::_1));
  }
  sub_img = node_->create_subscription<sensor_msgs::Image>(
      img_topic, image_qos, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));

  pubLaserCloudFullRes = node_->create_publisher<sensor_msgs::PointCloud2>("/cloud_registered", 10);
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
  if (backend_keyframes_enabled)
  {
    pubBackendKeyframePath =
        node_->create_publisher<nav_msgs::Path>(
            "/backend/keyframe_path_raw", 10);
    pubBackendKeyframeCloud =
        node_->create_publisher<sensor_msgs::PointCloud2>(
            "/backend/keyframe_cloud_raw", 2);
    if (backend_pose_graph_enabled)
    {
      pubBackendOptimizedPath =
          node_->create_publisher<nav_msgs::Path>(
              "/backend/keyframe_path_optimized", 10);
      pubBackendLocalSlamPath =
          node_->create_publisher<nav_msgs::Path>(
              "/backend/keyframe_path_local", 10);
      pubBackendQuarantinedPath =
          node_->create_publisher<nav_msgs::Path>(
              "/backend/keyframe_path_quarantined", 10);
      pubBackendRelocalizationStatus =
          node_->create_publisher<visualization_msgs::MarkerArray>(
              "/backend/relocalization_status",
              rclcpp::QoS(1).transient_local().reliable());
      pubBackendFrontendRestartRequest = node_->create_publisher<
          diagnostic_msgs::msg::DiagnosticArray>(
              "/backend/frontend_restart_required",
              rclcpp::QoS(1).transient_local().reliable());
      pubBackendOptimizedOdometry =
          node_->create_publisher<nav_msgs::Odometry>(
              "/backend/odometry_optimized", 10);
      pubBackendCurrentFrameGlobal =
          node_->create_publisher<sensor_msgs::PointCloud2>(
              "/backend/current_frame_global", 2);
      if (backend_global_map_enabled)
        pubBackendOptimizedGlobalMap =
            node_->create_publisher<sensor_msgs::PointCloud2>(
                "/backend/global_map_optimized", 1);
      if (backend_loop_detection_enabled)
      {
        pubBackendLoopCandidates =
            node_->create_publisher<visualization_msgs::MarkerArray>(
                "/backend/loop_candidates", 10);
        if (backend_loop_registration_enabled)
        {
          pubBackendLoopRegistrations =
              node_->create_publisher<visualization_msgs::MarkerArray>(
                  "/backend/loop_registrations", 10);
          if (backend_loop_verification_enabled)
            pubBackendVerifiedLoops =
                node_->create_publisher<visualization_msgs::MarkerArray>(
                    "/backend/verified_loops", 10);
        }
      }
    }
  }
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

bool LIVMapper::processImu()
{
  // double t0 = omp_get_wtime();

  const bool was_initializing = p_imu->imu_need_init;
  const StatesGroup state_before = _state;
  try
  {
    p_imu->Process2(LidarMeasures, _state, feats_undistort);
  }
  catch (const std::runtime_error &error)
  {
    // The input callbacks enforce strict time ordering, but an already
    // buffered cloud can still straddle a newly detected IMU discontinuity.
    // Reject that measurement group instead of allowing bad point-time data
    // to terminate a long offline mapping run.
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping LiDAR/IMU measurement group: %s", error.what());
    _state = state_before;
    feats_undistort->clear();
    double restart_time = std::max(
        LidarMeasures.last_lio_update_time,
        minimum_lidar_time_after_imu_gap);
    if (!p_imu->imu_need_init && last_timestamp_imu > 0.0 &&
        std::isfinite(last_timestamp_imu))
    {
      restart_time = std::max(restart_time, last_timestamp_imu);
      auto recovery_imu =
          std::make_shared<sensor_msgs::Imu>(newest_imu);
      p_imu->Reset(restart_time, recovery_imu);
      minimum_lidar_time_after_imu_gap = restart_time;
      imu_buffer.clear();
      lid_raw_data_buffer.clear();
      lid_header_time_buffer.clear();
      for (auto &source : lidar_sources) source->pending_frames.clear();
      {
        std::lock_guard<std::mutex> propagation_lock(mtx_buffer_imu_prop);
        prop_imu_buffer.clear();
        imu_propagate = _state;
        latest_ekf_state = _state;
        latest_ekf_time = restart_time;
        new_imu = false;
        state_update_flg = true;
      }
    }
    LidarMeasures = LidarMeasureGroup();
    LidarMeasures.last_lio_update_time = restart_time;
    lidar_pushed = false;
    return false;
  }

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
  }

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
  return true;
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
    if (sample_count == 0) reference = sample.orientation;
    Eigen::Quaterniond aligned = sample.orientation;
    if (reference.coeffs().dot(aligned.coeffs()) < 0.0) aligned.coeffs() *= -1.0;
    quaternion_sum += aligned.coeffs();
    mean_position += sample.position;
    ++sample_count;
  }

  if (sample_count == 0)
  {
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

  if (rtk_fusion_diagnostics)
  {
    double squared_position_error_sum = 0.0;
    double maximum_position_error = 0.0;
    double maximum_orientation_error_rad = 0.0;
    double first_sample_timestamp = initialization_time;
    double last_sample_timestamp = initialization_time;
    std::size_t diagnostic_sample_count = 0;
    for (const MinePose &sample : mine_pose_samples)
    {
      if (sample.stamp > initialization_time) break;
      if (diagnostic_sample_count == 0)
        first_sample_timestamp = sample.stamp;
      last_sample_timestamp = sample.stamp;
      const double position_error =
          (sample.position - mean_position).norm();
      squared_position_error_sum += position_error * position_error;
      maximum_position_error = std::max(
          maximum_position_error, position_error);
      maximum_orientation_error_rad = std::max(
          maximum_orientation_error_rad,
          mean_orientation.angularDistance(sample.orientation));
      ++diagnostic_sample_count;
    }
    my_livo::backend::RtkFusionDiagnostics::InitialAlignmentRecord record;
    record.initialization_timestamp = initialization_time;
    record.first_sample_timestamp = first_sample_timestamp;
    record.last_sample_timestamp = last_sample_timestamp;
    record.sample_count = diagnostic_sample_count;
    record.T_mine_lio_initial = my_livo::backend::Pose3d(
        mean_orientation, mean_position);
    record.position_rms_m = std::sqrt(
        squared_position_error_sum /
        static_cast<double>(diagnostic_sample_count));
    record.position_max_m = maximum_position_error;
    record.orientation_max_deg =
        maximum_orientation_error_rad * 180.0 / M_PI;
    rtk_fusion_diagnostics->RecordInitialAlignment(record);
  }

  // Change only the estimator's initial world frame. Subsequent RTK positions
  // never enter state propagation or measurement updates.
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
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

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
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
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

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;
  applyBackendRecoveryFrameVelocityTracking(
      LidarMeasures.last_lio_update_time);

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
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
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
  handleBackendKeyframe();
  executePendingFrontendSegmentRestart();

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publishBackendCurrentFrame();
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);
  publishReferenceTrajectory(LidarMeasures.last_lio_update_time);

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

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleBackendKeyframe()
{
  if (!keyframe_manager) return;
  if (!feats_down_body || feats_down_body->empty())
    throw std::runtime_error(
        "Cannot create backend keyframe from an empty LIO cloud.");

  // Backend covariance order is [position, rotation], matching Miao/g2o
  // pose-graph information matrices. FAST-LIVO2 state order is
  // [rotation, position, exposure, velocity, ...].
  my_livo::backend::Matrix6d odom_covariance =
      my_livo::backend::Matrix6d::Zero();
  odom_covariance.block<3, 3>(0, 0) = _state.cov.block<3, 3>(3, 3);
  odom_covariance.block<3, 3>(3, 3) = _state.cov.block<3, 3>(0, 0);
  odom_covariance.block<3, 3>(0, 3) = _state.cov.block<3, 3>(3, 0);
  odom_covariance.block<3, 3>(3, 0) = _state.cov.block<3, 3>(0, 3);

  const my_livo::backend::Pose3d T_odom_body(
      _state.rot_end, _state.pos_end);
  const auto keyframe = keyframe_manager->TryCreate(
      LidarMeasures.last_lio_update_time, T_odom_body,
      [this]() -> my_livo::backend::KeyframeCloud::ConstPtr {
        my_livo::backend::KeyframeCloud::Ptr cloud_body(
            new my_livo::backend::KeyframeCloud());
        cloud_body->reserve(feats_down_body->size());
        for (const PointType &point_lidar : feats_down_body->points)
        {
          const V3D point_body =
              extR * V3D(point_lidar.x, point_lidar.y, point_lidar.z) +
              extT;
          my_livo::backend::KeyframePoint point;
          point.x = static_cast<float>(point_body.x());
          point.y = static_cast<float>(point_body.y());
          point.z = static_cast<float>(point_body.z());
          point.intensity = point_lidar.intensity;
          cloud_body->push_back(point);
        }
        return cloud_body;
      },
      odom_covariance, voxelmap_manager->latest_observability_);
  if (!keyframe) return;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = backend_frontend_frame_id;
  pose.header.stamp = stampFromSec(keyframe->timestamp());
  pose.pose.position.x = keyframe->T_odom_body().translation.x();
  pose.pose.position.y = keyframe->T_odom_body().translation.y();
  pose.pose.position.z = keyframe->T_odom_body().translation.z();
  pose.pose.orientation.x = keyframe->T_odom_body().rotation.x();
  pose.pose.orientation.y = keyframe->T_odom_body().rotation.y();
  pose.pose.orientation.z = keyframe->T_odom_body().rotation.z();
  pose.pose.orientation.w = keyframe->T_odom_body().rotation.w();
  backend_keyframe_path.header = pose.header;
  backend_keyframe_path.poses.push_back(pose);
  if (keyframe->id() == 0 ||
      (keyframe->id() + 1) % backend_path_publish_interval == 0)
    pubBackendKeyframePath->publish(backend_keyframe_path);

  if (pose_graph_optimizer)
  {
    const auto update = pose_graph_optimizer->AddKeyframe(keyframe);
    if (!update.solution_usable)
      RCLCPP_ERROR(
          node_->get_logger(),
          "Backend optimization for keyframe %lu was unusable; previous "
          "estimates were restored.",
          static_cast<unsigned long>(keyframe->id()));

    if (!global_pose_layer)
      throw std::logic_error("Pose graph requires the global pose layer.");
    global_pose_layer->AppendKeyframe(keyframe);
    handleRtkForKeyframe(keyframe);
    publishBackendOptimizedProducts(true, "periodic");

    RCLCPP_INFO(
        node_->get_logger(),
        "GTSAM iSAM2 KF %lu: nodes=%lu, factors=%lu, ran=%d, "
        "usable=%d, updates=%d, cost=%.3e->%.3e, time=%.3f ms, "
        "relinearized=%lu, reeliminated=%lu",
        static_cast<unsigned long>(keyframe->id()),
        static_cast<unsigned long>(
            pose_graph_optimizer->statistics().nodes),
        static_cast<unsigned long>(
            pose_graph_optimizer->statistics().odometry_factors),
        static_cast<int>(update.optimization_ran),
        static_cast<int>(update.solution_usable), update.iterations,
        update.initial_cost, update.final_cost,
        update.optimization_time_ms,
        static_cast<unsigned long>(update.variables_relinearized),
        static_cast<unsigned long>(update.variables_reeliminated));
  }

  if (loop_candidate_detector)
  {
    const auto detection = loop_candidate_detector->AddKeyframe(keyframe);
    if (detection.checked)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "Loop search KF %lu: history=%zu, eligible=%zu, nearby=%zu, "
          "selected=%zu (candidate-only; no graph factor added)",
          static_cast<unsigned long>(detection.current_id),
          detection.history_keyframes, detection.eligible_history,
          detection.nearby_history, detection.candidates.size());
    }
    if (!detection.candidates.empty())
    {
      visualization_msgs::MarkerArray marker_array;
      const auto stored_keyframes = keyframe_manager->keyframes();
      for (const auto &candidate : detection.candidates)
      {
        const auto &candidate_keyframe =
            stored_keyframes.at(candidate.candidate_id);
        visualization_msgs::Marker marker;
        marker.header.frame_id = backend_frontend_frame_id;
        marker.header.stamp = pose.header.stamp;
        marker.ns = "backend_loop_candidates";
        marker.id = static_cast<int>(backend_loop_marker_id++);
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.r = 1.0F;
        marker.color.g = 0.82F;
        marker.color.b = 0.05F;
        marker.color.a = 0.95F;
        geometry_msgs::msg::Point historical_point;
        const auto historical_pose = candidate_keyframe->T_slam_body();
        historical_point.x = historical_pose.translation.x();
        historical_point.y = historical_pose.translation.y();
        historical_point.z = historical_pose.translation.z();
        geometry_msgs::msg::Point current_point;
        const auto current_pose = keyframe->T_slam_body();
        current_point.x = current_pose.translation.x();
        current_point.y = current_pose.translation.y();
        current_point.z = current_pose.translation.z();
        marker.points.push_back(historical_point);
        marker.points.push_back(current_point);
        marker_array.markers.push_back(marker);

        if (loop_registration &&
            !loop_registration->Enqueue(candidate, stored_keyframes))
          RCLCPP_ERROR(
              node_->get_logger(),
              "Failed to enqueue loop NDT candidate %lu<-%lu; queue is "
              "full or the snapshot is invalid.",
              static_cast<unsigned long>(candidate.candidate_id),
              static_cast<unsigned long>(candidate.current_id));
      }
      pubBackendLoopCandidates->publish(marker_array);
    }
  }

  if (backend_publish_keyframe_cloud)
  {
    my_livo::backend::KeyframeCloud cloud_odom;
    cloud_odom.reserve(keyframe->cloud_body()->size());
    for (const auto &point_body : keyframe->cloud_body()->points)
    {
      const V3D point_odom = keyframe->T_odom_body() *
          V3D(point_body.x, point_body.y, point_body.z);
      my_livo::backend::KeyframePoint point;
      point.x = static_cast<float>(point_odom.x());
      point.y = static_cast<float>(point_odom.y());
      point.z = static_cast<float>(point_odom.z());
      point.intensity = point_body.intensity;
      cloud_odom.push_back(point);
    }
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud_odom, message);
    message.header = pose.header;
    pubBackendKeyframeCloud->publish(message);
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "Backend keyframe %lu at %.9f: trigger=%s, body_points=%zu, "
      "T_odom_body=[%.3f, %.3f, %.3f]",
      static_cast<unsigned long>(keyframe->id()), keyframe->timestamp(),
      my_livo::backend::KeyframeManager::TriggerMaskToString(
          keyframe->trigger_mask()).c_str(),
      keyframe->cloud_body()->size(),
      keyframe->T_odom_body().translation.x(),
      keyframe->T_odom_body().translation.y(),
      keyframe->T_odom_body().translation.z());
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
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

void LIVMapper::run() 
{
  rclcpp::Rate rate(1000.0);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    if (!processImu())
    {
      rate.sleep();
      continue;
    }

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  savePCD();
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
    // A malformed cloud is a data-quality failure, not a process failure.
    // Keep all timing checks strict and discard the offending synchronized
    // set; the next complete set can still be deskewed with real IMU data.
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping invalid Multi-LiDAR frame set: %s", error.what());
    lock.unlock();
    sig_buffer.notify_all();
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

    lid_raw_data_buffer.push_back(merged);
    lid_header_time_buffer.push_back(merged_header_time);
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

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstSharedPtr &msg)
{
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
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping invalid LiDAR frame: timestamp must be finite and at "
        "least two valid time-tagged points must remain");
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }
  if (cur_head_time <= last_timestamp_lidar)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping non-increasing LiDAR timestamp %.9f (latest %.9f)",
        cur_head_time, last_timestamp_lidar);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
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
  if (!std::isfinite(cur_head_time) ||
      cur_head_time <= last_timestamp_lidar)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping invalid or non-increasing Livox timestamp %.9f "
        "(latest %.9f)", cur_head_time, last_timestamp_lidar);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
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

void LIVMapper::ins_status_cbk(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &msg_in)
{
  if (!rtk_observation_buffer) return;
  const double timestamp =
      stampToSec(msg_in->header.stamp) - imu_time_offset;
  const auto mode = my_livo::backend::ParseInsPosMode(*msg_in);
  if (!rtk_observation_buffer->AddStatus(timestamp, mode))
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Ignoring invalid or out-of-order CGI-610 INS status sample");
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
  const V3D raw_rear_axle_position = mine_pose.position;

  if (rear_axle_to_imu_enabled)
  {
    // The new Odometry preserves the same st_point_3d/f_pos_alt translation
    // carried by the legacy packed INS pose. When the estimator body is moved
    // from the rear axle to the physical IMU, move this reference pose by the
    // same lever arm so LIO and RTK/INS remain at the same physical point.
    mine_pose.position += mine_from_rear_axle * (-imu_to_rear_axle);
  }

  if (rtk_observation_buffer)
  {
    my_livo::backend::RtkSolution solution;
    solution.timestamp = mine_pose.stamp;
    solution.position = mine_pose.position;
    solution.raw_position = raw_rear_axle_position;
    solution.lever_arm_correction =
        mine_pose.position - raw_rear_axle_position;
    solution.has_raw_position = true;
    solution.orientation = mine_pose.orientation;
    const V3D receiver_velocity_body(
        msg_in->twist.twist.linear.x,
        msg_in->twist.twist.linear.y,
        msg_in->twist.twist.linear.z);
    // nav_msgs/Odometry defines twist in child_frame_id. Rotate the receiver
    // navigation velocity into mine/world. If the reference point is shifted
    // from rear axle to IMU, differentiate the high-rate INS attitude and add
    // omega x lever-arm so position and velocity describe the same point.
    bool receiver_velocity_valid = receiver_velocity_body.allFinite();
    V3D receiver_velocity_mine =
        mine_from_rear_axle * receiver_velocity_body;
    if (receiver_velocity_valid && rear_axle_to_imu_enabled)
    {
      receiver_velocity_valid = !mine_pose_samples.empty();
      if (receiver_velocity_valid)
      {
        const MinePose &previous_pose = mine_pose_samples.back();
        const double interval_sec = mine_pose.stamp - previous_pose.stamp;
        receiver_velocity_valid = std::isfinite(interval_sec) &&
            interval_sec > 1.0e-6 && interval_sec <= 0.05;
        if (receiver_velocity_valid)
        {
          const M3D previous_rotation =
              previous_pose.orientation.toRotationMatrix();
          const Eigen::AngleAxisd relative_rotation(
              previous_rotation.transpose() * mine_from_rear_axle);
          const V3D angular_velocity_body =
              relative_rotation.axis() *
              (relative_rotation.angle() / interval_sec);
          receiver_velocity_valid = angular_velocity_body.allFinite();
          if (receiver_velocity_valid)
          {
            const V3D rear_to_imu = -imu_to_rear_axle;
            receiver_velocity_mine += mine_from_rear_axle *
                angular_velocity_body.cross(rear_to_imu);
          }
        }
      }
    }
    if (receiver_velocity_valid)
    {
      solution.has_velocity = true;
      solution.velocity = receiver_velocity_mine;
      M3D receiver_velocity_covariance_body = M3D::Zero();
      for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
          receiver_velocity_covariance_body(row, column) =
              msg_in->twist.covariance[row * 6 + column];
      solution.velocity_covariance = mine_from_rear_axle *
          receiver_velocity_covariance_body *
          mine_from_rear_axle.transpose();
    }
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        solution.position_covariance(row, column) =
            msg_in->pose.covariance[row * 6 + column];
    if (!rtk_observation_buffer->AddSolution(solution))
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "Ignoring invalid or out-of-order CGI-610 RTK solution");
  }

  if (!mine_pose_samples.empty() &&
      mine_pose.stamp < mine_pose_samples.back().stamp)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Ignoring out-of-order INS odometry sample");
    return;
  }
  // Pose remains limited to the one-time mine-frame anchor, RTK buffer and
  // RViz reference path. Only its independently status-gated twist may enter
  // the low-rate, acceleration-bounded velocity safety controller.
  mine_pose_samples.push_back(mine_pose);
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  auto msg = std::make_shared<sensor_msgs::Imu>(*msg_in);
  double timestamp = stampToSec(msg->header.stamp) - imu_time_offset;

  if (!std::isfinite(timestamp) ||
      !std::isfinite(msg_in->angular_velocity.x) ||
      !std::isfinite(msg_in->angular_velocity.y) ||
      !std::isfinite(msg_in->angular_velocity.z) ||
      !std::isfinite(msg_in->linear_acceleration.x) ||
      !std::isfinite(msg_in->linear_acceleration.y) ||
      !std::isfinite(msg_in->linear_acceleration.z))
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping IMU sample with a non-finite timestamp or measurement");
    return;
  }

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

  if (last_timestamp_imu > 0.0 && timestamp <= last_timestamp_imu)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Dropping non-increasing IMU timestamp %.9f (latest %.9f, "
        "delta %.9f s)",
        timestamp, last_timestamp_imu, timestamp - last_timestamp_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
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
      // gap.
      RCLCPP_WARN(
          node_->get_logger(),
          "Startup IMU delivery has a %.6f s gap (limit %.6f s) between "
          "%.9f and %.9f; restarting IMU initialization from the newer "
          "sample",
          gap, imu_max_time_gap, last_timestamp_imu, timestamp);
      imu_buffer.clear();
      lid_raw_data_buffer.clear();
      lid_header_time_buffer.clear();
      for (auto &source : lidar_sources) source->pending_frames.clear();
      lidar_pushed = false;
      LidarMeasures = LidarMeasureGroup();
      LidarMeasures.last_lio_update_time = timestamp;
      minimum_lidar_time_after_imu_gap = timestamp;
      mine_pose_samples.clear();
      p_imu->Reset();
    }
    else
    {
      // A recorded sensor segment can have a short hole between MCAP files.
      // Never interpolate a fictitious IMU interval through that hole.  Drop
      // every not-yet-processed LiDAR envelope and restart propagation at the
      // first real IMU sample of the new segment.  A bounded short gap uses a
      // constant-velocity bridge; a longer gap deliberately holds the pose
      // instead of inventing motion over an unsupported interval.
      ++imu_gap_recovery_count;
      const bool bridge_motion = gap <= imu_max_recoverable_gap;
      const double bounded_gap =
          std::min(gap, imu_max_recoverable_gap);
      const double position_sigma = bounded_gap * bounded_gap;
      const double velocity_sigma = 2.0 * bounded_gap;
      const double rotation_sigma =
          bounded_gap * 10.0 * M_PI / 180.0;
      if (bridge_motion) _state.pos_end += _state.vel_end * gap;
      _state.cov.block<3, 3>(0, 0).diagonal().array() +=
          rotation_sigma * rotation_sigma;
      _state.cov.block<3, 3>(3, 3).diagonal().array() +=
          position_sigma * position_sigma;
      _state.cov.block<3, 3>(7, 7).diagonal().array() +=
          velocity_sigma * velocity_sigma;

      imu_buffer.clear();
      lid_raw_data_buffer.clear();
      lid_header_time_buffer.clear();
      for (auto &source : lidar_sources) source->pending_frames.clear();
      lidar_pushed = false;
      LidarMeasures = LidarMeasureGroup();
      LidarMeasures.last_lio_update_time = timestamp;
      minimum_lidar_time_after_imu_gap = timestamp;
      p_imu->Reset(timestamp, msg);

      {
        std::lock_guard<std::mutex> propagation_lock(mtx_buffer_imu_prop);
        prop_imu_buffer.clear();
        imu_propagate = _state;
        latest_ekf_state = _state;
        latest_ekf_time = timestamp;
        newest_imu = *msg;
        new_imu = false;
        state_update_flg = true;
      }
      RCLCPP_WARN(
          node_->get_logger(),
          "Skipped IMU gap #%llu: %.6f s between %.9f and %.9f; discarded "
          "pending LiDAR envelopes and rebased propagation without IMU "
          "interpolation (motion_bridge=%d)",
          static_cast<unsigned long long>(imu_gap_recovery_count), gap,
          last_timestamp_imu, timestamp, static_cast<int>(bridge_motion));
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
  {
    // Keep the newest accepted sample even when high-rate IMU odometry is
    // disabled. It is also the real sample used to rebase Process2 after a
    // rejected LiDAR/IMU group.
    std::lock_guard<std::mutex> propagation_lock(mtx_buffer_imu_prop);
    newest_imu = *msg;
    if (imu_prop_enable)
    {
      if (!p_imu->imu_need_init) prop_imu_buffer.push_back(*msg);
      new_imu = true;
    }
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
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = stampToSec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  ROS_INFO("Get image, its header time: %.6f", msg_header_time);
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
    // A scan that starts before the first real IMU sample after a detected
    // gap has no valid deskew envelope. Drop the whole scan; retaining only
    // its tail would mix points from two propagation epochs.
    while (!lid_raw_data_buffer.empty() &&
           !lid_header_time_buffer.empty() &&
           minimum_lidar_time_after_imu_gap >= 0.0 &&
           lid_header_time_buffer.front() + 1.0e-9 <
               minimum_lidar_time_after_imu_gap)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "Dropping LiDAR scan %.9f before recovered IMU epoch %.9f",
          lid_header_time_buffer.front(),
          minimum_lidar_time_after_imu_gap);
      lid_raw_data_buffer.pop_front();
      lid_header_time_buffer.pop_front();
      lidar_pushed = false;
    }
    if (lid_raw_data_buffer.empty() || lid_header_time_buffer.empty())
      return false;

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

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudFullRes,
                                    VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
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
  laserCloudmsg.header.frame_id = "mine";
  pubLaserCloudFullRes->publish(laserCloudmsg);

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
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

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
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
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
    laserCloudmsg.header.frame_id = "mine";
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
  laserCloudFullRes3.header.frame_id = "mine";
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
  odomAftMapped.header.frame_id = "mine";
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
  msg_body_pose.header.frame_id = "mine";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::Path>::SharedPtr &pubPath)
{
  // Output-only frontend history for visualization. Backend keyframes sample
  // _state independently; no optimizer reads this nav_msgs::Path container.
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = stampFromSec(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = "mine";
  path.header.stamp = msg_body_pose.header.stamp;
  path.header.frame_id = "mine";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
