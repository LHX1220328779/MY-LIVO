#include "backend/rtk_fusion_diagnostics.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace my_livo::backend
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

void OpenCsv(const std::string &path, std::ofstream *stream,
             const std::string &header)
{
  if (path.empty()) return;
  const std::filesystem::path csv_path(path);
  if (csv_path.has_parent_path())
    std::filesystem::create_directories(csv_path.parent_path());
  stream->open(csv_path, std::ios::out | std::ios::trunc);
  if (!stream->is_open())
    throw std::runtime_error("Cannot open RTK diagnostics CSV: " +
                             csv_path.string());
  *stream << header << '\n';
}

std::string Number(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

template <typename Integer>
std::string IntegerNumber(Integer value)
{
  return std::to_string(value);
}

double YawDegrees(const Eigen::Quaterniond &quaternion)
{
  const Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0)) * 180.0 / kPi;
}

double WrappedDegrees(double angle)
{
  return std::remainder(angle, 360.0);
}

Eigen::Vector3d RollPitchYawDegrees(
    const Eigen::Quaterniond &quaternion)
{
  const Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw) * 180.0 / kPi;
}

void WriteFields(std::ofstream *stream,
                 const std::vector<std::string> &fields)
{
  for (std::size_t index = 0; index < fields.size(); ++index)
  {
    if (index != 0U) *stream << ',';
    *stream << fields[index];
  }
  *stream << '\n';
  stream->flush();
}
}  // namespace

RtkFusionDiagnostics::RtkFusionDiagnostics(const Options &options)
{
  OpenCsv(
      options.keyframe_csv_path, &keyframe_csv_stream_,
      "keyframe_id,timestamp,observation_available,query_reason,health,"
      "decision,factor_added,lower_timestamp,upper_timestamp,lower_dt,"
      "upper_dt,interpolation_gap,interpolation_alpha,lower_ins_pos_mode,"
      "upper_ins_pos_mode,rtk_raw_x,rtk_raw_y,rtk_raw_z,lever_correction_x,"
      "lever_correction_y,lever_correction_z,lever_correction_norm,rtk_x,"
      "rtk_y,rtk_z,reported_variance_x,reported_variance_y,"
      "reported_variance_z,effective_sigma_x,effective_sigma_y,"
      "effective_sigma_z,lio_x,lio_y,lio_z,lio_yaw_deg,backend_before_x,"
      "backend_before_y,backend_before_z,backend_before_yaw_deg,"
      "backend_after_x,backend_after_y,backend_after_z,"
      "backend_after_yaw_deg,rtk_yaw_deg,lio_minus_rtk_raw_x,"
      "lio_minus_rtk_raw_y,lio_minus_rtk_raw_z,lio_minus_rtk_raw_norm,"
      "lio_minus_rtk_x,lio_minus_rtk_y,lio_minus_rtk_z,"
      "lio_minus_rtk_norm,backend_before_minus_rtk_x,"
      "backend_before_minus_rtk_y,backend_before_minus_rtk_z,"
      "backend_before_minus_rtk_norm,backend_after_minus_rtk_x,"
      "backend_after_minus_rtk_y,backend_after_minus_rtk_z,"
      "backend_after_minus_rtk_norm,lio_minus_rtk_yaw_deg,"
      "backend_before_minus_rtk_yaw_deg,backend_after_minus_rtk_yaw_deg,"
      "graph_update_translation_m,graph_update_yaw_deg");
  OpenCsv(
      options.initial_alignment_csv_path, &initial_alignment_csv_stream_,
      "initialization_timestamp,first_sample_timestamp,last_sample_timestamp,"
      "sample_count,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg,"
      "position_rms_m,position_max_m,orientation_max_deg");
}

void RtkFusionDiagnostics::RecordInitialAlignment(
    const InitialAlignmentRecord &record)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initial_alignment_csv_stream_.is_open()) return;
  const Eigen::Vector3d euler =
      RollPitchYawDegrees(record.T_mine_lio_initial.rotation);
  initial_alignment_csv_stream_
      << std::setprecision(17) << record.initialization_timestamp << ','
      << record.first_sample_timestamp << ',' << record.last_sample_timestamp
      << ',' << record.sample_count << ','
      << record.T_mine_lio_initial.translation.x() << ','
      << record.T_mine_lio_initial.translation.y() << ','
      << record.T_mine_lio_initial.translation.z() << ','
      << record.T_mine_lio_initial.rotation.x() << ','
      << record.T_mine_lio_initial.rotation.y() << ','
      << record.T_mine_lio_initial.rotation.z() << ','
      << record.T_mine_lio_initial.rotation.w() << ',' << euler.x() << ','
      << euler.y() << ',' << YawDegrees(record.T_mine_lio_initial.rotation)
      << ',' << record.position_rms_m << ',' << record.position_max_m << ','
      << record.orientation_max_deg << '\n';
  initial_alignment_csv_stream_.flush();
}

void RtkFusionDiagnostics::RecordKeyframe(
    const Keyframe &keyframe, const RtkObservationQuery &query,
    const std::string &decision, bool factor_added,
    const Pose3d &T_backend_before, const Pose3d &T_backend_after)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!keyframe_csv_stream_.is_open()) return;

  // Keep this count synchronized with the CSV header. Empty strings represent
  // measurements that were unavailable because timestamp/health gates failed.
  std::vector<std::string> fields(65);
  fields[0] = IntegerNumber(keyframe.id());
  fields[1] = Number(keyframe.timestamp());
  fields[2] = IntegerNumber(query.observation.has_value() ? 1 : 0);
  fields[3] = RtkObservationRejectReasonToString(query.reason);
  fields[4] = Number(query.observation ? query.observation->health : 0.0);
  fields[5] = decision;
  fields[6] = IntegerNumber(factor_added ? 1 : 0);

  const Pose3d &T_lio = keyframe.T_odom_body();
  const double lio_yaw = YawDegrees(T_lio.rotation);
  const double backend_before_yaw = YawDegrees(T_backend_before.rotation);
  const double backend_after_yaw = YawDegrees(T_backend_after.rotation);
  fields[31] = Number(T_lio.translation.x());
  fields[32] = Number(T_lio.translation.y());
  fields[33] = Number(T_lio.translation.z());
  fields[34] = Number(lio_yaw);
  fields[35] = Number(T_backend_before.translation.x());
  fields[36] = Number(T_backend_before.translation.y());
  fields[37] = Number(T_backend_before.translation.z());
  fields[38] = Number(backend_before_yaw);
  fields[39] = Number(T_backend_after.translation.x());
  fields[40] = Number(T_backend_after.translation.y());
  fields[41] = Number(T_backend_after.translation.z());
  fields[42] = Number(backend_after_yaw);
  fields[63] = Number(
      (T_backend_after.translation - T_backend_before.translation).norm());
  fields[64] = Number(WrappedDegrees(
      backend_after_yaw - backend_before_yaw));

  if (query.observation)
  {
    const RtkObservation &observation = *query.observation;
    fields[7] = Number(observation.lower_timestamp);
    fields[8] = Number(observation.upper_timestamp);
    fields[9] = Number(keyframe.timestamp() - observation.lower_timestamp);
    fields[10] = Number(observation.upper_timestamp - keyframe.timestamp());
    fields[11] = Number(observation.upper_timestamp -
                        observation.lower_timestamp);
    fields[12] = Number(observation.interpolation_alpha);
    fields[13] = IntegerNumber(observation.lower_ins_pos_mode);
    fields[14] = IntegerNumber(observation.upper_ins_pos_mode);
    fields[15] = Number(observation.raw_position.x());
    fields[16] = Number(observation.raw_position.y());
    fields[17] = Number(observation.raw_position.z());
    fields[18] = Number(observation.lever_arm_correction.x());
    fields[19] = Number(observation.lever_arm_correction.y());
    fields[20] = Number(observation.lever_arm_correction.z());
    fields[21] = Number(observation.lever_arm_correction.norm());
    fields[22] = Number(observation.position.x());
    fields[23] = Number(observation.position.y());
    fields[24] = Number(observation.position.z());
    fields[25] = Number(observation.reported_position_covariance(0, 0));
    fields[26] = Number(observation.reported_position_covariance(1, 1));
    fields[27] = Number(observation.reported_position_covariance(2, 2));
    fields[28] = Number(std::sqrt(observation.position_covariance(0, 0)));
    fields[29] = Number(std::sqrt(observation.position_covariance(1, 1)));
    fields[30] = Number(std::sqrt(observation.position_covariance(2, 2)));
    const double rtk_yaw = YawDegrees(observation.orientation);
    fields[43] = Number(rtk_yaw);

    const auto fill_residual = [&fields](std::size_t first,
                                         const Eigen::Vector3d &residual) {
      fields[first] = Number(residual.x());
      fields[first + 1] = Number(residual.y());
      fields[first + 2] = Number(residual.z());
      fields[first + 3] = Number(residual.norm());
    };
    fill_residual(44, T_lio.translation - observation.raw_position);
    fill_residual(48, T_lio.translation - observation.position);
    fill_residual(52, T_backend_before.translation - observation.position);
    fill_residual(56, T_backend_after.translation - observation.position);
    fields[60] = Number(WrappedDegrees(lio_yaw - rtk_yaw));
    fields[61] = Number(WrappedDegrees(backend_before_yaw - rtk_yaw));
    fields[62] = Number(WrappedDegrees(backend_after_yaw - rtk_yaw));
  }

  WriteFields(&keyframe_csv_stream_, fields);
}

}  // namespace my_livo::backend
