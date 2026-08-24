#include "backend/rtk_observation_buffer.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace my_livo::backend
{
namespace
{
void OpenCsv(const std::string &path, std::ofstream *stream,
             const std::string &header)
{
  if (path.empty()) return;
  const std::filesystem::path csv_path(path);
  if (csv_path.has_parent_path())
    std::filesystem::create_directories(csv_path.parent_path());
  stream->open(csv_path, std::ios::out | std::ios::trunc);
  if (!stream->is_open())
    throw std::runtime_error("Cannot open RTK CSV: " + csv_path.string());
  *stream << header << '\n';
}

bool FinitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

std::optional<int> ParseInsPosMode(
    const diagnostic_msgs::msg::DiagnosticArray &message)
{
  std::optional<int> parsed;
  for (const auto &status : message.status)
  {
    for (const auto &value : status.values)
    {
      if (value.key != "ins_pos_mode") continue;
      const std::string &text = value.value;
      const auto first = text.find_first_not_of(" \t\r\n");
      if (first == std::string::npos) return std::nullopt;
      const auto last = text.find_last_not_of(" \t\r\n");
      const std::string trimmed = text.substr(first, last - first + 1);
      int mode = 0;
      const auto conversion = std::from_chars(
          trimmed.data(), trimmed.data() + trimmed.size(), mode);
      if (conversion.ec != std::errc() ||
          conversion.ptr != trimmed.data() + trimmed.size())
        return std::nullopt;
      if (parsed && *parsed != mode) return std::nullopt;
      parsed = mode;
    }
  }
  return parsed;
}

const char *RtkObservationRejectReasonToString(
    RtkObservationRejectReason reason)
{
  switch (reason)
  {
    case RtkObservationRejectReason::kAccepted: return "accepted";
    case RtkObservationRejectReason::kNoBracket: return "no_bracket";
    case RtkObservationRejectReason::kInterpolationGap:
      return "interpolation_gap";
    case RtkObservationRejectReason::kEndpointDistance:
      return "endpoint_distance";
    case RtkObservationRejectReason::kStatusUnknown: return "status_unknown";
    case RtkObservationRejectReason::kStatusNotEligible:
      return "status_not_eligible";
  }
  return "unknown";
}

RtkObservationBuffer::RtkObservationBuffer(const Options &options)
    : options_(options)
{
  ValidateOptions(options_);
  OpenCsv(options_.status_csv_path, &status_csv_stream_,
          "timestamp,parsed,ins_pos_mode");
  OpenCsv(options_.solution_csv_path, &solution_csv_stream_,
          "timestamp,x,y,z,qx,qy,qz,qw,sigma_x,sigma_y,sigma_z,"
          "raw_x,raw_y,raw_z,lever_correction_x,lever_correction_y,"
          "lever_correction_z,reported_variance_x,reported_variance_y,"
          "reported_variance_z,velocity_valid,velocity_x,velocity_y,"
          "velocity_z,velocity_sigma_x,velocity_sigma_y,velocity_sigma_z,"
          "reported_velocity_variance_x,reported_velocity_variance_y,"
          "reported_velocity_variance_z");
  OpenCsv(options_.query_csv_path, &query_csv_stream_,
          "timestamp,accepted,reason,lower_timestamp,upper_timestamp,alpha,"
          "x,y,z,sigma_x,sigma_y,sigma_z,raw_x,raw_y,raw_z,"
          "lever_correction_x,lever_correction_y,lever_correction_z,"
          "reported_variance_x,reported_variance_y,reported_variance_z,"
          "lower_ins_pos_mode,upper_ins_pos_mode,health,velocity_valid,"
          "velocity_x,velocity_y,velocity_z,velocity_sigma_x,"
          "velocity_sigma_y,velocity_sigma_z,"
          "reported_velocity_variance_x,reported_velocity_variance_y,"
          "reported_velocity_variance_z");
}

void RtkObservationBuffer::ValidateOptions(const Options &options)
{
  if (!FinitePositive(options.maximum_status_age_sec) ||
      !FinitePositive(options.maximum_interpolation_gap_sec) ||
      !FinitePositive(options.maximum_endpoint_distance_sec) ||
      !FinitePositive(options.retention_sec) ||
      !FinitePositive(options.configured_sigma_xy_m) ||
      !FinitePositive(options.configured_sigma_z_m) ||
      !FinitePositive(options.minimum_sigma_xy_m) ||
      !FinitePositive(options.minimum_sigma_z_m) ||
      !FinitePositive(options.configured_velocity_sigma_xy_mps) ||
      !FinitePositive(options.configured_velocity_sigma_z_mps) ||
      !FinitePositive(options.minimum_velocity_sigma_xy_mps) ||
      !FinitePositive(options.minimum_velocity_sigma_z_mps))
    throw std::invalid_argument("RTK buffer time/noise options must be positive.");
  if (options.maximum_endpoint_distance_sec > options.retention_sec ||
      options.maximum_interpolation_gap_sec > options.retention_sec)
    throw std::invalid_argument("RTK retention is shorter than a query gate.");
}

bool RtkObservationBuffer::AddStatus(
    double timestamp, std::optional<int> ins_pos_mode)
{
  if (!std::isfinite(timestamp)) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  ++statistics_.status_messages;
  if (ins_pos_mode) ++statistics_.status_parsed;
  if (!status_buffer_.empty() && timestamp < status_buffer_.back().timestamp)
  {
    ++statistics_.out_of_order_rejected;
    return false;
  }
  if (!status_buffer_.empty() && timestamp == status_buffer_.back().timestamp)
    status_buffer_.back().mode = ins_pos_mode;
  else
    status_buffer_.push_back({timestamp, ins_pos_mode});
  PruneLocked(timestamp);
  statistics_.maximum_status_buffer_size = std::max(
      statistics_.maximum_status_buffer_size, status_buffer_.size());
  if (status_csv_stream_.is_open())
    status_csv_stream_ << std::setprecision(17) << timestamp << ','
                       << static_cast<int>(ins_pos_mode.has_value()) << ','
                       << (ins_pos_mode ? std::to_string(*ins_pos_mode) : "")
                       << '\n';
  return true;
}

bool RtkObservationBuffer::AddSolution(const RtkSolution &solution)
{
  if (!std::isfinite(solution.timestamp) ||
      !solution.position.allFinite() ||
      !solution.orientation.coeffs().allFinite() ||
      solution.orientation.norm() < 1.0e-9)
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!solution_buffer_.empty() &&
      solution.timestamp < solution_buffer_.back().timestamp)
  {
    ++statistics_.out_of_order_rejected;
    return false;
  }
  RtkSolution normalized = solution;
  normalized.orientation.normalize();
  normalized.reported_position_covariance = solution.position_covariance;
  normalized.position_covariance =
      SanitizedCovariance(solution.position_covariance);
  if (normalized.has_velocity)
  {
    if (!normalized.velocity.allFinite()) return false;
    normalized.reported_velocity_covariance =
        solution.velocity_covariance;
    normalized.velocity_covariance =
        SanitizedVelocityCovariance(solution.velocity_covariance);
  }
  if (!normalized.has_raw_position)
  {
    normalized.raw_position = normalized.position;
    normalized.lever_arm_correction.setZero();
  }
  else if (!normalized.raw_position.allFinite() ||
           !normalized.lever_arm_correction.allFinite())
  {
    return false;
  }
  if (!solution_buffer_.empty() &&
      solution.timestamp == solution_buffer_.back().timestamp)
    solution_buffer_.back() = normalized;
  else
  {
    solution_buffer_.push_back(normalized);
    ++statistics_.solutions;
  }
  PruneLocked(solution.timestamp);
  statistics_.maximum_solution_buffer_size = std::max(
      statistics_.maximum_solution_buffer_size, solution_buffer_.size());
  if (solution_csv_stream_.is_open())
    solution_csv_stream_ << std::setprecision(17) << normalized.timestamp
        << ',' << normalized.position.x() << ',' << normalized.position.y()
        << ',' << normalized.position.z() << ',' << normalized.orientation.x()
        << ',' << normalized.orientation.y() << ',' << normalized.orientation.z()
        << ',' << normalized.orientation.w() << ','
        << std::sqrt(normalized.position_covariance(0, 0)) << ','
        << std::sqrt(normalized.position_covariance(1, 1)) << ','
        << std::sqrt(normalized.position_covariance(2, 2)) << ','
        << normalized.raw_position.x() << ','
        << normalized.raw_position.y() << ','
        << normalized.raw_position.z() << ','
        << normalized.lever_arm_correction.x() << ','
        << normalized.lever_arm_correction.y() << ','
        << normalized.lever_arm_correction.z() << ','
        << normalized.reported_position_covariance(0, 0) << ','
        << normalized.reported_position_covariance(1, 1) << ','
        << normalized.reported_position_covariance(2, 2) << ','
        << static_cast<int>(normalized.has_velocity) << ',';
  if (solution_csv_stream_.is_open())
  {
    if (normalized.has_velocity)
      solution_csv_stream_ << normalized.velocity.x() << ','
          << normalized.velocity.y() << ',' << normalized.velocity.z()
          << ',' << std::sqrt(normalized.velocity_covariance(0, 0)) << ','
          << std::sqrt(normalized.velocity_covariance(1, 1)) << ','
          << std::sqrt(normalized.velocity_covariance(2, 2)) << ','
          << normalized.reported_velocity_covariance(0, 0) << ','
          << normalized.reported_velocity_covariance(1, 1) << ','
          << normalized.reported_velocity_covariance(2, 2) << '\n';
    else
      solution_csv_stream_ << ",,,,,,,,\n";
  }
  return true;
}

std::optional<int> RtkObservationBuffer::ModeAtLocked(double timestamp) const
{
  if (status_buffer_.empty()) return std::nullopt;
  const auto upper = std::lower_bound(
      status_buffer_.begin(), status_buffer_.end(), timestamp,
      [](const StatusSample &sample, double time) {
        return sample.timestamp < time;
      });
  const StatusSample *nearest = nullptr;
  if (upper != status_buffer_.end()) nearest = &*upper;
  if (upper != status_buffer_.begin())
  {
    const auto before = std::prev(upper);
    if (!nearest || timestamp - before->timestamp <=
                        nearest->timestamp - timestamp)
      nearest = &*before;
  }
  if (!nearest ||
      std::abs(nearest->timestamp - timestamp) >
          options_.maximum_status_age_sec)
    return std::nullopt;
  return nearest->mode;
}

bool RtkObservationBuffer::HasIneligibleStatusLocked(
    double begin, double end) const
{
  const auto first = std::lower_bound(
      status_buffer_.begin(), status_buffer_.end(), begin,
      [](const StatusSample &sample, double time) {
        return sample.timestamp < time;
      });
  for (auto iterator = first;
       iterator != status_buffer_.end() && iterator->timestamp <= end;
       ++iterator)
    if (!iterator->mode ||
        *iterator->mode != options_.required_ins_pos_mode)
      return true;
  return false;
}

Eigen::Matrix3d RtkObservationBuffer::SanitizedCovariance(
    const Eigen::Matrix3d &reported) const
{
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  const double configured[3] = {
      options_.configured_sigma_xy_m,
      options_.configured_sigma_xy_m,
      options_.configured_sigma_z_m};
  const double floors[3] = {
      options_.minimum_sigma_xy_m,
      options_.minimum_sigma_xy_m,
      options_.minimum_sigma_z_m};
  for (int axis = 0; axis < 3; ++axis)
  {
    double sigma = configured[axis];
    const double variance = reported(axis, axis);
    if (std::isfinite(variance) && variance > 0.0)
      sigma = std::sqrt(variance);
    sigma = std::max(sigma, floors[axis]);
    covariance(axis, axis) = sigma * sigma;
  }
  return covariance;
}

Eigen::Matrix3d RtkObservationBuffer::SanitizedVelocityCovariance(
    const Eigen::Matrix3d &reported) const
{
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  const double configured[3] = {
      options_.configured_velocity_sigma_xy_mps,
      options_.configured_velocity_sigma_xy_mps,
      options_.configured_velocity_sigma_z_mps};
  const double floors[3] = {
      options_.minimum_velocity_sigma_xy_mps,
      options_.minimum_velocity_sigma_xy_mps,
      options_.minimum_velocity_sigma_z_mps};
  for (int axis = 0; axis < 3; ++axis)
  {
    double sigma = configured[axis];
    const double variance = reported(axis, axis);
    if (std::isfinite(variance) && variance > 0.0)
      sigma = std::sqrt(variance);
    sigma = std::max(sigma, floors[axis]);
    covariance(axis, axis) = sigma * sigma;
  }
  return covariance;
}

void RtkObservationBuffer::PruneLocked(double newest_timestamp)
{
  const double oldest = newest_timestamp - options_.retention_sec;
  while (status_buffer_.size() > 1 &&
         status_buffer_[1].timestamp < oldest)
    status_buffer_.pop_front();
  while (solution_buffer_.size() > 1 &&
         solution_buffer_[1].timestamp < oldest)
    solution_buffer_.pop_front();
}

RtkObservationQuery RtkObservationBuffer::GetObservationAt(double timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++statistics_.queries;
  RtkObservationQuery query;
  if (!std::isfinite(timestamp) || solution_buffer_.empty())
  {
    ++statistics_.rejected_no_bracket;
    RecordQueryLocked(timestamp, query);
    return query;
  }
  const auto upper = std::lower_bound(
      solution_buffer_.begin(), solution_buffer_.end(), timestamp,
      [](const RtkSolution &solution, double time) {
        return solution.timestamp < time;
      });
  const RtkSolution *lower_solution = nullptr;
  const RtkSolution *upper_solution = nullptr;
  if (upper != solution_buffer_.end() && upper->timestamp == timestamp)
    lower_solution = upper_solution = &*upper;
  else if (upper != solution_buffer_.begin() && upper != solution_buffer_.end())
  {
    lower_solution = &*std::prev(upper);
    upper_solution = &*upper;
  }
  if (!lower_solution || !upper_solution)
  {
    ++statistics_.rejected_no_bracket;
    RecordQueryLocked(timestamp, query);
    return query;
  }
  const double gap = upper_solution->timestamp - lower_solution->timestamp;
  if (gap > options_.maximum_interpolation_gap_sec)
  {
    query.reason = RtkObservationRejectReason::kInterpolationGap;
    ++statistics_.rejected_gap;
    RecordQueryLocked(timestamp, query);
    return query;
  }
  if (timestamp - lower_solution->timestamp >
          options_.maximum_endpoint_distance_sec ||
      upper_solution->timestamp - timestamp >
          options_.maximum_endpoint_distance_sec)
  {
    query.reason = RtkObservationRejectReason::kEndpointDistance;
    ++statistics_.rejected_endpoint;
    RecordQueryLocked(timestamp, query);
    return query;
  }
  const auto lower_mode = ModeAtLocked(lower_solution->timestamp);
  const auto upper_mode = ModeAtLocked(upper_solution->timestamp);
  if (!lower_mode || !upper_mode)
  {
    query.reason = RtkObservationRejectReason::kStatusUnknown;
    ++statistics_.rejected_status_unknown;
    RecordQueryLocked(timestamp, query);
    return query;
  }
  if (*lower_mode != options_.required_ins_pos_mode ||
      *upper_mode != options_.required_ins_pos_mode ||
      HasIneligibleStatusLocked(lower_solution->timestamp,
                                upper_solution->timestamp))
  {
    query.reason = RtkObservationRejectReason::kStatusNotEligible;
    ++statistics_.rejected_status_mode;
    RecordQueryLocked(timestamp, query);
    return query;
  }

  const double alpha = gap > 0.0 ?
      (timestamp - lower_solution->timestamp) / gap : 0.0;
  RtkObservation observation;
  observation.timestamp = timestamp;
  observation.lower_timestamp = lower_solution->timestamp;
  observation.upper_timestamp = upper_solution->timestamp;
  observation.interpolation_alpha = alpha;
  observation.position = (1.0 - alpha) * lower_solution->position +
                         alpha * upper_solution->position;
  observation.raw_position =
      (1.0 - alpha) * lower_solution->raw_position +
      alpha * upper_solution->raw_position;
  observation.lever_arm_correction =
      (1.0 - alpha) * lower_solution->lever_arm_correction +
      alpha * upper_solution->lever_arm_correction;
  observation.orientation = lower_solution->orientation.slerp(
      alpha, upper_solution->orientation).normalized();
  observation.has_velocity = lower_solution->has_velocity &&
      upper_solution->has_velocity;
  if (observation.has_velocity)
  {
    observation.velocity =
        (1.0 - alpha) * lower_solution->velocity +
        alpha * upper_solution->velocity;
    observation.reported_velocity_covariance =
        (1.0 - alpha) * lower_solution->reported_velocity_covariance +
        alpha * upper_solution->reported_velocity_covariance;
    observation.velocity_covariance =
        (1.0 - alpha) * lower_solution->velocity_covariance +
        alpha * upper_solution->velocity_covariance;
  }
  observation.reported_position_covariance =
      (1.0 - alpha) * lower_solution->reported_position_covariance +
      alpha * upper_solution->reported_position_covariance;
  observation.position_covariance =
      (1.0 - alpha) * lower_solution->position_covariance +
      alpha * upper_solution->position_covariance;
  observation.lower_ins_pos_mode = *lower_mode;
  observation.upper_ins_pos_mode = *upper_mode;
  observation.health = 1.0;
  query.observation = observation;
  query.reason = RtkObservationRejectReason::kAccepted;
  ++statistics_.observations;
  RecordQueryLocked(timestamp, query);
  return query;
}

void RtkObservationBuffer::RecordQueryLocked(
    double timestamp, const RtkObservationQuery &query)
{
  if (!query_csv_stream_.is_open()) return;
  query_csv_stream_ << std::setprecision(17) << timestamp << ','
                    << static_cast<int>(query.observation.has_value()) << ','
                    << RtkObservationRejectReasonToString(query.reason);
  if (!query.observation)
  {
    // 31 fields follow reason in the current schema.
    query_csv_stream_ << std::string(31, ',') << '\n';
    // Queries are produced at keyframe rate and are part of the validation
    // contract: every keyframe must have one durable query record.  Do not
    // leave the tail in the C++ stream buffer, because ROS launch may stop the
    // process without running all destructors.
    query_csv_stream_.flush();
    return;
  }
  const auto &observation = *query.observation;
  query_csv_stream_ << ',' << observation.lower_timestamp << ','
                    << observation.upper_timestamp << ','
                    << observation.interpolation_alpha << ','
                    << observation.position.x() << ','
                    << observation.position.y() << ','
                    << observation.position.z() << ','
                    << std::sqrt(observation.position_covariance(0, 0)) << ','
                    << std::sqrt(observation.position_covariance(1, 1)) << ','
                    << std::sqrt(observation.position_covariance(2, 2)) << ','
                    << observation.raw_position.x() << ','
                    << observation.raw_position.y() << ','
                    << observation.raw_position.z() << ','
                    << observation.lever_arm_correction.x() << ','
                    << observation.lever_arm_correction.y() << ','
                    << observation.lever_arm_correction.z() << ','
                    << observation.reported_position_covariance(0, 0) << ','
                    << observation.reported_position_covariance(1, 1) << ','
                    << observation.reported_position_covariance(2, 2) << ','
                    << observation.lower_ins_pos_mode << ','
                    << observation.upper_ins_pos_mode << ','
                    << observation.health << ','
                    << static_cast<int>(observation.has_velocity) << ',';
  if (observation.has_velocity)
    query_csv_stream_ << observation.velocity.x() << ','
                      << observation.velocity.y() << ','
                      << observation.velocity.z() << ','
                      << std::sqrt(
                             observation.velocity_covariance(0, 0)) << ','
                      << std::sqrt(
                             observation.velocity_covariance(1, 1)) << ','
                      << std::sqrt(
                             observation.velocity_covariance(2, 2)) << ','
                      << observation.reported_velocity_covariance(0, 0)
                      << ','
                      << observation.reported_velocity_covariance(1, 1)
                      << ','
                      << observation.reported_velocity_covariance(2, 2)
                      << '\n';
  else
    query_csv_stream_ << ",,,,,,,,\n";
  query_csv_stream_.flush();
}

RtkObservationBuffer::Statistics RtkObservationBuffer::statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return statistics_;
}

RtkFactorSelector::RtkFactorSelector(const Options &options)
    : options_(options)
{
  if (!FinitePositive(options_.minimum_time_interval_sec) ||
      !FinitePositive(options_.maximum_time_interval_sec) ||
      !FinitePositive(options_.minimum_translation_m) ||
      options_.maximum_time_interval_sec < options_.minimum_time_interval_sec)
    throw std::invalid_argument("RTK factor selector options are invalid.");
}

bool RtkFactorSelector::ShouldSelect(
    std::uint64_t keyframe_id, const RtkObservation &observation) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (selected_keyframes_.count(keyframe_id) != 0U) return false;
  if (!last_observation_) return true;
  const double elapsed = observation.timestamp - last_observation_->timestamp;
  if (elapsed < options_.minimum_time_interval_sec) return false;
  return elapsed >= options_.maximum_time_interval_sec ||
         (observation.position - last_observation_->position).norm() >=
             options_.minimum_translation_m;
}

void RtkFactorSelector::MarkSelected(
    std::uint64_t keyframe_id, const RtkObservation &observation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!selected_keyframes_.insert(keyframe_id).second)
    throw std::logic_error("RTK keyframe was selected more than once.");
  last_observation_ = observation;
}

std::size_t RtkFactorSelector::selected_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return selected_keyframes_.size();
}

}  // namespace my_livo::backend
