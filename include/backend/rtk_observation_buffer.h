#ifndef MY_LIVO_BACKEND_RTK_OBSERVATION_BUFFER_H
#define MY_LIVO_BACKEND_RTK_OBSERVATION_BUFFER_H

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <optional>
#include <set>
#include <string>

namespace my_livo::backend
{

std::optional<int> ParseInsPosMode(
    const diagnostic_msgs::msg::DiagnosticArray &message);

struct RtkSolution
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Zero();
};

struct RtkObservation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Identity();
  double lower_timestamp = 0.0;
  double upper_timestamp = 0.0;
  double interpolation_alpha = 0.0;
};

enum class RtkObservationRejectReason
{
  kAccepted,
  kNoBracket,
  kInterpolationGap,
  kEndpointDistance,
  kStatusUnknown,
  kStatusNotEligible,
};

const char *RtkObservationRejectReasonToString(
    RtkObservationRejectReason reason);

struct RtkObservationQuery
{
  std::optional<RtkObservation> observation;
  RtkObservationRejectReason reason =
      RtkObservationRejectReason::kNoBracket;
};

class RtkObservationBuffer
{
public:
  struct Options
  {
    int required_ins_pos_mode = 4;
    double maximum_status_age_sec = 0.03;
    double maximum_interpolation_gap_sec = 0.05;
    double maximum_endpoint_distance_sec = 0.03;
    double retention_sec = 20.0;
    double configured_sigma_xy_m = 0.30;
    double configured_sigma_z_m = 0.50;
    double minimum_sigma_xy_m = 0.10;
    double minimum_sigma_z_m = 0.20;
    std::string status_csv_path;
    std::string solution_csv_path;
    std::string query_csv_path;
  };

  struct Statistics
  {
    std::uint64_t status_messages = 0;
    std::uint64_t status_parsed = 0;
    std::uint64_t solutions = 0;
    std::uint64_t out_of_order_rejected = 0;
    std::uint64_t queries = 0;
    std::uint64_t observations = 0;
    std::uint64_t rejected_no_bracket = 0;
    std::uint64_t rejected_gap = 0;
    std::uint64_t rejected_endpoint = 0;
    std::uint64_t rejected_status_unknown = 0;
    std::uint64_t rejected_status_mode = 0;
    std::size_t maximum_status_buffer_size = 0;
    std::size_t maximum_solution_buffer_size = 0;
  };

  explicit RtkObservationBuffer(const Options &options);

  bool AddStatus(double timestamp, std::optional<int> ins_pos_mode);
  bool AddSolution(const RtkSolution &solution);
  RtkObservationQuery GetObservationAt(double timestamp);
  Statistics statistics() const;

private:
  struct StatusSample
  {
    double timestamp = 0.0;
    std::optional<int> mode;
  };

  static void ValidateOptions(const Options &options);
  std::optional<int> ModeAtLocked(double timestamp) const;
  bool HasIneligibleStatusLocked(double begin, double end) const;
  Eigen::Matrix3d SanitizedCovariance(
      const Eigen::Matrix3d &reported) const;
  void PruneLocked(double newest_timestamp);
  void RecordQueryLocked(double timestamp,
                         const RtkObservationQuery &query);

  Options options_;
  mutable std::mutex mutex_;
  std::deque<StatusSample> status_buffer_;
  std::deque<RtkSolution> solution_buffer_;
  Statistics statistics_;
  std::ofstream status_csv_stream_;
  std::ofstream solution_csv_stream_;
  std::ofstream query_csv_stream_;
};

class RtkFactorSelector
{
public:
  struct Options
  {
    double minimum_time_interval_sec = 2.0;
    double maximum_time_interval_sec = 10.0;
    double minimum_translation_m = 5.0;
  };

  explicit RtkFactorSelector(const Options &options);
  bool ShouldSelect(std::uint64_t keyframe_id,
                    const RtkObservation &observation) const;
  void MarkSelected(std::uint64_t keyframe_id,
                    const RtkObservation &observation);
  std::size_t selected_count() const;

private:
  Options options_;
  mutable std::mutex mutex_;
  std::set<std::uint64_t> selected_keyframes_;
  std::optional<RtkObservation> last_observation_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_RTK_OBSERVATION_BUFFER_H
