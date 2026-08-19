#include "backend/loop_verifier.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace my_livo::backend
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::uint64_t IdDistance(std::uint64_t left, std::uint64_t right)
{
  return left > right ? left - right : right - left;
}
}  // namespace

const char *LoopRejectReasonToString(LoopRejectReason reason)
{
  switch (reason)
  {
    case LoopRejectReason::kNone:
      return "none";
    case LoopRejectReason::kRegistrationFailed:
      return "registration_failed";
    case LoopRejectReason::kNotConverged:
      return "not_converged";
    case LoopRejectReason::kInsufficientConvergedLevels:
      return "insufficient_converged_levels";
    case LoopRejectReason::kProbabilityTooLow:
      return "probability_too_low";
    case LoopRejectReason::kFitnessTooHigh:
      return "fitness_too_high";
    case LoopRejectReason::kOverlapTooLow:
      return "overlap_too_low";
    case LoopRejectReason::kOverlapRmseTooHigh:
      return "overlap_rmse_too_high";
    case LoopRejectReason::kTranslationSanity:
      return "translation_sanity";
    case LoopRejectReason::kRotationSanity:
      return "rotation_sanity";
    case LoopRejectReason::kNoConsistentNeighbor:
      return "no_consistent_neighbor";
  }
  return "unknown";
}

LoopVerifier::LoopVerifier(const Options &options) : options_(options)
{
  ValidateOptions(options_);
  if (!options_.csv_path.empty())
  {
    const std::filesystem::path csv_path(options_.csv_path);
    if (csv_path.has_parent_path())
      std::filesystem::create_directories(csv_path.parent_path());
    csv_stream_.open(csv_path, std::ios::out | std::ios::trunc);
    if (!csv_stream_.is_open())
      throw std::runtime_error("Cannot open loop-verification CSV: " +
                               csv_path.string());
    csv_stream_
        << "current_id,candidate_id,accepted,reject_reason,"
           "consistent_neighbors,best_neighbor_translation_error_m,"
           "best_neighbor_rotation_error_deg,converged,converged_levels,"
           "transformation_probability,fitness_score_m2,overlap,"
           "overlap_rmse_m,correction_translation_m,"
           "correction_angle_deg,measurement_tx,measurement_ty,"
           "measurement_tz,measurement_qx,measurement_qy,measurement_qz,"
           "measurement_qw\n";
  }
}

void LoopVerifier::ValidateOptions(const Options &options)
{
  const auto finite_nonnegative = [](double value, const char *name) {
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument(std::string(name) +
                                  " must be finite and non-negative.");
  };
  if (options.minimum_converged_levels == 0U)
    throw std::invalid_argument(
        "Loop verification requires at least one converged level.");
  finite_nonnegative(options.minimum_transformation_probability,
                     "Loop probability threshold");
  finite_nonnegative(options.maximum_fitness_score_m2,
                     "Loop fitness threshold");
  if (!std::isfinite(options.minimum_overlap) ||
      options.minimum_overlap < 0.0 || options.minimum_overlap > 1.0)
    throw std::invalid_argument(
        "Loop minimum overlap must be in [0, 1].");
  finite_nonnegative(options.maximum_overlap_rmse_m,
                     "Loop overlap RMSE threshold");
  finite_nonnegative(options.maximum_translation_correction_m,
                     "Loop translation sanity threshold");
  finite_nonnegative(options.maximum_rotation_correction_deg,
                     "Loop rotation sanity threshold");
  if (options.maximum_rotation_correction_deg > 180.0)
    throw std::invalid_argument(
        "Loop rotation sanity threshold must not exceed 180 degrees.");
  if (options.neighbor_current_id_window == 0U ||
      options.neighbor_candidate_id_window == 0U)
    throw std::invalid_argument(
        "Loop neighbor ID windows must be positive.");
  finite_nonnegative(options.maximum_neighbor_translation_error_m,
                     "Loop neighbor translation threshold");
  finite_nonnegative(options.maximum_neighbor_rotation_error_deg,
                     "Loop neighbor rotation threshold");
  if (options.minimum_consistent_neighbors == 0U)
    throw std::invalid_argument(
        "Loop verification requires at least one consistent neighbor.");
}

LoopRejectReason LoopVerifier::IndividualRejectReason(
    const LoopRegistrationResult &registration) const
{
  if (registration.status != LoopRegistrationStatus::kCompleted)
    return LoopRejectReason::kRegistrationFailed;
  if (options_.require_final_convergence && !registration.converged)
    return LoopRejectReason::kNotConverged;
  if (registration.converged_levels < options_.minimum_converged_levels)
    return LoopRejectReason::kInsufficientConvergedLevels;
  if (registration.transformation_probability <
      options_.minimum_transformation_probability)
    return LoopRejectReason::kProbabilityTooLow;
  if (registration.fitness_score_m2 > options_.maximum_fitness_score_m2)
    return LoopRejectReason::kFitnessTooHigh;
  if (registration.overlap < options_.minimum_overlap)
    return LoopRejectReason::kOverlapTooLow;
  if (registration.overlap_rmse_m > options_.maximum_overlap_rmse_m)
    return LoopRejectReason::kOverlapRmseTooHigh;
  if (registration.correction_translation_m >
      options_.maximum_translation_correction_m)
    return LoopRejectReason::kTranslationSanity;
  if (registration.correction_angle_deg >
      options_.maximum_rotation_correction_deg)
    return LoopRejectReason::kRotationSanity;
  return LoopRejectReason::kNone;
}

LoopVerifier::Consistency LoopVerifier::CheckConsistency(
    const LoopRegistrationResult &reference,
    const LoopRegistrationResult &query) const
{
  Consistency result;
  const std::uint64_t current_gap = IdDistance(
      reference.candidate.current_id, query.candidate.current_id);
  const std::uint64_t candidate_gap = IdDistance(
      reference.candidate.candidate_id, query.candidate.candidate_id);
  if (current_gap == 0U ||
      current_gap > options_.neighbor_current_id_window ||
      candidate_gap > options_.neighbor_candidate_id_window)
    return result;
  result.is_neighbor = true;

  // Propagate the reference loop through the short odometry segments on both
  // sides of the loop. For Z_hc = inverse(H) * C:
  // Z_query_pred = inverse(H_query) * H_ref * Z_ref *
  //                inverse(C_ref) * C_query.
  const Pose3d predicted_query =
      query.T_map_candidate_snapshot.inverse() *
      reference.T_map_candidate_snapshot *
      reference.T_candidate_current *
      reference.T_map_current_snapshot.inverse() *
      query.T_map_current_snapshot;
  const Pose3d error =
      predicted_query.inverse() * query.T_candidate_current;
  result.translation_error_m = error.translation.norm();
  result.rotation_error_deg =
      predicted_query.rotation.angularDistance(
          query.T_candidate_current.rotation) *
      180.0 / kPi;
  result.consistent =
      result.translation_error_m <=
          options_.maximum_neighbor_translation_error_m &&
      result.rotation_error_deg <=
          options_.maximum_neighbor_rotation_error_deg;
  return result;
}

LoopVerificationDecision LoopVerifier::MakeRejected(
    const LoopRegistrationResult &registration,
    LoopRejectReason reason) const
{
  LoopVerificationDecision decision;
  decision.registration = registration;
  decision.accepted = false;
  decision.reject_reason = reason;
  return decision;
}

LoopVerificationDecision LoopVerifier::MakeAccepted(
    const LoopRegistrationResult &registration,
    std::size_t consistent_neighbors,
    double best_translation_error_m,
    double best_rotation_error_deg) const
{
  LoopVerificationDecision decision;
  decision.registration = registration;
  decision.accepted = true;
  decision.reject_reason = LoopRejectReason::kNone;
  decision.consistent_neighbors = consistent_neighbors;
  decision.best_neighbor_translation_error_m = best_translation_error_m;
  decision.best_neighbor_rotation_error_deg = best_rotation_error_deg;
  return decision;
}

std::vector<LoopVerificationDecision> LoopVerifier::Add(
    const LoopRegistrationResult &registration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++statistics_.registrations;
  std::vector<LoopVerificationDecision> decisions;

  // A future registration cannot support entries outside the configured
  // current-ID window. Resolve them before evaluating the new measurement.
  auto pending = pending_.begin();
  while (pending != pending_.end())
  {
    if (registration.candidate.current_id >
            pending->registration.candidate.current_id &&
        registration.candidate.current_id -
                pending->registration.candidate.current_id >
            options_.neighbor_current_id_window)
    {
      decisions.push_back(MakeRejected(
          pending->registration, LoopRejectReason::kNoConsistentNeighbor));
      pending = pending_.erase(pending);
    }
    else
    {
      ++pending;
    }
  }

  const LoopRejectReason individual_reason =
      IndividualRejectReason(registration);
  if (individual_reason != LoopRejectReason::kNone)
  {
    decisions.push_back(MakeRejected(registration, individual_reason));
  }
  else
  {
    ++statistics_.individually_valid;
    std::size_t support_count = 0;
    double best_translation = std::numeric_limits<double>::infinity();
    double best_rotation = std::numeric_limits<double>::infinity();
    for (const auto &accepted : accepted_history_)
    {
      const Consistency consistency =
          CheckConsistency(accepted, registration);
      if (!consistency.consistent) continue;
      ++support_count;
      best_translation =
          std::min(best_translation, consistency.translation_error_m);
      best_rotation =
          std::min(best_rotation, consistency.rotation_error_deg);
    }

    std::vector<std::size_t> consistent_pending;
    for (std::size_t index = 0; index < pending_.size(); ++index)
    {
      const Consistency consistency =
          CheckConsistency(pending_[index].registration, registration);
      if (!consistency.consistent) continue;
      consistent_pending.push_back(index);
      ++support_count;
      ++pending_[index].consistent_neighbors;
      if (pending_[index].consistent_neighbors == 1U)
      {
        pending_[index].best_translation_error_m =
            consistency.translation_error_m;
        pending_[index].best_rotation_error_deg =
            consistency.rotation_error_deg;
      }
      else
      {
        pending_[index].best_translation_error_m = std::min(
            pending_[index].best_translation_error_m,
            consistency.translation_error_m);
        pending_[index].best_rotation_error_deg = std::min(
            pending_[index].best_rotation_error_deg,
            consistency.rotation_error_deg);
      }
      best_translation =
          std::min(best_translation, consistency.translation_error_m);
      best_rotation =
          std::min(best_rotation, consistency.rotation_error_deg);
    }

    if (support_count >= options_.minimum_consistent_neighbors)
    {
      decisions.push_back(MakeAccepted(
          registration, support_count, best_translation, best_rotation));
      accepted_history_.push_back(registration);

    }
    else
    {
      PendingRegistration pending_registration;
      pending_registration.registration = registration;
      pending_registration.consistent_neighbors = support_count;
      if (support_count > 0U)
      {
        pending_registration.best_translation_error_m = best_translation;
        pending_registration.best_rotation_error_deg = best_rotation;
      }
      pending_.push_back(std::move(pending_registration));
    }

    // The new registration supplies one independent consistency observation
    // to every matching pending entry, whether or not the new registration has
    // already accumulated enough evidence for itself.
    for (auto index = consistent_pending.rbegin();
         index != consistent_pending.rend(); ++index)
    {
      PendingRegistration &pending_registration = pending_[*index];
      if (pending_registration.consistent_neighbors <
          options_.minimum_consistent_neighbors)
        continue;
      decisions.push_back(MakeAccepted(
          pending_registration.registration,
          pending_registration.consistent_neighbors,
          pending_registration.best_translation_error_m,
          pending_registration.best_rotation_error_deg));
      accepted_history_.push_back(pending_registration.registration);
      pending_.erase(pending_.begin() + *index);
    }
  }

  // Only recent accepted loops can be neighbors of future registrations.
  accepted_history_.erase(
      std::remove_if(
          accepted_history_.begin(), accepted_history_.end(),
          [this, &registration](const LoopRegistrationResult &accepted) {
            return registration.candidate.current_id >
                       accepted.candidate.current_id &&
                   registration.candidate.current_id -
                           accepted.candidate.current_id >
                       options_.neighbor_current_id_window;
          }),
      accepted_history_.end());

  for (const auto &decision : decisions)
  {
    if (decision.accepted)
      ++statistics_.accepted;
    else
      ++statistics_.rejected;
    WriteCsv(decision);
  }
  statistics_.pending = pending_.size();
  return decisions;
}

std::vector<LoopVerificationDecision> LoopVerifier::Flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LoopVerificationDecision> decisions;
  decisions.reserve(pending_.size());
  for (const auto &pending : pending_)
  {
    decisions.push_back(MakeRejected(
        pending.registration, LoopRejectReason::kNoConsistentNeighbor));
    ++statistics_.rejected;
    WriteCsv(decisions.back());
  }
  pending_.clear();
  statistics_.pending = 0;
  return decisions;
}

LoopVerifier::Statistics LoopVerifier::statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return statistics_;
}

void LoopVerifier::WriteCsv(const LoopVerificationDecision &decision)
{
  if (!csv_stream_.is_open()) return;
  const auto &registration = decision.registration;
  const Pose3d &measurement = registration.T_candidate_current;
  csv_stream_
      << std::setprecision(17) << registration.candidate.current_id << ','
      << registration.candidate.candidate_id << ','
      << static_cast<int>(decision.accepted) << ','
      << LoopRejectReasonToString(decision.reject_reason) << ','
      << decision.consistent_neighbors << ','
      << decision.best_neighbor_translation_error_m << ','
      << decision.best_neighbor_rotation_error_deg << ','
      << static_cast<int>(registration.converged) << ','
      << registration.converged_levels << ','
      << registration.transformation_probability << ','
      << registration.fitness_score_m2 << ',' << registration.overlap << ','
      << registration.overlap_rmse_m << ','
      << registration.correction_translation_m << ','
      << registration.correction_angle_deg << ','
      << measurement.translation.x() << ',' << measurement.translation.y()
      << ',' << measurement.translation.z() << ','
      << measurement.rotation.x() << ',' << measurement.rotation.y() << ','
      << measurement.rotation.z() << ',' << measurement.rotation.w() << '\n';
  csv_stream_.flush();
}

}  // namespace my_livo::backend
