#ifndef MY_LIVO_BACKEND_LOOP_VERIFIER_H
#define MY_LIVO_BACKEND_LOOP_VERIFIER_H

#include "backend/loop_registration.h"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace my_livo::backend
{

enum class LoopRejectReason : std::uint8_t
{
  kNone = 0,
  kRegistrationFailed,
  kNotConverged,
  kInsufficientConvergedLevels,
  kProbabilityTooLow,
  kFitnessTooHigh,
  kOverlapTooLow,
  kOverlapRmseTooHigh,
  kTranslationSanity,
  kRotationSanity,
  kNoConsistentNeighbor,
};

const char *LoopRejectReasonToString(LoopRejectReason reason);

struct LoopVerificationDecision
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LoopRegistrationResult registration;
  bool accepted = false;
  LoopRejectReason reject_reason = LoopRejectReason::kNone;
  std::size_t consistent_neighbors = 0;
  double best_neighbor_translation_error_m = 0.0;
  double best_neighbor_rotation_error_deg = 0.0;
};

// High-precision loop verifier. A registration must pass all individual NDT
// gates and agree with at least one nearby, independently registered loop.
// It does not modify the graph; accepted decisions are returned to the caller.
class LoopVerifier
{
public:
  struct Options
  {
    bool require_final_convergence = true;
    std::size_t minimum_converged_levels = 4;
    double minimum_transformation_probability = 0.35;
    double maximum_fitness_score_m2 = 0.09;
    double minimum_overlap = 0.90;
    double maximum_overlap_rmse_m = 0.32;
    double maximum_translation_correction_m = 5.0;
    double maximum_rotation_correction_deg = 10.0;
    std::uint64_t neighbor_current_id_window = 40;
    std::uint64_t neighbor_candidate_id_window = 40;
    double maximum_neighbor_translation_error_m = 0.50;
    double maximum_neighbor_rotation_error_deg = 2.0;
    std::size_t minimum_consistent_neighbors = 1;
    std::string csv_path;
  };

  struct Statistics
  {
    std::uint64_t registrations = 0;
    std::uint64_t individually_valid = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::size_t pending = 0;
  };

  explicit LoopVerifier(const Options &options);

  std::vector<LoopVerificationDecision> Add(
      const LoopRegistrationResult &registration);
  std::vector<LoopVerificationDecision> Flush();
  Statistics statistics() const;

private:
  struct Consistency
  {
    bool is_neighbor = false;
    bool consistent = false;
    double translation_error_m = 0.0;
    double rotation_error_deg = 0.0;
  };

  struct PendingRegistration
  {
    LoopRegistrationResult registration;
    std::size_t consistent_neighbors = 0;
    double best_translation_error_m = 0.0;
    double best_rotation_error_deg = 0.0;
  };

  static void ValidateOptions(const Options &options);
  LoopRejectReason IndividualRejectReason(
      const LoopRegistrationResult &registration) const;
  Consistency CheckConsistency(
      const LoopRegistrationResult &reference,
      const LoopRegistrationResult &query) const;
  LoopVerificationDecision MakeRejected(
      const LoopRegistrationResult &registration,
      LoopRejectReason reason) const;
  LoopVerificationDecision MakeAccepted(
      const LoopRegistrationResult &registration,
      std::size_t consistent_neighbors,
      double best_translation_error_m,
      double best_rotation_error_deg) const;
  void WriteCsv(const LoopVerificationDecision &decision);

  Options options_;
  mutable std::mutex mutex_;
  std::vector<PendingRegistration> pending_;
  std::vector<LoopRegistrationResult> accepted_history_;
  Statistics statistics_;
  std::ofstream csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_LOOP_VERIFIER_H
