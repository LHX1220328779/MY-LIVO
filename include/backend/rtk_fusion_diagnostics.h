#ifndef MY_LIVO_BACKEND_RTK_FUSION_DIAGNOSTICS_H
#define MY_LIVO_BACKEND_RTK_FUSION_DIAGNOSTICS_H

#include "backend/keyframe.h"
#include "backend/rtk_observation_buffer.h"

#include <cstddef>
#include <fstream>
#include <mutex>
#include <string>

namespace my_livo::backend
{

// Phase-1 observation-only instrumentation for the RTK fusion refactor.
// This class deliberately owns no optimizer state and cannot change a pose.
// It captures the raw LIO pose, the current legacy graph pose and the
// timestamp-interpolated RTK measurement in one row so later phases can be
// compared against an immutable baseline.
class RtkFusionDiagnostics
{
public:
  struct Options
  {
    std::string keyframe_csv_path;
    std::string initial_alignment_csv_path;
  };

  struct InitialAlignmentRecord
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double initialization_timestamp = 0.0;
    double first_sample_timestamp = 0.0;
    double last_sample_timestamp = 0.0;
    std::size_t sample_count = 0;
    Pose3d T_mine_lio_initial;
    double position_rms_m = 0.0;
    double position_max_m = 0.0;
    double orientation_max_deg = 0.0;
  };

  explicit RtkFusionDiagnostics(const Options &options);

  void RecordInitialAlignment(const InitialAlignmentRecord &record);
  void RecordKeyframe(const Keyframe &keyframe,
                      const RtkObservationQuery &query,
                      const std::string &decision,
                      bool factor_added,
                      const Pose3d &T_backend_before,
                      const Pose3d &T_backend_after);

private:
  std::mutex mutex_;
  std::ofstream keyframe_csv_stream_;
  std::ofstream initial_alignment_csv_stream_;
};

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_RTK_FUSION_DIAGNOSTICS_H
