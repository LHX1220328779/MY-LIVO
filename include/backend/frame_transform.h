#ifndef MY_LIVO_BACKEND_FRAME_TRANSFORM_H
#define MY_LIVO_BACKEND_FRAME_TRANSFORM_H

#include "backend/keyframe.h"

#include <stdexcept>

namespace my_livo::backend
{

// The frontend remains a continuous odometry source. Global constraints only
// move this external transform; they are never fed back into the IEKF.
inline Pose3d ComputeMapToOdom(const Pose3d &T_map_body,
                               const Pose3d &T_odom_body)
{
  if (!T_map_body.isFinite() || !T_odom_body.isFinite())
    throw std::invalid_argument("Cannot compute map->odom from invalid poses.");
  return T_map_body * T_odom_body.inverse();
}

}  // namespace my_livo::backend

#endif  // MY_LIVO_BACKEND_FRAME_TRANSFORM_H
