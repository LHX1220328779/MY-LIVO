# Odometry-only pose graph

This stage adds an online SE(3) graph without loop or RTK factors. It does not
replace FAST-LIVO2 and never writes an optimized pose into the frontend IEKF or
voxel map.

## Graph definition

Each selected keyframe owns one node `X_k = T_map_body_k`. Node zero is fixed
at its raw frontend pose to remove the global six-degree-of-freedom gauge. For
every later node, exactly one constraint is appended:

```text
Z_(k-1,k) = inverse(T_odom_body_(k-1)) * T_odom_body_k
```

Translation in each relative measurement is expressed in the preceding body
frame. GTSAM `Pose3` uses tangent order `[rotation, translation]`, so the
project's configured `[translation, rotation]` standard deviations are
explicitly reordered before constructing the diagonal noise model. The
frontend state covariance is logged and published but is not used as a
relative-motion covariance because consecutive IEKF states are correlated.

The implementation keeps one persistent GTSAM 4.2.2 `ISAM2` instance. A new
keyframe submits only one `Pose3` initial value and one `BetweenFactor<Pose3>`;
the graph is not reconstructed or batch-solved. Node zero is anchored by a
tight `PriorFactor<Pose3>`. The initial value of every later node is propagated
from the latest optimized predecessor through its raw relative motion, which
remains valid after future loop or RTK corrections.

The tight prior is appropriate for this odometry-only acceptance stage because
the frontend origin has already been initialized from CGI-610. Before absolute
RTK factors are enabled, its covariance/removal policy must be reviewed so a
small initial ENU anchoring error is not forced into the rest of the trajectory.

GTSAM is built from `3rdparty/gtsam` with system Eigen 3.4.0 so its ABI matches
PCL and the frontend. TBB is disabled in this private build because the current
OpenCV package loads legacy TBB 2 while Ubuntu's GTSAM configuration otherwise
loads TBB 12. The build is isolated in `build-system-eigen` and
`install-system-eigen`; existing user GTSAM builds are not overwritten.

The graph organization and relative-pose convention follow the useful part of
Lightning-LM's Miao backend; Lightning-LM's LIO frontend is not imported.

## Outputs and invariants

```text
/backend/keyframe_path_optimized
/backend/odometry_optimized
Log/backend/pose_graph.csv
```

During this odometry-only stage, `backend.map_frame_id` is configured as
`mine`, so raw and optimized paths can be overlaid directly and no premature
`map -> odom` TF is published. A distinct `map` frame and its correction TF are
introduced only after an accepted global constraint exists.

The following must hold for every row:

```text
nodes == keyframe_id + 1
odometry_factors == nodes - 1
solution_usable == true
solver == gtsam_isam2
optimized trajectory approximately equals raw keyframe trajectory
```

The CSV additionally records iSAM2 relinearization and reelimination counts.
GTSAM 4.2.2 does not initialize `ISAM2Result::variablesReeliminated` when an
empty refinement update performs no Bayes-tree recalculation. The backend
therefore enables detailed results and counts value-initialized per-variable
status flags instead of reading that unsafe scalar. The validation script also
enforces the mathematical per-row upper bound
`nodes * (1 + additional_update_steps)`.
Any GTSAM exception or non-finite estimate stops the backend with the exact
keyframe ID rather than letting a partially invalid graph continue. Raw
`T_odom_body` values remain immutable in all cases.
