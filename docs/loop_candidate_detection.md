# Loop candidate detection

This stage finds plausible historical keyframes but deliberately stops before
point-cloud registration. It does not create a loop factor and therefore cannot
change the GTSAM solution or the FAST-LIVO2 frontend.

## Selection policy

The implementation adapts Lightning-LM's `DetectLoopCandidates()` policy to
this project's persistent keyframes and GTSAM poses:

1. Check one current keyframe every configured ID interval.
2. Reject history that is too recent in either keyframe ID or sensor time.
3. Use `T_map_body` from the current GTSAM estimate for XY distance and height
   gates.
4. Sort surviving history by XY distance.
5. Suppress selected historical keyframes that are too close to each other in
   ID and cap the result count.

Compared with Lightning-LM, the independent sensor-time and height gates avoid
obvious false candidates on slow/stationary or multi-level mine trajectories.
Nearest-first ranking gives deterministic bounded work to the later NDT stage.
The current Truck 29 defaults are:

```text
check interval                    20 keyframes
minimum current/history gap       50 keyframes and 30 seconds
maximum XY distance               20 m
maximum height difference         5 m
selected-candidate ID separation  20 keyframes
maximum selected candidates       3 per check
```

For every candidate, the initial transform is

```text
T_candidate_current_initial =
    inverse(T_map_body_candidate) * T_map_body_current
```

It maps points in the current body frame into the historical candidate body
frame. This is only an initialization for the next registration stage.

## Outputs

```text
/backend/loop_candidates             persistent yellow line markers in RViz
Log/backend/loop_detection.csv       one accounting row per keyframe
Log/backend/loop_candidates.csv      one row per selected candidate pair
```

The summary CSV makes a skipped check distinguishable from an executed search
with no nearby history. The candidate CSV records IDs, timestamps, all gate
distances, and the complete initial SE(3) transform.

After one full bag run, validate gating, cadence, ranking, suppression, counts,
and transform direction with:

```bash
./scripts/validate_loop_candidates.py
```

The validator recomputes every expected candidate from `pose_graph.csv`; it
fails if no checks or no candidates were produced. Seeing yellow candidate
lines does not mean a loop has been accepted—the NDT registration and robust
graph-factor stages remain intentionally disabled.
