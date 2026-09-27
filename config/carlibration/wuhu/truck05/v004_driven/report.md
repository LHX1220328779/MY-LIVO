# wuhu/truck05/v004_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck05/v003_ground
- lineage: wuhu/truck05/v001_import -> wuhu/truck05/v002_axle -> wuhu/truck05/v003_ground -> wuhu/truck05/v004_driven
- created: 2026-09-24T14:34:18+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck05_lio_0924_142354/Log/fusion/1790231486106403040
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        1.2,
        7.32,
        2.08
      ],
      "drpy_deg": [
        -0.029,
        -0.004,
        -0.046
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -1.92,
        3.56,
        -4.91
      ],
      "drpy_deg": [
        -0.132,
        0.007,
        -0.194
      ]
    }
  }
}
```

## metrics

```
{
  "refinement": {
    "refinement": {
      "reference": "front_lidar",
      "corrections": {
        "front_right_lidar": {
          "rotvec_deg": [
            -0.3038116671628296,
            0.5824862358503357,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            4.515085132903587
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.23316749866069172,
            -0.2461681982492946,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -1.017195227904469
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            -1.9245485106003306,
            -0.0438164488245191,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -13.89929394887509
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu/05/20260914/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        300,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              -0.029267332338615146,
              -0.003229731596274623,
              -0.047739446000967356
            ],
            "trans_cm": [
              1.2047442855279535,
              7.322572045761564,
              2.0807844487977603
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "only 1174 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": true,
              "iterations": 21,
              "matches": 12197,
              "vertical_matches": 1174,
              "median_before_cm": 8.77,
              "median_after_cm": 8.63,
              "vertical_before_cm": 7.61,
              "vertical_after_cm": 5.42,
              "rotation_deg": 0.056,
              "translation_cm": 7.7,
              "match_time_gap_s": 1.35,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.13111942643821442,
              -0.007423158380441897,
              -0.1985122649137408
            ],
            "trans_cm": [
              -1.9138974086686398,
              3.562839768606344,
              -4.90742658706254
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "not converged in 25 iterations",
                "only 889 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": false,
              "iterations": 25,
              "matches": 11061,
              "vertical_matches": 889,
              "median_before_cm": 8.29,
              "median_after_cm": 8.17,
              "vertical_before_cm": 6.06,
              "vertical_after_cm": 5.43,
              "rotation_deg": 0.238,
              "translation_cm": 6.4,
              "match_time_gap_s": 1.59,
              "pass2": null
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck05_lio_0924_142354/Log/fusion/1790231486106403040",
        "window": [
          8.0,
          40.0
        ],
        "prior": [
          0.5,
          0.1
        ],
        "poses": {
          "pass1": "native frames.csv",
          "pass2": "RTK-optimized trajectory_optimized.txt",
          "cross_time_s": 2.0
        },
        "quality": "warn"
      }
    }
  }
}
```

## summary

```
{
  "lidars": [
    {
      "lidar": "front_right_lidar",
      "key": "transform_matrix_lidar_1",
      "t": [
        2.8131,
        5.5135,
        1.9875
      ],
      "rpy_deg": [
        0.623,
        -2.247,
        -1.141
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1268,
        -0.7633,
        0.6302
      ],
      "rpy_deg": [
        -1.616,
        -1.82,
        -90.312
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.029,
        5.4106,
        1.883
      ],
      "rpy_deg": [
        -0.583,
        -1.934,
        179.713
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0968,
        7.5648,
        0.5356
      ],
      "rpy_deg": [
        0.072,
        -5.301,
        89.874
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0022,
      7.5704,
      2.2531
    ],
    "rpy_deg": [
      -100.385,
      0.003,
      -0.449
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.570556640625,
      1855.53173828125,
      971.0083025529602,
      534.2602337509306
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "source": "defaults (wuhu truck29 yaml)"
  },
  "time_offsets": {
    "lidar": -0.0368,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v003_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck05_lio_0924_142354/Log/fusion/1790231486106403040
window 8.0..40.0 s, lidar_time_offset -0.0368
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 228479 world points
front_right_lidar: pass 1 matches 12173->12197 |res| median 8.77->8.63 cm; ground 9.10->9.17 (n=10380); vertical 7.61->5.42 (n=1174); match time gap median 1.3 s; correction rot deg (x,y,z)=(-0.029,-0.003,-0.048) trans cm=(+1.2,+7.3,+2.1); iters 21  [quality: warn; only 1174 vertical planar matches (yaw/x/y weakly observed)]
front_left_lidar: pass 1 matches 10964->11061 |res| median 8.29->8.17 cm; ground 8.64->8.66 (n=9636); vertical 6.06->5.43 (n=889); match time gap median 1.6 s; correction rot deg (x,y,z)=(+0.131,-0.007,-0.199) trans cm=(-1.9,+3.6,-4.9); iters 25  [quality: warn; not converged in 25 iterations; only 889 vertical planar matches (yaw/x/y weakly observed)]
back_lidar: insufficient matches; manual matrix kept  [quality: warn]
quality: warn  (front_right_lidar: only 1174 vertical planar matches (yaw/x/y weakly observed); front_left_lidar: not converged in 25 iterations, only 889 vertical planar matches (yaw/x/y weakly observed); back_lidar: insufficient matches; matrix kept)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v004_driven/calibration.json
```
