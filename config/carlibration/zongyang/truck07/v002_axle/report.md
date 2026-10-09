# zongyang/truck07/v002_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: zongyang/truck07/v001_import
- lineage: zongyang/truck07/v001_import -> zongyang/truck07/v002_axle
- created: 2026-10-08T17:29:38+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/zongyang/07/20261007/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/07/20261007/ros2bag', 'kind': 'raw_split', 'segments': 12, 'duration_s': 359.99, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -10.57,
        -29.76,
        1.56
      ],
      "drpy_deg": [
        -0.892,
        0.326,
        -0.43
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -15.6,
        -29.37,
        12.6
      ],
      "drpy_deg": [
        -0.324,
        -0.893,
        -0.426
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -10.73,
        -25.36,
        5.34
      ],
      "drpy_deg": [
        0.894,
        -0.32,
        -0.415
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -8.63,
        -26.46,
        0.0
      ],
      "drpy_deg": [
        0.319,
        0.894,
        -0.404
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -8.68,
      -26.6,
      0.0
    ],
    "drpy_deg": [
      -0.884,
      0.346,
      -0.44
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0041
    }
  }
}
```

## metrics

```
{
  "rear_axle_alignment": {
    "reference": "transform_matrix_lidar_6",
    "poses": "imu_data/odometry (RTK fixed)",
    "applied_to": "all LiDARs and the camera (rig correction)",
    "solution_rotvec_deg": [
      -0.8911,
      0.3271,
      -0.4169
    ],
    "solution_translation_cm": [
      -8.63,
      -26.46,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.8911,
      0.3271,
      -0.4169
    ],
    "correction_translation_cm": [
      -8.63,
      -26.46,
      0.0
    ],
    "correction_matrix": [
      0.9999572271683944,
      0.007232105979579297,
      0.00576545547160391,
      -0.1552302353311904,
      -0.007320900824647385,
      0.9998525728877983,
      0.015531802881952411,
      -0.3026741134666926,
      -0.005652277842656995,
      -0.01557334687047976,
      0.9998627518926982,
      0.1174466623469379,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.1209,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 3.592,
      "after": 2.922,
      "ground_after": 2.933,
      "vertical_after": 2.265
    },
    "sqrt_information": {
      "rx": 4168.4,
      "ry": 4095.6,
      "rz": 1369.7,
      "tx": 49.0,
      "ty": 40.3,
      "dt": 394.4
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.023,
        0.05,
        0.01
      ],
      "translation_cm": [
        -3.2,
        -13.5,
        0.0
      ],
      "time_ms": -10.4
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": 0.0303,
        "median_cm": 4.263,
        "ground_cm": 4.061
      },
      {
        "baseline_s": -0.15,
        "dt_s": 0.0607,
        "median_cm": 3.793,
        "ground_cm": 3.63
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0824,
        "median_cm": 3.629,
        "ground_cm": 3.507
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0966,
        "median_cm": 3.575,
        "ground_cm": 3.489
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.1069,
        "median_cm": 3.718,
        "ground_cm": 3.686
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.1115,
        "median_cm": 3.849,
        "ground_cm": 3.882
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.1129,
        "median_cm": 3.878,
        "ground_cm": 3.95
      }
    ],
    "time_profile_min_dt_s": 0.0966,
    "scans_used": 576,
    "window_s": [
      9.0,
      359.4
    ],
    "dof": "rpy_xy",
    "iterations": 24,
    "converged": false,
    "quality": "warn",
    "reasons": [
      "time shift +121 ms is a large change vs the parent offset (candidates/longer recording confirms it)",
      "not converged in 24 iterations",
      "halves disagree (few turns/structure: use a longer recording with turns)"
    ],
    "runtime_s": 280.2
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
        2.7655,
        5.1845,
        1.7493
      ],
      "rpy_deg": [
        -2.379,
        0.984,
        -0.337
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1687,
        -0.8432,
        0.6945
      ],
      "rpy_deg": [
        0.923,
        1.102,
        -90.392
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.2282,
        4.9732,
        1.7878
      ],
      "rpy_deg": [
        2.42,
        -0.061,
        179.344
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1092,
        7.2631,
        2.5109
      ],
      "rpy_deg": [
        0.485,
        2.809,
        89.258
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1403,
      7.2764,
      2.4087
    ],
    "rpy_deg": [
      -90.448,
      1.632,
      0.97
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      621.1,
      900.65,
      958.95,
      537.17
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "intrinsics_check": {
      "method": "supplier ground-distance curves (midrange_camera[0].equation_*m)",
      "curves": 10,
      "distances_m": [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        70.0,
        80.0,
        90.0,
        100.0
      ],
      "fits": {
        "free": {
          "intrinsics": [
            621.1,
            900.65,
            958.95,
            537.17
          ],
          "rms_px": 0.309,
          "circle_centre_rear_m": [
            -0.01,
            7.536
          ],
          "ground_z_rear_m": -1.263
        },
        "square": {
          "intrinsics": [
            621.33,
            621.33,
            1050.56,
            537.09
          ],
          "rms_px": 2.404,
          "circle_centre_rear_m": [
            -0.942,
            7.544
          ],
          "ground_z_rear_m": -2.916
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.309,
      "camera_height_m": 3.672,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -62.19,
        "fy_pct": -51.46,
        "cx_px": -12.1,
        "cy_px": 2.9
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.309 px)"
  },
  "time_offsets": {
    "lidar": -0.0041,
    "lidar_frontend": -0.125,
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
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck07/v001_import/calibration.json
bag=/home/project/data/haibo/zongyang/07/20261007/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 1791, skipped {'slow': 62, 'startup': 0, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 576 (9.0..359.4 s), points 4103082; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/+0.030s:4.26cm, -0.150/+0.061s:3.79cm, -0.120/+0.082s:3.63cm, -0.090/+0.097s:3.58cm, -0.060/+0.107s:3.72cm, -0.030/+0.112s:3.85cm, +0.000/+0.113s:3.88cm
detected time shift +0.097 s (median 3.58 cm) -> final joint solve
residual median 3.59 -> 2.92 cm (ground 3.48 -> 2.93, n=82810; vertical 4.33 -> 2.26, n=6359); match time gap median 38.6 s; iterations 24 (NOT converged)
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.891, y(roll) +0.327, z(yaw) -0.417 deg; translation x -8.6, y -26.5 cm
applied (rpy_xy): rotvec x(pitch) -0.891, y(roll) +0.327, z(yaw) -0.417 deg; translation x -8.6, y -26.5, z +0.0 cm; LiDAR time shift +120.9 ms -> time_offsets.lidar -0.0041 s
repeatability (first vs second half of the used scans): rotvec [0.023, 0.05, 0.01] deg, translation [-3.2, -13.5, 0.0] cm, time -10.4 ms
quality: warn  (time shift +121 ms is a large change vs the parent offset (candidates/longer recording confirms it); not converged in 24 iterations; halves disagree (few turns/structure: use a longer recording with turns))
written /home/project/MY-LIVO/config/carlibration/zongyang/truck07/v002_axle/calibration.json (280 s)
```
