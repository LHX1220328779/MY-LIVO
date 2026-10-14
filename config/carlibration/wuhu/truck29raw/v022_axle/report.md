# wuhu/truck29raw/v022_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck29raw/v001_import
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v022_axle
- created: 2026-10-14T22:05:16+08:00
- note: agent 2026-10-14: same-time driven pairing + whole-recording lateral axle, from v001_import

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- window: [0.0, 120.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -9.3,
        40.55,
        -10.28
      ],
      "drpy_deg": [
        -0.576,
        2.472,
        1.15
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -2.58,
        34.0,
        7.46
      ],
      "drpy_deg": [
        -2.473,
        -0.569,
        1.074
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -8.77,
        29.28,
        15.35
      ],
      "drpy_deg": [
        0.576,
        -2.472,
        1.097
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -21.29,
        33.64,
        0.0
      ],
      "drpy_deg": [
        2.487,
        0.512,
        1.026
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -13.88,
      35.24,
      -0.32
    ],
    "drpy_deg": [
      -0.491,
      2.489,
      1.101
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0361
    },
    "lidar_frontend": {
      "from": null,
      "to": -0.0311
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
      -0.4887,
      2.4902,
      1.1048
    ],
    "solution_translation_cm": [
      -23.6,
      33.64,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.4887,
      2.4902,
      1.1048
    ],
    "correction_translation_cm": [
      -23.6,
      33.64,
      0.0
    ],
    "correction_matrix": [
      0.9988698846852347,
      -0.01945947440191076,
      0.04336222232436451,
      -0.08994497740726522,
      0.019088875361523303,
      0.9997777819203914,
      0.00894436223406972,
      0.33557088598045226,
      -0.04352663904252735,
      -0.008106518015979735,
      0.9990193772191399,
      0.06641615316345195,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.0889,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 2.142,
      "after": 1.971,
      "ground_after": 1.566,
      "vertical_after": 2.815
    },
    "sqrt_information": {
      "rx": 3923.5,
      "ry": 2380.1,
      "rz": 2083.4,
      "tx": 37.8,
      "ty": 30.6,
      "dt": 293.9
    },
    "repeatability_halves": {
      "rotvec_deg": [
        -0.001,
        -0.084,
        -0.057
      ],
      "translation_cm": [
        3.6,
        12.9,
        0.0
      ],
      "time_ms": 7.8
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0365,
        "median_cm": 3.883,
        "ground_cm": 3.786
      },
      {
        "baseline_s": -0.15,
        "dt_s": 0.0,
        "median_cm": 3.567,
        "ground_cm": 3.385
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0373,
        "median_cm": 3.017,
        "ground_cm": 2.702
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0594,
        "median_cm": 2.608,
        "ground_cm": 2.204
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0758,
        "median_cm": 2.281,
        "ground_cm": 1.873
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0811,
        "median_cm": 2.273,
        "ground_cm": 1.886
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.083,
        "median_cm": 2.295,
        "ground_cm": 1.919
      }
    ],
    "time_profile_min_dt_s": 0.0811,
    "scans_used": 491,
    "window_s": [
      21.6,
      120.0
    ],
    "dof": "rpy_xy",
    "iterations": 14,
    "converged": true,
    "quality": "warn",
    "reasons": [
      "time shift +89 ms is a large change vs the parent offset (candidates/longer recording confirms it)"
    ],
    "runtime_s": 273.9
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.005000000000000282,
    "corr": 0.9981,
    "yaw_rms_rad_s": 0.0676,
    "template": -0.125,
    "decision": "replaced template with -0.0311"
  },
  "lateral": {
    "correction_x_cm": 2.31,
    "match_gap_median_s": 153.6,
    "converged": true,
    "residual_cm": {
      "before": 2.309,
      "after": 1.996,
      "ground_after": 1.757,
      "vertical_after": 2.99
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.031,
        -0.043,
        -0.088
      ],
      "translation_cm": [
        -7.0,
        -4.3,
        0.0
      ],
      "time_ms": 0.0
    },
    "scans_used": 590,
    "window_s": [
      21.6,
      842.8
    ],
    "decision": "applied x +2.3 cm"
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
        2.7387,
        5.8507,
        1.8103
      ],
      "rpy_deg": [
        -0.865,
        -3.236,
        -1.405
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0954,
        -0.3651,
        0.7007
      ],
      "rpy_deg": [
        -1.155,
        0.429,
        -91.317
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1256,
        5.6324,
        1.9882
      ],
      "rpy_deg": [
        -3.236,
        -2.072,
        178.549
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.106,
        7.9492,
        0.051
      ],
      "rpy_deg": [
        1.485,
        -1.56,
        89.984
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0067,
      7.9506,
      1.7516
    ],
    "rpy_deg": [
      -96.477,
      1.668,
      0.485
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
    "lidar": -0.0361,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v001_import/calibration.json
bag=/home/project/data/haibo/wuhu/29/20261013/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 599, skipped {'slow': 84, 'startup': 22, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 491 (21.6..120.0 s), points 5069129; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.036s:3.88cm, -0.150/+0.000s:3.57cm, -0.120/+0.037s:3.02cm, -0.090/+0.059s:2.61cm, -0.060/+0.076s:2.28cm, -0.030/+0.081s:2.27cm, +0.000/+0.083s:2.29cm
detected time shift +0.081 s (median 2.27 cm) -> final joint solve
residual median 2.14 -> 1.97 cm (ground 1.67 -> 1.57, n=51707; vertical 3.09 -> 2.82, n=13353); match time gap median 3.6 s; iterations 14
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.489, y(roll) +2.490, z(yaw) +1.105 deg; translation x -23.6, y +33.6 cm
applied (rpy_xy): rotvec x(pitch) -0.489, y(roll) +2.490, z(yaw) +1.105 deg; translation x -23.6, y +33.6, z +0.0 cm; LiDAR time shift +88.9 ms -> time_offsets.lidar -0.0361 s
repeatability (first vs second half of the used scans): rotvec [-0.001, -0.084, -0.057] deg, translation [3.6, 12.9, 0.0] cm, time +7.8 ms
quality: warn  (time shift +89 ms is a large change vs the parent offset (candidates/longer recording confirms it))
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v022_axle/calibration.json (274 s)
frontend time base: replaced template with -0.0311; imu/ins lag +5.0 ms, corr 0.998, yaw rms 0.068 rad/s
lateral lever arm (whole recording, dof x, rotation/time kept): applied x +2.3 cm; solved x +2.3 cm, match time gap median 154 s
```
