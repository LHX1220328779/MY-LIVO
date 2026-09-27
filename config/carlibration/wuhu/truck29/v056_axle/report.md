# wuhu/truck29/v056_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck29/v055_import
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle
- created: 2026-09-24T13:38:57+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -9.91,
        49.43,
        -10.02
      ],
      "drpy_deg": [
        -0.604,
        2.439,
        1.09
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -3.79,
        43.11,
        7.87
      ],
      "drpy_deg": [
        -2.44,
        -0.597,
        1.014
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -9.42,
        38.78,
        15.3
      ],
      "drpy_deg": [
        0.603,
        -2.438,
        1.035
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -21.57,
        42.73,
        0.0
      ],
      "drpy_deg": [
        2.454,
        0.54,
        0.966
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -14.26,
      44.4,
      -0.31
    ],
    "drpy_deg": [
      -0.52,
      2.457,
      1.04
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.1459
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
      -0.5162,
      2.4578,
      1.0432
    ],
    "solution_translation_cm": [
      -21.57,
      42.73,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.5162,
      2.4578,
      1.0432
    ],
    "correction_translation_cm": [
      -21.57,
      42.73,
      0.0
    ],
    "correction_matrix": [
      0.9989143975939524,
      -0.01839285946998454,
      0.042798703251724496,
      -0.07776105983282382,
      0.018006480691398947,
      0.9997937205271916,
      0.00939590589061845,
      0.4264195945188485,
      -0.042962692334420204,
      -0.008615051648857484,
      0.999039532727542,
      0.0702262182473796,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": -0.0209,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 1.965,
      "after": 1.881,
      "ground_after": 1.654,
      "vertical_after": 3.263
    },
    "sqrt_information": {
      "rx": 5082.9,
      "ry": 2550.9,
      "rz": 2251.5,
      "tx": 20.9,
      "ty": 20.8,
      "dt": 379.4
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.001,
        0.001,
        0.053
      ],
      "translation_cm": [
        7.9,
        6.8,
        0.0
      ],
      "time_ms": -0.9
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0853,
        "median_cm": 4.593,
        "ground_cm": 5.14
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0616,
        "median_cm": 4.186,
        "ground_cm": 4.365
      },
      {
        "baseline_s": -0.12,
        "dt_s": -0.042,
        "median_cm": 3.577,
        "ground_cm": 3.523
      },
      {
        "baseline_s": -0.09,
        "dt_s": -0.0302,
        "median_cm": 2.856,
        "ground_cm": 2.667
      },
      {
        "baseline_s": -0.06,
        "dt_s": -0.0231,
        "median_cm": 2.19,
        "ground_cm": 1.965
      },
      {
        "baseline_s": -0.03,
        "dt_s": -0.021,
        "median_cm": 1.891,
        "ground_cm": 1.663
      },
      {
        "baseline_s": 0.0,
        "dt_s": -0.0213,
        "median_cm": 1.871,
        "ground_cm": 1.646
      }
    ],
    "time_profile_min_dt_s": -0.0213,
    "scans_used": 550,
    "window_s": [
      9.8,
      119.6
    ],
    "dof": "rpy_xy",
    "iterations": 11,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 206.7
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
        2.7326,
        5.9395,
        1.8129
      ],
      "rpy_deg": [
        -0.893,
        -3.269,
        -1.465
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1075,
        -0.274,
        0.7048
      ],
      "rpy_deg": [
        -1.122,
        0.401,
        -91.377
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1321,
        5.7274,
        1.9877
      ],
      "rpy_deg": [
        -3.209,
        -2.038,
        178.487
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1088,
        8.0401,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.532,
        89.924
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0029,
      8.0422,
      1.7517
    ],
    "rpy_deg": [
      -96.506,
      1.636,
      0.424
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
    "lidar": -0.1459,
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v055_import/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 600, skipped {'slow': 24, 'startup': 23, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 550 (9.8..119.6 s), points 4071144; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.085s:4.59cm, -0.150/-0.062s:4.19cm, -0.120/-0.042s:3.58cm, -0.090/-0.030s:2.86cm, -0.060/-0.023s:2.19cm, -0.030/-0.021s:1.89cm, +0.000/-0.021s:1.87cm
detected time shift -0.021 s (median 1.87 cm) -> final joint solve
residual median 1.96 -> 1.88 cm (ground 1.71 -> 1.65, n=82220; vertical 3.41 -> 3.26, n=6973); match time gap median 3.5 s; iterations 11
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.516, y(roll) +2.458, z(yaw) +1.043 deg; translation x -21.6, y +42.7 cm
applied (rpy_xy): rotvec x(pitch) -0.516, y(roll) +2.458, z(yaw) +1.043 deg; translation x -21.6, y +42.7, z +0.0 cm; LiDAR time shift -20.9 ms -> time_offsets.lidar -0.1459 s
repeatability (first vs second half of the used scans): rotvec [0.001, 0.001, 0.053] deg, translation [7.9, 6.8, 0.0] cm, time -0.9 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v056_axle/calibration.json (207 s)
```
