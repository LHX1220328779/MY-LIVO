# xingye/truck08/v002_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: xingye/truck08/v001_import
- lineage: xingye/truck08/v001_import -> xingye/truck08/v002_axle
- created: 2026-10-20T16:57:46+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/xingye/08/20261018/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/08/20261018/ros2bag', 'kind': 'raw_split', 'segments': 10, 'duration_s': 299.99, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -21.57,
        5.54,
        -3.0
      ],
      "drpy_deg": [
        -0.598,
        1.097,
        0.843
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -9.47,
        1.48,
        13.62
      ],
      "drpy_deg": [
        -0.914,
        -0.853,
        0.855
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -21.55,
        -0.09,
        3.21
      ],
      "drpy_deg": [
        1.039,
        -0.696,
        0.818
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -22.31,
        2.14,
        0.0
      ],
      "drpy_deg": [
        0.923,
        0.843,
        0.886
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -21.29,
      2.86,
      0.22
    ],
    "drpy_deg": [
      -0.877,
      0.89,
      0.803
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0054
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": 0.0021
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
      -0.8429,
      0.9225,
      0.8391
    ],
    "solution_translation_cm": [
      -22.31,
      2.14,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.8429,
      0.9225,
      0.8391
    ],
    "correction_translation_cm": [
      -22.31,
      2.14,
      0.0
    ],
    "correction_matrix": [
      0.9997631636034313,
      -0.014761605305785012,
      0.015990988413925808,
      -0.13275738795393047,
      0.014524761265131823,
      0.9997845673703364,
      0.014827344951152052,
      0.009074616027449522,
      -0.016206418847142502,
      -0.01459156800709744,
      0.9997621907890117,
      0.1042452800667879,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.1196,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 2.157,
      "after": 2.062,
      "ground_after": 1.751,
      "vertical_after": 2.775
    },
    "sqrt_information": {
      "rx": 3182.2,
      "ry": 2792.6,
      "rz": 1628.3,
      "tx": 34.2,
      "ty": 41.6,
      "dt": 407.6
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.003,
        -0.088,
        -0.149
      ],
      "translation_cm": [
        2.4,
        8.1,
        0.0
      ],
      "time_ms": -1.9
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0638,
        "median_cm": 3.862,
        "ground_cm": 4.211
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0303,
        "median_cm": 3.639,
        "ground_cm": 3.804
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0049,
        "median_cm": 3.356,
        "ground_cm": 3.332
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0395,
        "median_cm": 3.056,
        "ground_cm": 2.86
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0645,
        "median_cm": 2.833,
        "ground_cm": 2.565
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0812,
        "median_cm": 2.785,
        "ground_cm": 2.545
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.0932,
        "median_cm": 2.762,
        "ground_cm": 2.545
      },
      {
        "baseline_s": 0.03,
        "dt_s": 0.1019,
        "median_cm": 2.73,
        "ground_cm": 2.496
      },
      {
        "baseline_s": 0.06,
        "dt_s": 0.1093,
        "median_cm": 2.537,
        "ground_cm": 2.272
      },
      {
        "baseline_s": 0.09,
        "dt_s": 0.1155,
        "median_cm": 2.22,
        "ground_cm": 1.93
      },
      {
        "baseline_s": 0.12,
        "dt_s": 0.1201,
        "median_cm": 2.07,
        "ground_cm": 1.756
      },
      {
        "baseline_s": 0.15,
        "dt_s": 0.1257,
        "median_cm": 2.209,
        "ground_cm": 1.926
      }
    ],
    "time_profile_min_dt_s": 0.1201,
    "scans_used": 433,
    "window_s": [
      13.0,
      299.7
    ],
    "dof": "rpy_xy",
    "iterations": 15,
    "converged": true,
    "quality": "warn",
    "reasons": [
      "time shift +120 ms is a large change vs the parent offset (candidates/longer recording confirms it)"
    ],
    "runtime_s": 193.9
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.007500000000000284,
    "corr": 0.9988,
    "yaw_rms_rad_s": 0.0418,
    "template": -0.125,
    "decision": "replaced template with +0.0021"
  },
  "lateral": {
    "decision": "skipped (the window already covers the recording)"
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
        1.8514,
        6.94,
        1.1669
      ],
      "rpy_deg": [
        -1.153,
        0.154,
        14.256
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2914,
        -1.9658,
        0.6837
      ],
      "rpy_deg": [
        0.057,
        -1.417,
        -90.158
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.0271,
        6.9363,
        1.2319
      ],
      "rpy_deg": [
        0.32,
        -1.472,
        166.65
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0833,
        6.9974,
        0.7941
      ],
      "rpy_deg": [
        1.031,
        3.331,
        90.436
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1729,
      6.9547,
      1.384
    ],
    "rpy_deg": [
      -101.245,
      2.746,
      -1.747
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.35,
      1855.45,
      959.039,
      539.189
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0054,
    "lidar_frontend": 0.0021,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0054 + imu/ins clock lag +0.0075 (corr 0.999); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck08/v001_import/calibration.json
bag=/home/project/data/haibo/xingye/08/20261018/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 1495, skipped {'slow': 1060, 'startup': 0, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 433 (13.0..299.7 s), points 3202077; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.064s:3.86cm, -0.150/-0.030s:3.64cm, -0.120/+0.005s:3.36cm, -0.090/+0.040s:3.06cm, -0.060/+0.065s:2.83cm, -0.030/+0.081s:2.79cm, +0.000/+0.093s:2.76cm, +0.030/+0.102s:2.73cm, +0.060/+0.109s:2.54cm, +0.090/+0.116s:2.22cm, +0.120/+0.120s:2.07cm, +0.150/+0.126s:2.21cm
detected time shift +0.120 s (median 2.07 cm) -> final joint solve
residual median 2.16 -> 2.06 cm (ground 1.82 -> 1.75, n=53625; vertical 2.96 -> 2.78, n=10758); match time gap median 4.1 s; iterations 15
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.843, y(roll) +0.922, z(yaw) +0.839 deg; translation x -22.3, y +2.1 cm
applied (rpy_xy): rotvec x(pitch) -0.843, y(roll) +0.922, z(yaw) +0.839 deg; translation x -22.3, y +2.1, z +0.0 cm; LiDAR time shift +119.6 ms -> time_offsets.lidar -0.0054 s
repeatability (first vs second half of the used scans): rotvec [0.003, -0.088, -0.149] deg, translation [2.4, 8.1, 0.0] cm, time -1.9 ms
quality: warn  (time shift +120 ms is a large change vs the parent offset (candidates/longer recording confirms it))
written /home/project/MY-LIVO/config/carlibration/xingye/truck08/v002_axle/calibration.json (194 s)
frontend time base: replaced template with +0.0021; imu/ins lag +7.5 ms, corr 0.999, yaw rms 0.042 rad/s
```
