# zongyang/truck02/v010_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: zongyang/truck02/v009_ground
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v008_axle -> zongyang/truck02/v009_ground -> zongyang/truck02/v010_axle
- created: 2026-10-27T13:40:18+08:00
- note: agent: start auto calibration after empty body_filter fix

## inputs

- bag: /home/project/data/haibo/zongyang/02/20261026/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/02/20261026/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 239.98, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -0.09,
        0.73,
        0.0
      ],
      "drpy_deg": [
        0.0,
        0.0,
        0.0
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -0.09,
        0.72,
        0.0
      ],
      "drpy_deg": [
        0.0,
        0.0,
        0.0
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -0.09,
        0.72,
        0.0
      ],
      "drpy_deg": [
        0.0,
        0.0,
        0.0
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -0.1,
        0.72,
        0.0
      ],
      "drpy_deg": [
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -0.1,
      0.72,
      0.0
    ],
    "drpy_deg": [
      0.0,
      0.0,
      0.0
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.001,
      "to": 0.0033
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
      0.007,
      -0.0152,
      -0.1098
    ],
    "solution_translation_cm": [
      -0.1,
      0.73,
      0.0
    ],
    "correction_rotvec_deg": [
      0.0,
      0.0,
      -0.0
    ],
    "correction_translation_cm": [
      -0.1,
      0.73,
      0.0
    ],
    "correction_matrix": [
      1.0,
      6.505213034913027e-19,
      0.0,
      -0.0009592142427997802,
      -2.168404344971009e-18,
      1.0000000000000002,
      -6.938893903907228e-18,
      0.00725769759100281,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.0043,
    "time_offsets_used": {
      "lidar": -0.001,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 3.416,
      "after": 2.888,
      "ground_after": 3.225,
      "vertical_after": 1.869
    },
    "sqrt_information": {
      "rx": 2034.5,
      "ry": 2829.7,
      "rz": 924.0,
      "tx": 52.6,
      "ty": 34.1,
      "dt": 332.5
    },
    "repeatability_halves": {
      "rotvec_deg": [
        -0.144,
        -0.008,
        0.031
      ],
      "translation_cm": [
        -0.4,
        -4.6,
        0.0
      ],
      "time_ms": 0.9
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0793,
        "median_cm": 3.301,
        "ground_cm": 3.559
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0491,
        "median_cm": 3.174,
        "ground_cm": 3.478
      },
      {
        "baseline_s": -0.12,
        "dt_s": -0.0258,
        "median_cm": 3.005,
        "ground_cm": 3.347
      },
      {
        "baseline_s": -0.09,
        "dt_s": -0.0088,
        "median_cm": 2.997,
        "ground_cm": 3.337
      },
      {
        "baseline_s": -0.06,
        "dt_s": -0.0017,
        "median_cm": 2.98,
        "ground_cm": 3.278
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0025,
        "median_cm": 2.961,
        "ground_cm": 3.281
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.0035,
        "median_cm": 2.931,
        "ground_cm": 3.245
      },
      {
        "baseline_s": 0.03,
        "dt_s": 0.0061,
        "median_cm": 2.936,
        "ground_cm": 3.217
      }
    ],
    "time_profile_min_dt_s": 0.0035,
    "scans_used": 564,
    "window_s": [
      8.2,
      238.5
    ],
    "dof": "xy",
    "iterations": 22,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 137.2
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.005000000000000282,
    "corr": 0.9996,
    "yaw_rms_rad_s": 0.1762,
    "template": 0.004,
    "decision": "template confirmed (candidate +0.0083)"
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
        2.518,
        4.9782,
        1.8001
      ],
      "rpy_deg": [
        -0.257,
        -0.225,
        -2.919
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2268,
        -1.03,
        0.7988
      ],
      "rpy_deg": [
        -0.125,
        -3.488,
        -90.321
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1637,
        4.9062,
        1.8927
      ],
      "rpy_deg": [
        -1.675,
        -0.372,
        178.248
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0705,
        7.1162,
        2.4626
      ],
      "rpy_deg": [
        1.039,
        1.785,
        90.946
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1056,
      7.1264,
      2.1813
    ],
    "rpy_deg": [
      -90.039,
      3.007,
      -0.068
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      611.054443359375,
      895.9816284179688,
      954.4519112515845,
      541.4709836341353
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
    "lidar": 0.0033,
    "lidar_frontend": 0.004,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0010 + imu/ins clock lag +0.0050 (corr 1.000); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck02/v009_ground/calibration.json
bag=/home/project/data/haibo/zongyang/02/20261026/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.0010 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 1196, skipped {'slow': 45, 'startup': 21, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 564 (8.2..238.5 s), points 3253735; dof xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.079s:3.30cm, -0.150/-0.049s:3.17cm, -0.120/-0.026s:3.00cm, -0.090/-0.009s:3.00cm, -0.060/-0.002s:2.98cm, -0.030/+0.003s:2.96cm, +0.000/+0.004s:2.93cm, +0.030/+0.006s:2.94cm
detected time shift +0.004 s (median 2.93 cm) -> final joint solve
residual median 3.42 -> 2.89 cm (ground 3.70 -> 3.22, n=43120; vertical 2.09 -> 1.87, n=9706); match time gap median 72.3 s; iterations 22
solution about the LiDAR (rear-axle frame): rotvec x(pitch) +0.007, y(roll) -0.015, z(yaw) -0.110 deg; translation x -0.1, y +0.7 cm
applied (xy): rotvec x(pitch) +0.000, y(roll) +0.000, z(yaw) -0.000 deg; translation x -0.1, y +0.7, z +0.0 cm; LiDAR time shift +4.3 ms -> time_offsets.lidar +0.0033 s
repeatability (first vs second half of the used scans): rotvec [-0.144, -0.008, 0.031] deg, translation [-0.4, -4.6, 0.0] cm, time +0.9 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/zongyang/truck02/v010_axle/calibration.json (137 s)
frontend time base: template confirmed (candidate +0.0083); imu/ins lag +5.0 ms, corr 1.000, yaw rms 0.176 rad/s
```
