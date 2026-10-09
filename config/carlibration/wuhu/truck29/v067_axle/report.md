# wuhu/truck29/v067_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck29/v066_driven
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle -> wuhu/truck29/v057_ground -> wuhu/truck29/v058_driven -> wuhu/truck29/v059_camtime -> wuhu/truck29/v060_camlidar -> wuhu/truck29/v061_camloop -> wuhu/truck29/v062_manual -> wuhu/truck29/v063_manual -> wuhu/truck29/v065_driven -> wuhu/truck29/v066_driven -> wuhu/truck29/v067_axle
- created: 2026-10-09T08:33:55+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- window: [0.0, 1000000000.0]
- dof: xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -0.05,
        0.04,
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
        -0.05,
        0.04,
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
        -0.06,
        0.04,
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
        -0.06,
        0.04,
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
      -0.05,
      0.04,
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
      "from": -0.1459,
      "to": -0.1462
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
      0.0,
      -0.0001,
      -0.0005
    ],
    "solution_translation_cm": [
      -0.05,
      0.04,
      0.0
    ],
    "correction_rotvec_deg": [
      0.0,
      0.0,
      -0.0
    ],
    "correction_translation_cm": [
      -0.05,
      0.04,
      0.0
    ],
    "correction_matrix": [
      0.9999999999999999,
      1.0842021724855044e-19,
      3.469446951953614e-18,
      -0.0005283988536551043,
      0.0,
      1.0,
      0.0,
      0.0004483806108535049,
      3.469446951953614e-18,
      3.469446951953614e-18,
      1.0,
      2.7755575615628914e-17,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": -0.0003,
    "time_offsets_used": {
      "lidar": -0.1459,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 1.961,
      "after": 1.882,
      "ground_after": 1.655,
      "vertical_after": 3.241
    },
    "sqrt_information": {
      "rx": 5081.1,
      "ry": 2551.3,
      "rz": 2254.5,
      "tx": 20.8,
      "ty": 20.9,
      "dt": 382.2
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.001,
        0.001,
        0.054
      ],
      "translation_cm": [
        7.8,
        6.9,
        0.0
      ],
      "time_ms": -1.0
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0647,
        "median_cm": 4.642,
        "ground_cm": 5.182
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.042,
        "median_cm": 4.197,
        "ground_cm": 4.429
      },
      {
        "baseline_s": -0.12,
        "dt_s": -0.0235,
        "median_cm": 3.634,
        "ground_cm": 3.596
      },
      {
        "baseline_s": -0.09,
        "dt_s": -0.0095,
        "median_cm": 2.907,
        "ground_cm": 2.733
      },
      {
        "baseline_s": -0.06,
        "dt_s": -0.0016,
        "median_cm": 2.225,
        "ground_cm": 2.0
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0007,
        "median_cm": 1.938,
        "ground_cm": 1.715
      },
      {
        "baseline_s": 0.0,
        "dt_s": -0.0003,
        "median_cm": 1.88,
        "ground_cm": 1.658
      },
      {
        "baseline_s": 0.03,
        "dt_s": -0.0005,
        "median_cm": 1.894,
        "ground_cm": 1.671
      }
    ],
    "time_profile_min_dt_s": -0.0003,
    "scans_used": 550,
    "window_s": [
      9.8,
      119.6
    ],
    "dof": "xy",
    "iterations": 12,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 224.5
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.012500000000000289,
    "corr": 0.9978,
    "yaw_rms_rad_s": 0.0596,
    "template": -0.125,
    "decision": "template confirmed (candidate -0.1337)"
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
        2.7559,
        6.0141,
        1.809
      ],
      "rpy_deg": [
        -0.944,
        -2.957,
        -1.443
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0883,
        -0.2263,
        0.6288
      ],
      "rpy_deg": [
        -1.045,
        0.749,
        -91.443
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0911,
        5.7896,
        1.9503
      ],
      "rpy_deg": [
        -3.471,
        -0.723,
        178.598
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1094,
        8.0405,
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
      0.0024,
      8.0426,
      1.7517
    ],
    "rpy_deg": [
      -96.512,
      1.62,
      0.291
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
    "lidar": -0.1462,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "operator set (livo_calib.py set): 时间偏移改回 IMU 硬件对齐对（lidar -0.125 / camera -0.115）：axle 解 -0.1459 含 INS 链路延迟，喂给前端后 21ms 错位打花高角速率段的去畸变与颜色采样（灯柱白度 51.5%→79.3%，牌面着色同步改善）；融合精度在该区间平坦（LIVO 120s xy max 7.12→7.08 cm，达标）。证据链见 docs/mcp-color-regression-time-offsets.md",
    "lidar_frontend": -0.125
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v066_driven/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1459 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 600, skipped {'slow': 25, 'startup': 22, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 550 (9.8..119.6 s), points 4071144; dof xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.065s:4.64cm, -0.150/-0.042s:4.20cm, -0.120/-0.024s:3.63cm, -0.090/-0.009s:2.91cm, -0.060/-0.002s:2.23cm, -0.030/+0.001s:1.94cm, +0.000/-0.000s:1.88cm, +0.030/-0.001s:1.89cm
detected time shift -0.000 s (median 1.88 cm) -> final joint solve
residual median 1.96 -> 1.88 cm (ground 1.70 -> 1.65, n=82180; vertical 3.43 -> 3.24, n=7005); match time gap median 3.5 s; iterations 12
solution about the LiDAR (rear-axle frame): rotvec x(pitch) +0.000, y(roll) -0.000, z(yaw) -0.001 deg; translation x -0.1, y +0.0 cm
applied (xy): rotvec x(pitch) +0.000, y(roll) +0.000, z(yaw) -0.000 deg; translation x -0.1, y +0.0, z +0.0 cm; LiDAR time shift -0.3 ms -> time_offsets.lidar -0.1462 s
repeatability (first vs second half of the used scans): rotvec [0.001, 0.001, 0.054] deg, translation [7.8, 6.9, 0.0] cm, time -1.0 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v067_axle/calibration.json (225 s)
frontend time base: template confirmed (candidate -0.1337); imu/ins lag +12.5 ms, corr 0.998, yaw rms 0.060 rad/s
```
