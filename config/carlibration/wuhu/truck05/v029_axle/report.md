# wuhu/truck05/v029_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck05/v001_import
- lineage: wuhu/truck05/v001_import -> wuhu/truck05/v029_axle
- created: 2026-10-14T13:54:38+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260826/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260826/ros2bag', 'kind': 'raw_split', 'segments': 24, 'duration_s': 719.99, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.27,
        -2.12,
        1.0
      ],
      "drpy_deg": [
        -0.072,
        -0.145,
        0.313
      ]
    },
    "back_lidar": {
      "dt_cm": [
        5.91,
        -3.73,
        1.14
      ],
      "drpy_deg": [
        0.144,
        -0.074,
        0.31
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        2.29,
        -5.26,
        -0.44
      ],
      "drpy_deg": [
        0.074,
        0.143,
        0.307
      ]
    },
    "front_lidar": {
      "dt_cm": [
        1.46,
        -3.87,
        0.0
      ],
      "drpy_deg": [
        -0.144,
        0.074,
        0.324
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      1.03,
      -3.59,
      0.02
    ],
    "drpy_deg": [
      -0.073,
      -0.144,
      0.31
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0368
    },
    "lidar_frontend": {
      "from": null,
      "to": -0.0418
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
      -0.0747,
      -0.1432,
      0.3099
    ],
    "solution_translation_cm": [
      1.46,
      -3.87,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.0747,
      -0.1432,
      0.3099
    ],
    "correction_translation_cm": [
      1.46,
      -3.87,
      0.0
    ],
    "correction_matrix": [
      0.9999822539773425,
      -0.005406416952699076,
      -0.0025024759988358574,
      0.057060727048815124,
      0.005409676645059964,
      0.9999845256239964,
      0.0012976560202157122,
      -0.038714290752214,
      0.002495421605074849,
      -0.001311170577948273,
      0.9999960268434712,
      0.010249760150455645,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.0882,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 4.385,
      "after": 3.518,
      "ground_after": 3.482,
      "vertical_after": 3.132
    },
    "sqrt_information": {
      "rx": 2394.4,
      "ry": 2140.8,
      "rz": 2266.2,
      "tx": 67.8,
      "ty": 45.1,
      "dt": 371.0
    },
    "repeatability_halves": {
      "rotvec_deg": [
        -0.032,
        -0.025,
        0.015
      ],
      "translation_cm": [
        1.0,
        0.4,
        0.0
      ],
      "time_ms": -3.0
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0807,
        "median_cm": 5.505,
        "ground_cm": 5.518
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0417,
        "median_cm": 5.09,
        "ground_cm": 5.126
      },
      {
        "baseline_s": -0.12,
        "dt_s": -0.0047,
        "median_cm": 4.713,
        "ground_cm": 4.69
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0311,
        "median_cm": 4.25,
        "ground_cm": 4.261
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0561,
        "median_cm": 3.861,
        "ground_cm": 3.993
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0716,
        "median_cm": 3.897,
        "ground_cm": 4.053
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.0782,
        "median_cm": 3.817,
        "ground_cm": 4.02
      },
      {
        "baseline_s": 0.03,
        "dt_s": 0.0822,
        "median_cm": 3.853,
        "ground_cm": 3.886
      }
    ],
    "time_profile_min_dt_s": 0.0782,
    "scans_used": 619,
    "window_s": [
      53.2,
      653.9
    ],
    "dof": "rpy_xy",
    "iterations": 24,
    "converged": false,
    "quality": "warn",
    "reasons": [
      "time shift +88 ms is a large change vs the parent offset (candidates/longer recording confirms it)",
      "not converged in 24 iterations"
    ],
    "runtime_s": 486.0
  },
  "frontend_sync": {
    "imu_ins_lag_s": -0.004999999999999727,
    "corr": 0.9986,
    "yaw_rms_rad_s": 0.1104,
    "template": -0.125,
    "decision": "replaced template with -0.0418"
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
        2.8011,
        5.4403,
        1.9215
      ],
      "rpy_deg": [
        0.967,
        -2.82,
        -1.109
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1268,
        -0.7633,
        0.7692
      ],
      "rpy_deg": [
        -1.67,
        0.104,
        -90.311
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0098,
        5.375,
        1.9423
      ],
      "rpy_deg": [
        -0.684,
        -2.188,
        179.915
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
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0418,
    "lidar_frontend_source": "axle: lidar -0.0368 + imu/ins clock lag -0.0050 (corr 0.999); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v001_import/calibration.json
bag=/home/project/data/haibo/wuhu/05/20260826/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 3599, skipped {'slow': 1721, 'startup': 20, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 619 (53.2..653.9 s), points 7050465; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.081s:5.50cm, -0.150/-0.042s:5.09cm, -0.120/-0.005s:4.71cm, -0.090/+0.031s:4.25cm, -0.060/+0.056s:3.86cm, -0.030/+0.072s:3.90cm, +0.000/+0.078s:3.82cm, +0.030/+0.082s:3.85cm
detected time shift +0.078 s (median 3.82 cm) -> final joint solve
residual median 4.38 -> 3.52 cm (ground 4.22 -> 3.48, n=24985; vertical 3.78 -> 3.13, n=9852); match time gap median 49.7 s; iterations 24 (NOT converged)
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.075, y(roll) -0.143, z(yaw) +0.310 deg; translation x +1.5, y -3.9 cm
applied (rpy_xy): rotvec x(pitch) -0.075, y(roll) -0.143, z(yaw) +0.310 deg; translation x +1.5, y -3.9, z +0.0 cm; LiDAR time shift +88.2 ms -> time_offsets.lidar -0.0368 s
repeatability (first vs second half of the used scans): rotvec [-0.032, -0.025, 0.015] deg, translation [1.0, 0.4, 0.0] cm, time -3.0 ms
quality: warn  (time shift +88 ms is a large change vs the parent offset (candidates/longer recording confirms it); not converged in 24 iterations)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v029_axle/calibration.json (486 s)
frontend time base: replaced template with -0.0418; imu/ins lag -5.0 ms, corr 0.999, yaw rms 0.110 rad/s
```
