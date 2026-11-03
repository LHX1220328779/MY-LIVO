# xingye/truck13/v002_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: xingye/truck13/v001_import
- lineage: xingye/truck13/v001_import -> xingye/truck13/v002_axle
- created: 2026-10-28T11:03:20+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/xingye/13/20261027/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/13/20261027/ros2bag', 'kind': 'raw_split', 'segments': 68, 'duration_s': 1020.12, 'has_camera': True}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.7,
        7.99,
        -1.75
      ],
      "drpy_deg": [
        -0.636,
        0.69,
        0.84
      ]
    },
    "back_lidar": {
      "dt_cm": [
        13.08,
        4.04,
        12.32
      ],
      "drpy_deg": [
        -0.515,
        -0.785,
        0.865
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.75,
        2.41,
        1.77
      ],
      "drpy_deg": [
        0.873,
        -0.344,
        0.829
      ]
    },
    "front_lidar": {
      "dt_cm": [
        0.24,
        4.49,
        0.0
      ],
      "drpy_deg": [
        0.523,
        0.78,
        0.853
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      0.93,
      5.56,
      -0.02
    ],
    "drpy_deg": [
      -0.791,
      0.507,
      0.818
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0502
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": -0.0427
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
      -0.7817,
      0.5189,
      0.8336
    ],
    "solution_translation_cm": [
      0.24,
      4.49,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.7817,
      0.5189,
      0.8336
    ],
    "correction_translation_cm": [
      0.24,
      4.49,
      0.0
    ],
    "correction_matrix": [
      0.9998531506696179,
      -0.014610151707729646,
      0.008956593276141534,
      0.09832339693482438,
      0.01448658793098996,
      0.9998010828290799,
      0.0137088855825933,
      0.03732679233886227,
      -0.009155100554051561,
      -0.013577121965868254,
      0.99986591395695,
      0.0946244837750011,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.0748,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 2.838,
      "after": 2.221,
      "ground_after": 2.075,
      "vertical_after": 3.084
    },
    "sqrt_information": {
      "rx": 4253.2,
      "ry": 3842.8,
      "rz": 2027.0,
      "tx": 79.2,
      "ty": 48.1,
      "dt": 381.3
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.008,
        0.008,
        0.121
      ],
      "translation_cm": [
        -3.1,
        -7.5,
        0.0
      ],
      "time_ms": -2.5
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0372,
        "median_cm": 4.619,
        "ground_cm": 4.289
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0018,
        "median_cm": 3.953,
        "ground_cm": 3.68
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0311,
        "median_cm": 3.28,
        "ground_cm": 3.06
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0521,
        "median_cm": 2.783,
        "ground_cm": 2.605
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0658,
        "median_cm": 2.561,
        "ground_cm": 2.397
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.071,
        "median_cm": 2.654,
        "ground_cm": 2.493
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.0708,
        "median_cm": 2.796,
        "ground_cm": 2.623
      }
    ],
    "time_profile_min_dt_s": 0.0658,
    "scans_used": 654,
    "window_s": [
      28.4,
      952.3
    ],
    "dof": "rpy_xy",
    "iterations": 22,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 273.7
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.007500000000000284,
    "corr": 0.9972,
    "yaw_rms_rad_s": 0.0691,
    "template": -0.125,
    "decision": "replaced template with -0.0427"
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
        2.0223,
        6.9705,
        1.0551
      ],
      "rpy_deg": [
        -2.009,
        -0.247,
        14.202
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0796,
        -1.9239,
        0.5417
      ],
      "rpy_deg": [
        -0.135,
        -3.895,
        -89.936
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8361,
        6.9237,
        1.099
      ],
      "rpy_deg": [
        1.254,
        -0.462,
        168.334
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0002,
        7.0096,
        0.6541
      ],
      "rpy_deg": [
        0.426,
        2.516,
        90.688
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0123,
      7.0409,
      1.4569
    ],
    "rpy_deg": [
      -98.675,
      1.353,
      -0.512
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.15,
      1855.56,
      963.247,
      544.29
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
    "lidar": -0.0502,
    "lidar_frontend": -0.0427,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0502 + imu/ins clock lag +0.0075 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck13/v001_import/calibration.json
bag=/home/project/data/haibo/xingye/13/20261027/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 5100, skipped {'slow': 1816, 'startup': 14, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 654 (28.4..952.3 s), points 5274056; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.037s:4.62cm, -0.150/-0.002s:3.95cm, -0.120/+0.031s:3.28cm, -0.090/+0.052s:2.78cm, -0.060/+0.066s:2.56cm, -0.030/+0.071s:2.65cm, +0.000/+0.071s:2.80cm
detected time shift +0.066 s (median 2.56 cm) -> final joint solve
residual median 2.84 -> 2.22 cm (ground 2.47 -> 2.07, n=81232; vertical 5.29 -> 3.08, n=5127); match time gap median 255.4 s; iterations 22
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.782, y(roll) +0.519, z(yaw) +0.834 deg; translation x +0.2, y +4.5 cm
applied (rpy_xy): rotvec x(pitch) -0.782, y(roll) +0.519, z(yaw) +0.834 deg; translation x +0.2, y +4.5, z +0.0 cm; LiDAR time shift +74.8 ms -> time_offsets.lidar -0.0502 s
repeatability (first vs second half of the used scans): rotvec [0.008, 0.008, 0.121] deg, translation [-3.1, -7.5, 0.0] cm, time -2.5 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/xingye/truck13/v002_axle/calibration.json (274 s)
frontend time base: replaced template with -0.0427; imu/ins lag +7.5 ms, corr 0.997, yaw rms 0.069 rad/s
```
