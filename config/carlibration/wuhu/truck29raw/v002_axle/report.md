# wuhu/truck29raw/v002_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck29raw/v001_import
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle
- created: 2026-09-24T10:15:53+08:00
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
        -9.84,
        48.11,
        -10.02
      ],
      "drpy_deg": [
        -0.602,
        2.438,
        1.087
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -3.75,
        41.81,
        7.85
      ],
      "drpy_deg": [
        -2.439,
        -0.595,
        1.011
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -9.35,
        37.49,
        15.29
      ],
      "drpy_deg": [
        0.602,
        -2.438,
        1.032
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -21.49,
        41.43,
        0.0
      ],
      "drpy_deg": [
        2.454,
        0.539,
        0.963
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -14.18,
      43.09,
      -0.31
    ],
    "drpy_deg": [
      -0.519,
      2.456,
      1.037
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.1463
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
    "correction_rotvec_deg": [
      -0.5148,
      2.4576,
      1.0403
    ],
    "correction_translation_cm": [
      -21.49,
      41.42,
      0.0
    ],
    "correction_matrix": [
      0.9989154721094147,
      -0.018343117310239167,
      0.042794971991666715,
      -0.07731640983499793,
      0.01795777197242311,
      0.9997948246931007,
      0.009371602994094713,
      0.4134029379894084,
      -0.04295809593326024,
      -0.008592936880675542,
      0.9990399208387792,
      0.07005735137403948,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": -0.0213,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 8.259,
      "after": 1.872,
      "ground_after": 1.654,
      "vertical_after": 3.252
    },
    "sqrt_information": {
      "rx": 5082.1,
      "ry": 2540.3,
      "rz": 2251.3,
      "tx": 21.0,
      "ty": 21.1,
      "dt": 380.9
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.002,
        -0.003,
        0.036
      ],
      "translation_cm": [
        9.0,
        6.9,
        0.0
      ],
      "time_ms": -0.5
    },
    "scans_used": 550,
    "window_s": [
      9.8,
      119.6
    ],
    "dof": "rpy_xy",
    "iterations": 14,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 61.1
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
        2.7333,
        5.9263,
        1.8129
      ],
      "rpy_deg": [
        -0.891,
        -3.27,
        -1.468
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1071,
        -0.287,
        0.7046
      ],
      "rpy_deg": [
        -1.121,
        0.403,
        -91.38
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1314,
        5.7145,
        1.9876
      ],
      "rpy_deg": [
        -3.21,
        -2.038,
        178.484
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.108,
        8.0271,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.533,
        89.921
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0037,
      8.0291,
      1.7517
    ],
    "rpy_deg": [
      -96.505,
      1.635,
      0.421
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
    "lidar": -0.1463,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v001_import/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 600, skipped {'slow': 24, 'startup': 23, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 550 (9.8..119.6 s), points 4071144; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
residual median 8.26 -> 1.87 cm (ground 9.59 -> 1.65, n=82178; vertical 4.85 -> 3.25, n=7043); match time gap median 3.5 s; iterations 14
correction about the LiDAR (rear-axle frame): rotvec x(pitch) -0.515, y(roll) +2.458, z(yaw) +1.040 deg; translation x -21.5, y +41.4, z +0.0 cm; LiDAR time shift -21.3 ms -> time_offsets.lidar -0.1463 s
repeatability (first vs second half of the used scans): rotvec [0.002, -0.003, 0.036] deg, translation [9.0, 6.9, 0.0] cm, time -0.5 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v002_axle/calibration.json (61 s)
```
