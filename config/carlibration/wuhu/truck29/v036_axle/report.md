# wuhu/truck29/v036_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck29/v035_driven
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v018_camtime -> wuhu/truck29/v019_camlidar -> wuhu/truck29/v022_camloop -> wuhu/truck29/v023_camtime -> wuhu/truck29/v024_camlidar -> wuhu/truck29/v027_camloop -> wuhu/truck29/v028_camloop -> wuhu/truck29/v029_camtime -> wuhu/truck29/v030_camlidar -> wuhu/truck29/v031_camloop -> wuhu/truck29/v032_camloop -> wuhu/truck29/v033_ground -> wuhu/truck29/v035_driven -> wuhu/truck29/v036_axle
- created: 2026-09-24T11:08:53+08:00
- note: fine-tune of the hand-tuned lineage: lever arm + LiDAR time shift applied, rotation kept

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
        -7.92,
        23.67,
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
        -7.92,
        23.67,
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
        -7.92,
        23.67,
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
        -7.92,
        23.67,
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
      -7.92,
      23.67,
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
    "solution_rotvec_deg": [
      -0.137,
      0.1516,
      -0.2315
    ],
    "solution_translation_cm": [
      -7.92,
      23.67,
      0.0
    ],
    "correction_rotvec_deg": [
      0.0,
      0.0,
      0.0
    ],
    "correction_translation_cm": [
      -7.92,
      23.67,
      0.0
    ],
    "correction_matrix": [
      0.9999999999999999,
      -8.673617379884035e-19,
      3.469446951953614e-18,
      -0.07916612601126735,
      2.168404344971009e-19,
      1.0,
      0.0,
      0.23671594419954456,
      -3.469446951953614e-18,
      3.469446951953614e-18,
      0.9999999999999999,
      6.938893903907228e-18,
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
      "before": 3.494,
      "after": 1.872,
      "ground_after": 1.651,
      "vertical_after": 3.259
    },
    "sqrt_information": {
      "rx": 5082.7,
      "ry": 2539.7,
      "rz": 2252.6,
      "tx": 21.0,
      "ty": 21.0,
      "dt": 380.8
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.002,
        -0.003,
        0.036
      ],
      "translation_cm": [
        9.1,
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
    "dof": "xy",
    "iterations": 11,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 54.1
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
        2.7604,
        6.0102,
        1.8179
      ],
      "rpy_deg": [
        -0.823,
        -3.153,
        -1.206
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0645,
        -0.2986,
        0.6511
      ],
      "rpy_deg": [
        -0.935,
        0.906,
        -91.146
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1014,
        5.7837,
        1.9598
      ],
      "rpy_deg": [
        -3.575,
        -0.606,
        178.839
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1122,
        8.0297,
        0.051
      ],
      "rpy_deg": [
        1.3,
        -1.67,
        90.16
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.005,
      8.0281,
      1.7519
    ],
    "rpy_deg": [
      -96.347,
      1.471,
      0.557
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
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v035_driven/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 600, skipped {'slow': 24, 'startup': 23, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 550 (9.8..119.6 s), points 4071144; dof xy + LiDAR time shift; cross-time gap > 1.5 s
residual median 3.49 -> 1.87 cm (ground 3.41 -> 1.65, n=82160; vertical 3.96 -> 3.26, n=7060); match time gap median 3.5 s; iterations 11
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.137, y(roll) +0.152, z(yaw) -0.232 deg; translation x -7.9, y +23.7 cm
applied (xy): rotvec x(pitch) +0.000, y(roll) +0.000, z(yaw) +0.000 deg; translation x -7.9, y +23.7, z +0.0 cm; LiDAR time shift -21.3 ms -> time_offsets.lidar -0.1463 s
repeatability (first vs second half of the used scans): rotvec [0.002, -0.003, 0.036] deg, translation [9.1, 6.9, 0.0] cm, time -0.5 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v036_axle/calibration.json (54 s)
```
