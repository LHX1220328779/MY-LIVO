# wuhu/truck29-0930/v013_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck29-0930/v002_import
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v013_axle
- created: 2026-10-19T17:26:50+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- window: [0.0, 180.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -11.77,
        40.12,
        -10.39
      ],
      "drpy_deg": [
        -0.563,
        2.465,
        1.118
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -5.38,
        33.68,
        7.31
      ],
      "drpy_deg": [
        -2.466,
        -0.558,
        1.045
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -11.12,
        29.17,
        15.18
      ],
      "drpy_deg": [
        0.579,
        -2.462,
        1.044
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -23.65,
        33.34,
        0.0
      ],
      "drpy_deg": [
        2.479,
        0.502,
        0.997
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -16.26,
      34.9,
      -0.32
    ],
    "drpy_deg": [
      -0.481,
      2.482,
      1.072
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0372
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": -0.0322
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
      -0.4779,
      2.4827,
      1.0755
    ],
    "solution_translation_cm": [
      -23.65,
      33.34,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.4779,
      2.4827,
      1.0755
    ],
    "correction_translation_cm": [
      -23.65,
      33.34,
      0.0
    ],
    "correction_matrix": [
      0.9988852741961702,
      -0.01894408940372862,
      0.04323575454073103,
      -0.09434261510475189,
      0.01858276717801059,
      0.9997890917968315,
      0.008743722781221982,
      0.3325402135931741,
      -0.04339227763151709,
      -0.007930535967420223,
      0.9990266347006076,
      0.06506170099531208,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.0878,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 2.276,
      "after": 2.008,
      "ground_after": 1.682,
      "vertical_after": 3.062
    },
    "sqrt_information": {
      "rx": 4237.3,
      "ry": 2509.7,
      "rz": 1729.0,
      "tx": 35.3,
      "ty": 25.2,
      "dt": 352.8
    },
    "repeatability_halves": {
      "rotvec_deg": [
        -0.017,
        -0.042,
        0.147
      ],
      "translation_cm": [
        -0.6,
        2.3,
        0.0
      ],
      "time_ms": 6.4
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": 0.0326,
        "median_cm": 3.562,
        "ground_cm": 3.2
      },
      {
        "baseline_s": -0.15,
        "dt_s": 0.0553,
        "median_cm": 3.055,
        "ground_cm": 2.684
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0684,
        "median_cm": 2.589,
        "ground_cm": 2.242
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0764,
        "median_cm": 2.326,
        "ground_cm": 1.989
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0822,
        "median_cm": 2.277,
        "ground_cm": 1.975
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0845,
        "median_cm": 2.447,
        "ground_cm": 2.164
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.0847,
        "median_cm": 2.578,
        "ground_cm": 2.288
      }
    ],
    "time_profile_min_dt_s": 0.0822,
    "scans_used": 375,
    "window_s": [
      21.6,
      179.7
    ],
    "dof": "rpy_xy",
    "iterations": 12,
    "converged": true,
    "quality": "warn",
    "reasons": [
      "time shift +88 ms is a large change vs the parent offset (candidates/longer recording confirms it)"
    ],
    "runtime_s": 98.7
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.005000000000000282,
    "corr": 0.9981,
    "yaw_rms_rad_s": 0.0658,
    "template": -0.125,
    "decision": "replaced template with -0.0322"
  },
  "lateral": {
    "decision": "skipped (data window 0-180 s is respected; --lateral-whole-recording opts in)"
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
        2.7341,
        5.858,
        1.8198
      ],
      "rpy_deg": [
        -0.988,
        -2.994,
        -1.378
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1234,
        -0.3683,
        0.6992
      ],
      "rpy_deg": [
        -1.148,
        0.44,
        -91.346
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1208,
        5.5906,
        2.0049
      ],
      "rpy_deg": [
        3.298,
        -4.373,
        178.208
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1296,
        7.9462,
        0.051
      ],
      "rpy_deg": [
        1.477,
        -1.57,
        89.955
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0171,
      7.9472,
      1.7516
    ],
    "rpy_deg": [
      -96.467,
      1.661,
      0.456
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
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0372,
    "lidar_frontend": -0.0322,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0372 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v002_import/calibration.json
bag=/home/project/data/haibo/wuhu/29/20261013/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 897, skipped {'slow': 124, 'startup': 22, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 375 (21.6..179.7 s), points 3583505; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/+0.033s:3.56cm, -0.150/+0.055s:3.06cm, -0.120/+0.068s:2.59cm, -0.090/+0.076s:2.33cm, -0.060/+0.082s:2.28cm, -0.030/+0.085s:2.45cm, +0.000/+0.085s:2.58cm
detected time shift +0.082 s (median 2.28 cm) -> final joint solve
residual median 2.28 -> 2.01 cm (ground 1.84 -> 1.68, n=69691; vertical 3.48 -> 3.06, n=10784); match time gap median 4.2 s; iterations 12
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.478, y(roll) +2.483, z(yaw) +1.075 deg; translation x -23.6, y +33.3 cm
applied (rpy_xy): rotvec x(pitch) -0.478, y(roll) +2.483, z(yaw) +1.075 deg; translation x -23.6, y +33.3, z +0.0 cm; LiDAR time shift +87.8 ms -> time_offsets.lidar -0.0372 s
repeatability (first vs second half of the used scans): rotvec [-0.017, -0.042, 0.147] deg, translation [-0.6, 2.3, 0.0] cm, time +6.4 ms
quality: warn  (time shift +88 ms is a large change vs the parent offset (candidates/longer recording confirms it))
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v013_axle/calibration.json (99 s)
frontend time base: replaced template with -0.0322; imu/ins lag +5.0 ms, corr 0.998, yaw rms 0.066 rad/s
lateral lever arm (whole recording): skipped, the stage stays inside the data window 0-180 s
```
