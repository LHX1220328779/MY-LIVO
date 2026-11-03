# wuhu/truck28/v031_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck28/v023_import
- lineage: wuhu/truck28/v023_import -> wuhu/truck28/v031_axle
- created: 2026-10-26T18:04:25+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu/28/测试/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/测试/ros2bag', 'kind': 'raw_split', 'segments': 4, 'duration_s': 119.99, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -15.46,
        30.15,
        -15.11
      ],
      "drpy_deg": [
        0.749,
        2.507,
        0.583
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -14.03,
        28.67,
        -10.52
      ],
      "drpy_deg": [
        -2.514,
        0.735,
        0.437
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -14.77,
        24.25,
        9.95
      ],
      "drpy_deg": [
        -0.685,
        -2.525,
        0.562
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -24.52,
        29.51,
        0.0
      ],
      "drpy_deg": [
        2.507,
        -0.758,
        0.43
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -16.76,
      27.44,
      -1.42
    ],
    "drpy_deg": [
      0.796,
      2.492,
      0.552
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.1609
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": -0.1559
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
      0.7694,
      2.5003,
      0.567
    ],
    "solution_translation_cm": [
      -24.52,
      29.51,
      0.0
    ],
    "correction_rotvec_deg": [
      0.7694,
      2.5003,
      0.567
    ],
    "correction_translation_cm": [
      -24.52,
      29.51,
      0.0
    ],
    "correction_matrix": [
      0.9989990847825568,
      -0.009599255524127564,
      0.04368847556276981,
      -0.16510319319077985,
      0.01018514241557465,
      0.9998608990606442,
      -0.013207778148335202,
      0.29303371802897704,
      -0.04355561361742813,
      0.013639531627724133,
      0.9989578928560439,
      -0.09965879833576569,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": -0.0359,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 2.996,
      "after": 1.522,
      "ground_after": 1.245,
      "vertical_after": 2.329
    },
    "sqrt_information": {
      "rx": 1906.9,
      "ry": 3325.0,
      "rz": 1045.1,
      "tx": 40.7,
      "ty": 42.4,
      "dt": 294.8
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.024,
        -0.042,
        0.055
      ],
      "translation_cm": [
        3.4,
        2.5,
        0.0
      ],
      "time_ms": 3.1
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.1093,
        "median_cm": 3.607,
        "ground_cm": 4.712
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0767,
        "median_cm": 3.492,
        "ground_cm": 4.374
      },
      {
        "baseline_s": -0.12,
        "dt_s": -0.0565,
        "median_cm": 3.288,
        "ground_cm": 3.921
      },
      {
        "baseline_s": -0.09,
        "dt_s": -0.045,
        "median_cm": 3.038,
        "ground_cm": 3.368
      },
      {
        "baseline_s": -0.06,
        "dt_s": -0.0398,
        "median_cm": 2.85,
        "ground_cm": 3.003
      },
      {
        "baseline_s": -0.03,
        "dt_s": -0.038,
        "median_cm": 2.834,
        "ground_cm": 3.017
      },
      {
        "baseline_s": 0.0,
        "dt_s": -0.0352,
        "median_cm": 2.902,
        "ground_cm": 3.127
      }
    ],
    "time_profile_min_dt_s": -0.038,
    "scans_used": 281,
    "window_s": [
      1.2,
      119.6
    ],
    "dof": "rpy_xy",
    "iterations": 12,
    "converged": true,
    "quality": "ok",
    "reasons": [],
    "runtime_s": 99.3
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.005000000000000282,
    "corr": 0.9972,
    "yaw_rms_rad_s": 0.0507,
    "template": -0.125,
    "decision": "replaced template with -0.1559"
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
        2.7306,
        5.8576,
        1.3771
      ],
      "rpy_deg": [
        0.715,
        2.495,
        -0.179
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2169,
        -0.3327,
        0.3252
      ],
      "rpy_deg": [
        -2.446,
        3.349,
        -90.585
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0704,
        5.6266,
        1.6132
      ],
      "rpy_deg": [
        -1.503,
        -0.835,
        178.366
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.144,
        7.9129,
        -0.1568
      ],
      "rpy_deg": [
        2.014,
        -3.526,
        89.96
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1923,
      7.8117,
      1.5926
    ],
    "rpy_deg": [
      -100.296,
      0.152,
      0.901
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1637.05,
      1851.4,
      1029.33,
      495.55
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "intrinsics_check": {
      "method": "supplier ground-distance curves (midrange_camera[0].equation_*m)",
      "curves": 10,
      "distances_m": [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        70.0,
        80.0,
        90.0,
        100.0
      ],
      "fits": {
        "free": {
          "intrinsics": [
            1637.05,
            1851.4,
            1029.33,
            495.55
          ],
          "rms_px": 0.014,
          "circle_centre_rear_m": [
            0.0,
            7.38
          ],
          "ground_z_rear_m": -1.22
        },
        "square": {
          "intrinsics": [
            1639.36,
            1639.36,
            864.27,
            446.63
          ],
          "rms_px": 1.072,
          "circle_centre_rear_m": [
            0.806,
            7.314
          ],
          "ground_z_rear_m": -1.588
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.014,
      "camera_height_m": 2.827,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.34,
        "fy_pct": -0.22,
        "cx_px": 58.3,
        "cy_px": -38.7
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.014 px)"
  },
  "time_offsets": {
    "lidar": -0.1609,
    "lidar_frontend": -0.1559,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.1609 + imu/ins clock lag +0.0050 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v023_import/calibration.json
bag=/home/project/data/haibo/wuhu/28/测试/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 0.5 m/s, >= 1.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 600, skipped {'slow': 312, 'startup': 4, 'not_rtk_fixed': 0, 'outside_ins': 3}
scans used 281 (1.2..119.6 s), points 3864494; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.109s:3.61cm, -0.150/-0.077s:3.49cm, -0.120/-0.057s:3.29cm, -0.090/-0.045s:3.04cm, -0.060/-0.040s:2.85cm, -0.030/-0.038s:2.83cm, +0.000/-0.035s:2.90cm
detected time shift -0.038 s (median 2.83 cm) -> final joint solve
residual median 3.00 -> 1.52 cm (ground 2.95 -> 1.24, n=49391; vertical 2.72 -> 2.33, n=10417); match time gap median 19.9 s; iterations 12
solution about the LiDAR (rear-axle frame): rotvec x(pitch) +0.769, y(roll) +2.500, z(yaw) +0.567 deg; translation x -24.5, y +29.5 cm
applied (rpy_xy): rotvec x(pitch) +0.769, y(roll) +2.500, z(yaw) +0.567 deg; translation x -24.5, y +29.5, z +0.0 cm; LiDAR time shift -35.9 ms -> time_offsets.lidar -0.1609 s
repeatability (first vs second half of the used scans): rotvec [0.024, -0.042, 0.055] deg, translation [3.4, 2.5, 0.0] cm, time +3.1 ms
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v031_axle/calibration.json (99 s)
frontend time base: replaced template with -0.1559; imu/ins lag +5.0 ms, corr 0.997, yaw rms 0.051 rad/s
```
