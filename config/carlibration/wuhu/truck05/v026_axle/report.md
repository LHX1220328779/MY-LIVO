# wuhu/truck05/v026_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck05/v025_driven
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v017_axle -> wuhu/truck05/v018_ground -> wuhu/truck05/v019_driven -> wuhu/truck05/v020_manual -> wuhu/truck05/v021_manual -> wuhu/truck05/v022_manual -> wuhu/truck05/v023_manual -> wuhu/truck05/v024_precheck -> wuhu/truck05/v025_driven -> wuhu/truck05/v026_axle
- created: 2026-09-29T14:46:24+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- window: [0.0, 1000000000.0]
- dof: xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -2.19,
        -1.33,
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
        -2.18,
        -1.33,
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
        -2.18,
        -1.33,
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
        -2.18,
        -1.33,
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
      -2.18,
      -1.33,
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
      "from": -0.0328,
      "to": -0.0267
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
      0.0243,
      0.0072,
      0.0219
    ],
    "solution_translation_cm": [
      -2.18,
      -1.33,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.0,
      0.0,
      -0.0
    ],
    "correction_translation_cm": [
      -2.18,
      -1.33,
      0.0
    ],
    "correction_matrix": [
      1.0,
      1.8973538018496328e-19,
      1.0842021724855044e-19,
      -0.021808605320614086,
      1.6263032587282567e-19,
      1.0,
      0.0,
      -0.013301238034181928,
      1.0842021724855044e-19,
      -2.7755575615628914e-17,
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.0061,
    "time_offsets_used": {
      "lidar": -0.0328,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 4.191,
      "after": 3.436,
      "ground_after": 3.356,
      "vertical_after": 3.123
    },
    "sqrt_information": {
      "rx": 2518.7,
      "ry": 2298.7,
      "rz": 2339.1,
      "tx": 71.0,
      "ty": 52.5,
      "dt": 391.9
    },
    "repeatability_halves": {
      "rotvec_deg": [
        -0.041,
        0.006,
        0.016
      ],
      "translation_cm": [
        -3.9,
        2.0,
        0.0
      ],
      "time_ms": -8.6
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0831,
        "median_cm": 4.707,
        "ground_cm": 4.879
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0439,
        "median_cm": 4.202,
        "ground_cm": 4.583
      },
      {
        "baseline_s": -0.12,
        "dt_s": -0.0165,
        "median_cm": 3.81,
        "ground_cm": 4.001
      },
      {
        "baseline_s": -0.09,
        "dt_s": -0.0059,
        "median_cm": 3.607,
        "ground_cm": 3.786
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0015,
        "median_cm": 3.513,
        "ground_cm": 3.502
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0052,
        "median_cm": 3.516,
        "ground_cm": 3.45
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.0066,
        "median_cm": 3.49,
        "ground_cm": 3.419
      },
      {
        "baseline_s": 0.03,
        "dt_s": 0.0084,
        "median_cm": 3.455,
        "ground_cm": 3.372
      },
      {
        "baseline_s": 0.06,
        "dt_s": 0.0134,
        "median_cm": 3.454,
        "ground_cm": 3.365
      },
      {
        "baseline_s": 0.09,
        "dt_s": 0.0253,
        "median_cm": 3.586,
        "ground_cm": 3.501
      }
    ],
    "time_profile_min_dt_s": 0.0134,
    "scans_used": 620,
    "window_s": [
      53.0,
      653.7
    ],
    "dof": "xy",
    "iterations": 24,
    "converged": false,
    "quality": "warn",
    "reasons": [
      "not converged in 24 iterations"
    ],
    "runtime_s": 579.0
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
        2.8233,
        5.5626,
        1.9472
      ],
      "rpy_deg": [
        0.749,
        -2.113,
        -1.153
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0238,
        -0.6768,
        0.8568
      ],
      "rpy_deg": [
        -1.037,
        1.29,
        -90.161
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0785,
        5.4797,
        1.9256
      ],
      "rpy_deg": [
        -0.481,
        -1.864,
        179.569
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0973,
        7.6438,
        0.5356
      ],
      "rpy_deg": [
        0.058,
        -5.285,
        89.869
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0014,
      7.6498,
      2.2531
    ],
    "rpy_deg": [
      -100.4,
      -0.011,
      -0.455
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.37,
      1849.37,
      1003.31,
      502.72
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
            1849.32,
            1831.26,
            1004.09,
            499.42
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            -0.002,
            7.529
          ],
          "ground_z_rear_m": -1.283
        },
        "square": {
          "intrinsics": [
            1849.37,
            1849.37,
            1003.31,
            502.72
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            0.001,
            7.535
          ],
          "ground_z_rear_m": -1.249
        }
      },
      "model": "square pixels (fx = fy)",
      "rms_px": 0.05,
      "camera_height_m": 3.502,
      "deviation_vs_truck29_defaults": {
        "fx_pct": 12.59,
        "fy_pct": -0.33,
        "cx_px": 32.3,
        "cy_px": -31.5
      }
    },
    "source": "supplier ground-distance curves equation_*m (square pixels (fx = fy), rms 0.05 px)"
  },
  "time_offsets": {
    "lidar": -0.0267,
    "camera": -0.0019,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0119,
    "camera_source": "operator set (livo_calib.py set): 相机偏移按同源时钟规则：camera_h = 家族值-0.115 + 打戳差0.1131 = -0.0019 ≈ 0，与 v016 时代 img=0 最优的实测互证"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v025_driven/calibration.json
bag=/home/project/data/haibo/wuhu/05/20260914/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.0328 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 3599, skipped {'slow': 1717, 'startup': 20, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 620 (53.0..653.7 s), points 7056221; dof xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.083s:4.71cm, -0.150/-0.044s:4.20cm, -0.120/-0.017s:3.81cm, -0.090/-0.006s:3.61cm, -0.060/+0.002s:3.51cm, -0.030/+0.005s:3.52cm, +0.000/+0.007s:3.49cm, +0.030/+0.008s:3.46cm, +0.060/+0.013s:3.45cm, +0.090/+0.025s:3.59cm
detected time shift +0.013 s (median 3.45 cm) -> final joint solve
residual median 4.19 -> 3.44 cm (ground 3.77 -> 3.36, n=27199; vertical 4.11 -> 3.12, n=9293); match time gap median 52.9 s; iterations 24 (NOT converged)
solution about the LiDAR (rear-axle frame): rotvec x(pitch) +0.024, y(roll) +0.007, z(yaw) +0.022 deg; translation x -2.2, y -1.3 cm
applied (xy): rotvec x(pitch) -0.000, y(roll) +0.000, z(yaw) -0.000 deg; translation x -2.2, y -1.3, z +0.0 cm; LiDAR time shift +6.1 ms -> time_offsets.lidar -0.0267 s
repeatability (first vs second half of the used scans): rotvec [-0.041, 0.006, 0.016] deg, translation [-3.9, 2.0, 0.0] cm, time -8.6 ms
quality: warn  (not converged in 24 iterations)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v026_axle/calibration.json (579 s)
```
