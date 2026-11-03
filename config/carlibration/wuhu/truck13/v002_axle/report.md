# wuhu/truck13/v002_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: wuhu/truck13/v001_import
- lineage: wuhu/truck13/v001_import -> wuhu/truck13/v002_axle
- created: 2026-10-26T17:23:54+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/wuhu/13/1/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/13/1/ros2bag', 'kind': 'raw_split', 'segments': 2, 'duration_s': 60.0, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -1.81,
        24.32,
        4.87
      ],
      "drpy_deg": [
        -0.682,
        -0.481,
        -0.507
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -6.34,
        25.14,
        10.16
      ],
      "drpy_deg": [
        0.482,
        -0.682,
        -0.497
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -1.86,
        29.68,
        0.07
      ],
      "drpy_deg": [
        0.691,
        0.468,
        359.455
      ]
    },
    "front_lidar": {
      "dt_cm": [
        1.5,
        24.81,
        0.0
      ],
      "drpy_deg": [
        -0.468,
        0.691,
        -0.501
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      0.15,
      26.86,
      -0.05
    ],
    "drpy_deg": [
      -0.656,
      -0.516,
      -0.538
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": 0.021
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": 0.036
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
      -0.6901,
      -0.4695,
      -0.5174
    ],
    "solution_translation_cm": [
      1.51,
      24.81,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.6901,
      -0.4695,
      -0.5174
    ],
    "correction_translation_cm": [
      1.51,
      24.81,
      0.0
    ],
    "correction_matrix": [
      0.9999256521721785,
      0.009079812803777385,
      -0.008139233839289016,
      -0.04931908259604072,
      -0.008981127038318411,
      0.9998866959508094,
      0.012080340131583014,
      0.24303579283704746,
      0.008247998858138459,
      -0.012006342491429674,
      0.9998939034991736,
      0.0904440331681785,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.146,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 1.823,
      "after": 1.53,
      "ground_after": 1.513,
      "vertical_after": 4.505
    },
    "sqrt_information": {
      "rx": 2573.9,
      "ry": 3735.9,
      "rz": 343.2,
      "tx": 13.1,
      "ty": 13.2,
      "dt": 246.5
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.04,
        0.205,
        0.319
      ],
      "translation_cm": [
        4.3,
        24.2,
        0.0
      ],
      "time_ms": -0.3
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": 0.0826,
        "median_cm": 2.168,
        "ground_cm": 2.141
      },
      {
        "baseline_s": -0.15,
        "dt_s": 0.1013,
        "median_cm": 1.902,
        "ground_cm": 1.879
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.1165,
        "median_cm": 1.813,
        "ground_cm": 1.792
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.1291,
        "median_cm": 1.851,
        "ground_cm": 1.833
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.1372,
        "median_cm": 1.949,
        "ground_cm": 1.928
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.1389,
        "median_cm": 2.152,
        "ground_cm": 2.13
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.1385,
        "median_cm": 2.209,
        "ground_cm": 2.181
      }
    ],
    "time_profile_min_dt_s": 0.1165,
    "scans_used": 85,
    "window_s": [
      30.9,
      50.9
    ],
    "dof": "rpy_xy",
    "iterations": 17,
    "converged": true,
    "quality": "warn",
    "reasons": [
      "halves disagree (few turns/structure: use a longer recording with turns)"
    ],
    "runtime_s": 50.0
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.015000000000000291,
    "corr": 0.996,
    "yaw_rms_rad_s": 0.0443,
    "template": -0.125,
    "decision": "replaced template with +0.0360"
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
        2.8383,
        5.6644,
        2.2409
      ],
      "rpy_deg": [
        -1.328,
        -1.106,
        -1.194
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0126,
        -0.65,
        0.826
      ],
      "rpy_deg": [
        -0.664,
        2.11,
        -91.251
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0644,
        5.661,
        2.2391
      ],
      "rpy_deg": [
        1.29,
        -2.074,
        179.802
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0242,
        7.7501,
        0.4596
      ],
      "rpy_deg": [
        0.215,
        -1.686,
        89.873
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0755,
      7.8739,
      2.2451
    ],
    "rpy_deg": [
      -102.142,
      1.542,
      -4.24
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.47862,
      1848.72893,
      1004.25076,
      503.038474
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
    "lidar": 0.021,
    "lidar_frontend": 0.036,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar +0.0210 + imu/ins clock lag +0.0150 (corr 0.996); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck13/v001_import/calibration.json
bag=/home/project/data/haibo/wuhu/13/1/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 0.5 m/s, >= 1.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 299, skipped {'slow': 208, 'startup': 4, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 85 (30.9..50.9 s), points 477621; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/+0.083s:2.17cm, -0.150/+0.101s:1.90cm, -0.120/+0.117s:1.81cm, -0.090/+0.129s:1.85cm, -0.060/+0.137s:1.95cm, -0.030/+0.139s:2.15cm, +0.000/+0.139s:2.21cm
detected time shift +0.117 s (median 1.81 cm) -> final joint solve
residual median 1.82 -> 1.53 cm (ground 1.80 -> 1.51, n=147345; vertical 6.51 -> 4.50, n=377); match time gap median 5.1 s; iterations 17
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.690, y(roll) -0.469, z(yaw) -0.517 deg; translation x +1.5, y +24.8 cm
applied (rpy_xy): rotvec x(pitch) -0.690, y(roll) -0.469, z(yaw) -0.517 deg; translation x +1.5, y +24.8, z +0.0 cm; LiDAR time shift +146.0 ms -> time_offsets.lidar +0.0210 s
repeatability (first vs second half of the used scans): rotvec [0.04, 0.205, 0.319] deg, translation [4.3, 24.2, 0.0] cm, time -0.3 ms
quality: warn  (halves disagree (few turns/structure: use a longer recording with turns))
written /home/project/MY-LIVO/config/carlibration/wuhu/truck13/v002_axle/calibration.json (50 s)
frontend time base: replaced template with +0.0360; imu/ins lag +15.0 ms, corr 0.996, yaw rms 0.044 rad/s
```
