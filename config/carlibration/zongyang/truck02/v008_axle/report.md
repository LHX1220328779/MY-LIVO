# zongyang/truck02/v008_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: zongyang/truck02/v001_import
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v008_axle
- created: 2026-10-27T11:55:20+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/zongyang/02/20261026/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/02/20261026/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 239.98, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -6.27,
        -43.83,
        6.04
      ],
      "drpy_deg": [
        -2.202,
        0.341,
        -0.719
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -14.71,
        -44.4,
        31.02
      ],
      "drpy_deg": [
        -0.44,
        -2.185,
        -0.711
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -6.35,
        -36.31,
        10.81
      ],
      "drpy_deg": [
        2.195,
        -0.386,
        -0.738
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -3.03,
        -37.76,
        0.0
      ],
      "drpy_deg": [
        0.489,
        2.174,
        -0.723
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -3.23,
      -38.79,
      -0.03
    ],
    "drpy_deg": [
      -2.185,
      0.452,
      -0.835
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.001
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": 0.004
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
      -2.185,
      0.4387,
      -0.7289
    ],
    "solution_translation_cm": [
      -3.03,
      -37.76,
      0.0
    ],
    "correction_rotvec_deg": [
      -2.185,
      0.4387,
      -0.7289
    ],
    "correction_translation_cm": [
      -3.03,
      -37.76,
      0.0
    ],
    "correction_matrix": [
      0.9998897735118851,
      0.012572623072266653,
      0.007897466393275182,
      -0.14388137287703917,
      -0.012864588256476923,
      0.9991920262443268,
      0.0380762006868598,
      -0.4658459515448259,
      -0.007412367728433265,
      -0.038173601334396165,
      0.9992436304354501,
      0.2873625546247962,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.124,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 3.418,
      "after": 2.799,
      "ground_after": 3.106,
      "vertical_after": 1.911
    },
    "sqrt_information": {
      "rx": 2017.5,
      "ry": 2802.4,
      "rz": 897.4,
      "tx": 51.2,
      "ty": 34.4,
      "dt": 318.3
    },
    "repeatability_halves": {
      "rotvec_deg": [
        -0.131,
        -0.007,
        0.114
      ],
      "translation_cm": [
        -0.7,
        -5.3,
        0.0
      ],
      "time_ms": -3.7
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0113,
        "median_cm": 3.542,
        "ground_cm": 3.615
      },
      {
        "baseline_s": -0.15,
        "dt_s": 0.0199,
        "median_cm": 3.31,
        "ground_cm": 3.384
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0443,
        "median_cm": 3.167,
        "ground_cm": 3.284
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0617,
        "median_cm": 3.077,
        "ground_cm": 3.247
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0736,
        "median_cm": 3.004,
        "ground_cm": 3.277
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0811,
        "median_cm": 2.997,
        "ground_cm": 3.337
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.087,
        "median_cm": 2.968,
        "ground_cm": 3.336
      },
      {
        "baseline_s": 0.03,
        "dt_s": 0.0956,
        "median_cm": 2.924,
        "ground_cm": 3.28
      },
      {
        "baseline_s": 0.06,
        "dt_s": 0.1068,
        "median_cm": 2.881,
        "ground_cm": 3.206
      },
      {
        "baseline_s": 0.09,
        "dt_s": 0.1163,
        "median_cm": 2.898,
        "ground_cm": 3.17
      }
    ],
    "time_profile_min_dt_s": 0.1068,
    "scans_used": 563,
    "window_s": [
      8.2,
      238.3
    ],
    "dof": "rpy_xy",
    "iterations": 24,
    "converged": false,
    "quality": "warn",
    "reasons": [
      "time shift +124 ms is a large change vs the parent offset (candidates/longer recording confirms it)",
      "not converged in 24 iterations"
    ],
    "runtime_s": 189.1
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.005000000000000282,
    "corr": 0.9996,
    "yaw_rms_rad_s": 0.1762,
    "template": -0.125,
    "decision": "replaced template with +0.0040"
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
        2.5189,
        4.9709,
        1.7707
      ],
      "rpy_deg": [
        -0.242,
        -0.083,
        -2.919
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2259,
        -1.0372,
        0.8423
      ],
      "rpy_deg": [
        0.442,
        -3.455,
        -90.33
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1628,
        4.899,
        1.8706
      ],
      "rpy_deg": [
        -1.719,
        -0.421,
        178.248
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0695,
        7.109,
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
      -0.1046,
      7.1192,
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
    "lidar": -0.001,
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
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck02/v001_import/calibration.json
bag=/home/project/data/haibo/zongyang/02/20261026/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 1196, skipped {'slow': 47, 'startup': 21, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 563 (8.2..238.3 s), points 3246431; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.011s:3.54cm, -0.150/+0.020s:3.31cm, -0.120/+0.044s:3.17cm, -0.090/+0.062s:3.08cm, -0.060/+0.074s:3.00cm, -0.030/+0.081s:3.00cm, +0.000/+0.087s:2.97cm, +0.030/+0.096s:2.92cm, +0.060/+0.107s:2.88cm, +0.090/+0.116s:2.90cm
detected time shift +0.107 s (median 2.88 cm) -> final joint solve
residual median 3.42 -> 2.80 cm (ground 3.66 -> 3.11, n=42344; vertical 2.21 -> 1.91, n=9968); match time gap median 72.3 s; iterations 24 (NOT converged)
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -2.185, y(roll) +0.439, z(yaw) -0.729 deg; translation x -3.0, y -37.8 cm
applied (rpy_xy): rotvec x(pitch) -2.185, y(roll) +0.439, z(yaw) -0.729 deg; translation x -3.0, y -37.8, z +0.0 cm; LiDAR time shift +124.0 ms -> time_offsets.lidar -0.0010 s
repeatability (first vs second half of the used scans): rotvec [-0.131, -0.007, 0.114] deg, translation [-0.7, -5.3, 0.0] cm, time -3.7 ms
quality: warn  (time shift +124 ms is a large change vs the parent offset (candidates/longer recording confirms it); not converged in 24 iterations)
written /home/project/MY-LIVO/config/carlibration/zongyang/truck02/v008_axle/calibration.json (189 s)
frontend time base: replaced template with +0.0040; imu/ins lag +5.0 ms, corr 1.000, yaw rms 0.176 rad/s
```
