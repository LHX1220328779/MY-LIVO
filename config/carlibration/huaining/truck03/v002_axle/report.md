# huaining/truck03/v002_axle

- stage: axle (后轴对齐（RTK/INS）)
- parent: huaining/truck03/v001_import
- lineage: huaining/truck03/v001_import -> huaining/truck03/v002_axle
- created: 2026-10-30T17:06:34+08:00
- note: pipeline: axle

## inputs

- bag: /home/project/data/haibo/huaining/03/0729/ros2bag
- dataset: {'path': '/home/project/data/haibo/huaining/03/0729/ros2bag', 'kind': 'raw_split', 'segments': 12, 'duration_s': 359.99, 'has_camera': False}
- window: [0.0, 1000000000.0]
- dof: rpy_xy
- time_offset: estimated

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -4.93,
        -75.73,
        -1.06
      ],
      "drpy_deg": [
        -0.528,
        0.331,
        0.902
      ]
    },
    "back_lidar": {
      "dt_cm": [
        7.7,
        -79.74,
        7.85
      ],
      "drpy_deg": [
        -0.359,
        -0.508,
        0.878
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -4.95,
        -81.6,
        1.36
      ],
      "drpy_deg": [
        0.521,
        -0.341,
        0.852
      ]
    },
    "front_lidar": {
      "dt_cm": [
        -5.6,
        -79.22,
        0.0
      ],
      "drpy_deg": [
        0.366,
        0.503,
        0.871
      ]
    }
  },
  "camera_to_rear": {
    "dt_cm": [
      -3.97,
      -77.41,
      0.19
    ],
    "drpy_deg": [
      -0.51,
      0.355,
      0.881
    ]
  },
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.0016
    },
    "lidar_frontend": {
      "from": -0.125,
      "to": 0.0034
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
      -0.5041,
      0.3645,
      0.8788
    ],
    "solution_translation_cm": [
      -5.6,
      -79.22,
      0.0
    ],
    "correction_rotvec_deg": [
      -0.5041,
      0.3645,
      0.8788
    ],
    "correction_translation_cm": [
      -5.6,
      -79.22,
      0.0
    ],
    "correction_matrix": [
      0.9998621413317799,
      -0.015365078008545875,
      0.006293862821785841,
      0.05848158938819976,
      0.015309103865356817,
      0.9998436676586444,
      0.008847122795028379,
      -0.7960623234440911,
      -0.006428815619371538,
      -0.008749549742809257,
      0.9999410561173243,
      0.06727778047339666,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "time_shift_s": 0.1234,
    "time_offsets_used": {
      "lidar": -0.125,
      "rtk_position": 0.025,
      "ins_attitude": 0.0
    },
    "residual_cm": {
      "before": 2.227,
      "after": 1.981,
      "ground_after": 1.862,
      "vertical_after": 2.11
    },
    "sqrt_information": {
      "rx": 2249.0,
      "ry": 2735.0,
      "rz": 1761.8,
      "tx": 68.4,
      "ty": 56.8,
      "dt": 615.6
    },
    "repeatability_halves": {
      "rotvec_deg": [
        0.092,
        0.046,
        0.041
      ],
      "translation_cm": [
        1.0,
        1.8,
        0.0
      ],
      "time_ms": -1.0
    },
    "time_profile": [
      {
        "baseline_s": -0.18,
        "dt_s": -0.0711,
        "median_cm": 3.296,
        "ground_cm": 3.007
      },
      {
        "baseline_s": -0.15,
        "dt_s": -0.0243,
        "median_cm": 3.127,
        "ground_cm": 2.804
      },
      {
        "baseline_s": -0.12,
        "dt_s": 0.0164,
        "median_cm": 2.888,
        "ground_cm": 2.659
      },
      {
        "baseline_s": -0.09,
        "dt_s": 0.0527,
        "median_cm": 2.677,
        "ground_cm": 2.422
      },
      {
        "baseline_s": -0.06,
        "dt_s": 0.0754,
        "median_cm": 2.522,
        "ground_cm": 2.308
      },
      {
        "baseline_s": -0.03,
        "dt_s": 0.0918,
        "median_cm": 2.476,
        "ground_cm": 2.271
      },
      {
        "baseline_s": 0.0,
        "dt_s": 0.1007,
        "median_cm": 2.455,
        "ground_cm": 2.249
      },
      {
        "baseline_s": 0.03,
        "dt_s": 0.112,
        "median_cm": 2.379,
        "ground_cm": 2.195
      },
      {
        "baseline_s": 0.06,
        "dt_s": 0.1184,
        "median_cm": 2.321,
        "ground_cm": 2.137
      },
      {
        "baseline_s": 0.09,
        "dt_s": 0.1208,
        "median_cm": 2.182,
        "ground_cm": 1.992
      },
      {
        "baseline_s": 0.12,
        "dt_s": 0.1231,
        "median_cm": 2.094,
        "ground_cm": 1.894
      },
      {
        "baseline_s": 0.15,
        "dt_s": 0.1298,
        "median_cm": 2.204,
        "ground_cm": 2.024
      }
    ],
    "time_profile_min_dt_s": 0.1231,
    "scans_used": 457,
    "window_s": [
      36.3,
      346.0
    ],
    "dof": "rpy_xy",
    "iterations": 10,
    "converged": true,
    "quality": "warn",
    "reasons": [
      "translation correction 79 cm confirmed by independent halves (<= 3 cm)",
      "time shift +123 ms is a large change vs the parent offset (candidates/longer recording confirms it)"
    ],
    "runtime_s": 200.1
  },
  "frontend_sync": {
    "imu_ins_lag_s": 0.005000000000000282,
    "corr": 0.9992,
    "yaw_rms_rad_s": 0.1114,
    "template": -0.125,
    "decision": "replaced template with +0.0034"
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
        1.9797,
        6.6507,
        0.9916
      ],
      "rpy_deg": [
        -1.913,
        -2.372,
        -3.293
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2541,
        -1.8369,
        0.4832
      ],
      "rpy_deg": [
        0.113,
        -0.14,
        -90.232
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8823,
        6.6614,
        1.0667
      ],
      "rpy_deg": [
        -2.346,
        -3.071,
        177.839
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0239,
        6.87,
        0.5194
      ],
      "rpy_deg": [
        0.126,
        -1.028,
        90.616
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0596,
      6.6937,
      2.6443
    ],
    "rpy_deg": [
      -95.853,
      -0.046,
      -0.59
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1644.2,
      1857.01,
      956.116,
      542.763
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
    "lidar": -0.0016,
    "lidar_frontend": 0.0034,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0016 + imu/ins clock lag +0.0050 (corr 0.999); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/huaining/truck03/v001_import/calibration.json
bag=/home/project/data/haibo/huaining/03/0729/ros2bag
reference front_lidar (transform_matrix_lidar_6); poses imu_data/odometry (RTK fixed, >= 1.0 m/s, >= 5.0 s after motion start); time offsets lidar -0.1250 s, rtk_position +0.025 s, ins_attitude +0.000 s
scans read 1797, skipped {'slow': 882, 'startup': 0, 'not_rtk_fixed': 0, 'outside_ins': 2}
scans used 457 (36.3..346.0 s), points 3377443; dof rpy_xy + LiDAR time shift; cross-time gap > 1.5 s
time-offset candidates (exact re-pose baseline + local joint solve): -0.180/-0.071s:3.30cm, -0.150/-0.024s:3.13cm, -0.120/+0.016s:2.89cm, -0.090/+0.053s:2.68cm, -0.060/+0.075s:2.52cm, -0.030/+0.092s:2.48cm, +0.000/+0.101s:2.46cm, +0.030/+0.112s:2.38cm, +0.060/+0.118s:2.32cm, +0.090/+0.121s:2.18cm, +0.120/+0.123s:2.09cm, +0.150/+0.130s:2.20cm
detected time shift +0.123 s (median 2.09 cm) -> final joint solve
residual median 2.23 -> 1.98 cm (ground 2.01 -> 1.86, n=55287; vertical 2.62 -> 2.11, n=17834); match time gap median 71.2 s; iterations 10
solution about the LiDAR (rear-axle frame): rotvec x(pitch) -0.504, y(roll) +0.364, z(yaw) +0.879 deg; translation x -5.6, y -79.2 cm
applied (rpy_xy): rotvec x(pitch) -0.504, y(roll) +0.364, z(yaw) +0.879 deg; translation x -5.6, y -79.2, z +0.0 cm; LiDAR time shift +123.4 ms -> time_offsets.lidar -0.0016 s
repeatability (first vs second half of the used scans): rotvec [0.092, 0.046, 0.041] deg, translation [1.0, 1.8, 0.0] cm, time -1.0 ms
quality: warn  (translation correction 79 cm confirmed by independent halves (<= 3 cm); time shift +123 ms is a large change vs the parent offset (candidates/longer recording confirms it))
written /home/project/MY-LIVO/config/carlibration/huaining/truck03/v002_axle/calibration.json (200 s)
frontend time base: replaced template with +0.0034; imu/ins lag +5.0 ms, corr 0.999, yaw rms 0.111 rad/s
```
