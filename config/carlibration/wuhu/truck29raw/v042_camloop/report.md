# wuhu/truck29raw/v042_camloop

- stage: camloop (相机外参闭环标定)
- parent: wuhu/truck29raw/v041_camlidar
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v034_axle -> wuhu/truck29raw/v035_ground -> wuhu/truck29raw/v036_driven -> wuhu/truck29raw/v038_rtktime -> wuhu/truck29raw/v039_gcons -> wuhu/truck29raw/v040_camtime -> wuhu/truck29raw/v041_camlidar -> wuhu/truck29raw/v042_camloop
- created: 2026-10-15T09:13:08+08:00
- note: 相机俯仰闭环：Δpitch=+0.001°（尺度斜率 0.098/°，沿用历史值 wuhu/truck29raw/v009_camloop，1 轮，预测原生尺度 1.0000）

## changes vs parent

```
{
  "camera_to_rear": {
    "dt_cm": [
      0.0,
      0.0,
      0.0
    ],
    "drpy_deg": [
      0.001,
      0.0,
      0.0
    ]
  }
}
```

## metrics

```
{
  "changes": {
    "camera_rot_delta_deg_cam_xyz": [
      0.001,
      0.0,
      0.0
    ]
  },
  "pitch_fit": {
    "ok": true,
    "points": [
      {
        "round": "calib_truck29raw_livo_1015_090255",
        "pitch_delta_deg": 0.0,
        "path_length_ratio": 0.9999,
        "duration_s": 57.7,
        "native_drift_m": 0.5189
      }
    ],
    "target": 1.0,
    "errors": [],
    "slope_per_deg": 0.098,
    "slope_source": "model/previous run",
    "intercept": 0.9999,
    "rms": 0.0,
    "pitch_delta_deg": 0.001,
    "clamped": false,
    "predicted_ratio": 1.0,
    "reason": "zero crossing with the known slope"
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
        2.7755,
        5.9149,
        1.8312
      ],
      "rpy_deg": [
        -1.031,
        -3.057,
        -1.219
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0611,
        -0.2984,
        0.7111
      ],
      "rpy_deg": [
        -1.047,
        0.481,
        -91.365
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0915,
        5.6621,
        2.0104
      ],
      "rpy_deg": [
        3.243,
        -4.537,
        178.24
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.106,
        7.9492,
        0.051
      ],
      "rpy_deg": [
        1.485,
        -1.56,
        89.984
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0067,
      7.9506,
      1.7516
    ],
    "rpy_deg": [
      -96.476,
      1.668,
      0.485
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
    "lidar": -0.0361,
    "camera": 0.0,
    "rtk_position": 0.0151,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.84); effective +10.1 ms (se 2.0 ms)"
  }
}
```
