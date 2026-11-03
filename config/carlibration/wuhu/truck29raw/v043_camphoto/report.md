# wuhu/truck29raw/v043_camphoto

- stage: camphoto (相机光度标定（内参复核 + 外参/时间精修）)
- parent: wuhu/truck29raw/v042_camloop
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v034_axle -> wuhu/truck29raw/v035_ground -> wuhu/truck29raw/v036_driven -> wuhu/truck29raw/v038_rtktime -> wuhu/truck29raw/v039_gcons -> wuhu/truck29raw/v040_camtime -> wuhu/truck29raw/v041_camlidar -> wuhu/truck29raw/v042_camloop -> wuhu/truck29raw/v043_camphoto
- created: 2026-10-21T02:57:19+08:00
- note: 相机光度标定：旋转 (相机系 x/y/z) +0.306 / -0.148 / -0.025°，时间偏移 -0.0122 s，内参复核不变（光度一致性 +27.2 %）

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: results/mcp36_livo_c60b/Log/fusion/1792520245537193957

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
      -0.311,
      0.008,
      -0.149
    ]
  },
  "time_offsets": {
    "camera": {
      "from": 0.0,
      "to": -0.0122
    }
  }
}
```

## metrics

```
{
  "camera_photometric": {
    "method": "multi-view photo-consistency of the round's returns (LCN cross-view variance)",
    "frames": 536,
    "views": 95,
    "returns": 50000,
    "cost_zero": [
      0.20396,
      0.20577
    ],
    "halves": [
      {
        "rot_deg": [
          0.3061,
          -0.1432,
          -0.0263
        ],
        "dt_s": -0.0128,
        "train_gain": 0.2796,
        "test_gain": 0.2647,
        "intrinsics": {
          "fx_scale": 0.99964,
          "fy_scale": 1.00133,
          "k1": -0.00595,
          "test_gain": 0.2706
        }
      },
      {
        "rot_deg": [
          0.3063,
          -0.1526,
          -0.0218
        ],
        "dt_s": -0.01171,
        "train_gain": 0.2654,
        "test_gain": 0.2788,
        "intrinsics": {
          "fx_scale": 0.99878,
          "fy_scale": 1.00241,
          "k1": -0.00878,
          "test_gain": 0.2783
        }
      }
    ],
    "gate": {
      "min_test_gain": 0.05,
      "halves_agree": true,
      "gain_ok": true,
      "intrinsics_extra_gain": [
        0.0058,
        -0.0005
      ],
      "intrinsics_ok": false
    },
    "applied": {
      "rot_delta_deg_cam_xyz": [
        0.3057,
        -0.1478,
        -0.0251
      ],
      "camera_time_offset_s": -0.0122,
      "dt_s": -0.01218,
      "intrinsics": false,
      "cost_gain_all": 0.2723
    }
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
      -96.787,
      1.676,
      0.336
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
    "camera": -0.0122,
    "rtk_position": 0.0151,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.84); effective +10.1 ms (se 2.0 ms)",
    "camera_source": "camera-photo: multi-view photo-consistency, +0.0000 -12.2 ms (held-out gains 26.5 / 27.9 %)"
  }
}
```

## stage report

```
collection: 536 frames with an image, 95 views, 50000 returns (>= 3 views), 448938 samples
half 0: rotation+time rx +0.306 ry -0.143 rz -0.026 deg, dt -12.8 ms: train +28.0 %, held-out +26.5 %; with intrinsics (fx -0.04 %, fy +0.13 %, k1 -0.0060) held-out +27.1 %
half 1: rotation+time rx +0.306 ry -0.153 rz -0.022 deg, dt -11.7 ms: train +26.5 %, held-out +27.9 %; with intrinsics (fx -0.12 %, fy +0.24 %, k1 -0.0088) held-out +27.8 %
applied: camera rotation (camera frame) rx +0.306 ry -0.148 rz -0.025 deg, time offset +0.0000 -> -0.0122 s; intrinsics confirmed (unchanged); cost +27.2 % on all returns (138 s)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v043_camphoto/calibration.json
```
