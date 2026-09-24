# wuhu/truck29raw/v009_camloop

- stage: camloop (相机外参闭环标定)
- parent: wuhu/truck29raw/v006_camlidar
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle -> wuhu/truck29raw/v003_ground -> wuhu/truck29raw/v004_driven -> wuhu/truck29raw/v005_camtime -> wuhu/truck29raw/v006_camlidar -> wuhu/truck29raw/v009_camloop
- created: 2026-09-24T10:28:04+08:00
- note: 相机俯仰闭环：Δpitch=+0.012°（尺度斜率 0.098/°，3 轮，预测原生尺度 1.0000）

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
      0.013,
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
      0.0122,
      0.0,
      0.0
    ]
  },
  "pitch_fit": {
    "ok": true,
    "points": [
      {
        "round": "calib_truck29raw_livo_0924_101925",
        "pitch_delta_deg": 0.0,
        "path_length_ratio": 0.999,
        "duration_s": 117.83
      },
      {
        "round": "calib_truck29raw_livo_0924_101925_scan_up",
        "pitch_delta_deg": 0.05,
        "path_length_ratio": 1.0036,
        "duration_s": 112.43
      },
      {
        "round": "calib_truck29raw_livo_0924_101925_scan_dn",
        "pitch_delta_deg": -0.05,
        "path_length_ratio": 0.9938,
        "duration_s": 112.47
      }
    ],
    "target": 1.0,
    "errors": [],
    "slope_per_deg": 0.098,
    "intercept": 0.9988,
    "rms": 0.000141,
    "pitch_delta_deg": 0.0122,
    "clamped": false,
    "predicted_ratio": 1.0,
    "reason": "zero crossing of native scale vs camera pitch"
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
        2.756,
        5.9895,
        1.8333
      ],
      "rpy_deg": [
        -1.001,
        -3.026,
        -1.433
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1094,
        -0.311,
        0.6862
      ],
      "rpy_deg": [
        -1.079,
        0.746,
        -91.392
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1269,
        5.7908,
        1.9755
      ],
      "rpy_deg": [
        -3.418,
        -0.801,
        178.584
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
      -96.497,
      1.617,
      0.258
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
    "camera": -0.1211,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
